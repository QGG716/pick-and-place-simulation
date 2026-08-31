"""Extract auditable gripper geometry and uniform-density mass properties from STEP.

This is an offline asset-preparation utility.  ``cadquery-ocp`` is intentionally
an optional tool dependency and is not imported by the core simulator.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from OCP.Bnd import Bnd_Box
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS


def _matrix(matrix) -> np.ndarray:
    return np.asarray(
        [[matrix.Value(row, column) for column in (1, 2, 3)] for row in (1, 2, 3)],
        dtype=float,
    )


def _round_list(values, digits: int = 9):
    return np.round(np.asarray(values, dtype=float), digits).tolist()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("step", type=Path)
    parser.add_argument("--mass-kg", type=float, required=True)
    parser.add_argument("--cup-radius-mm", type=float, default=21.5)
    parser.add_argument("--cup-compression-mm", type=float, default=15.0)
    parser.add_argument("--expected-cups", type=int, default=72)
    parser.add_argument("--zones", type=int, default=3)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--stl", type=Path)
    args = parser.parse_args()
    if args.mass_kg <= 0.0 or not math.isfinite(args.mass_kg):
        raise ValueError("mass must be finite and positive")

    reader = STEPControl_Reader()
    if reader.ReadFile(str(args.step.resolve())) != IFSelect_RetDone:
        raise RuntimeError(f"failed to read STEP file: {args.step}")
    if reader.TransferRoots() <= 0:
        raise RuntimeError("STEP file contains no transferable roots")
    shape = reader.OneShape()

    topology_bounds = Bnd_Box()
    BRepBndLib.Add_s(shape, topology_bounds)
    topology_bbox_mm = np.asarray(topology_bounds.Get(), dtype=float)

    bounds = Bnd_Box()
    solid_count = 0
    solid_bounds_mm: list[list[float]] = []
    solids = TopExp_Explorer(shape, TopAbs_SOLID)
    while solids.More():
        solid = solids.Current()
        BRepBndLib.AddOptimal_s(solid, bounds, False, False)
        solid_bounds = Bnd_Box()
        BRepBndLib.AddOptimal_s(solid, solid_bounds, False, False)
        solid_bounds_mm.append(_round_list(solid_bounds.Get(), 6))
        solid_count += 1
        solids.Next()
    bbox_mm = np.asarray(bounds.Get(), dtype=float)
    bbox_min_mm, bbox_max_mm = bbox_mm[:3], bbox_mm[3:]

    volume_properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, volume_properties)
    volume_mm3 = float(volume_properties.Mass())
    if volume_mm3 <= 0.0:
        raise RuntimeError("STEP shape has no positive solid volume")
    center = volume_properties.CentreOfMass()
    com_step_mm = np.asarray([center.X(), center.Y(), center.Z()], dtype=float)
    inertia_step_kg_m2 = (
        _matrix(volume_properties.MatrixOfInertia())
        / volume_mm3
        * args.mass_kg
        * 1e-6
    )

    cup_centers_xz: set[tuple[float, float]] = set()
    flange_planes: list[tuple[float, float, np.ndarray]] = []
    faces = TopExp_Explorer(shape, TopAbs_FACE)
    while faces.More():
        face = TopoDS.Face_s(faces.Current())
        surface = BRepAdaptor_Surface(face)
        surface_type = surface.GetType()
        if surface_type == GeomAbs_Cylinder:
            cylinder = surface.Cylinder()
            direction = cylinder.Axis().Direction()
            if (
                abs(direction.Y()) > 0.98
                and abs(cylinder.Radius() - args.cup_radius_mm) <= 0.05
            ):
                location = cylinder.Location()
                cup_centers_xz.add((round(location.X(), 6), round(location.Z(), 6)))
        elif surface_type == GeomAbs_Plane:
            direction = surface.Plane().Axis().Direction()
            if abs(direction.Y()) > 0.98:
                properties = GProp_GProps()
                BRepGProp.SurfaceProperties_s(face, properties)
                face_center = properties.CentreOfMass()
                flange_planes.append(
                    (
                        float(properties.Mass()),
                        float(face_center.Y()),
                        np.asarray([face_center.X(), face_center.Z()], dtype=float),
                    )
                )
        faces.Next()

    cup_centers = np.asarray(sorted(cup_centers_xz), dtype=float)
    if len(cup_centers) != args.expected_cups:
        raise RuntimeError(
            f"detected {len(cup_centers)} cup centres, expected {args.expected_cups}"
        )
    columns = np.unique(np.round(cup_centers[:, 0], 6))
    rows = np.unique(np.round(cup_centers[:, 1], 6))
    array_center_xz_mm = np.mean(cup_centers, axis=0)

    # Each FG42 occurrence is represented by two solids in this assembly: the
    # compliant cup and its insert.  Keep both out of the rigid PhysX collision
    # proxy so the configured 15 mm compression is not replaced by an
    # undeformable CAD envelope.  The suction/contact model handles these
    # solids separately at the detected cup centres.
    deformable_cup_solid_indices: list[int] = []
    for index, solid_bbox in enumerate(solid_bounds_mm):
        solid_bbox_array = np.asarray(solid_bbox, dtype=float)
        solid_center_xz = np.asarray(
            [
                0.5 * (solid_bbox_array[0] + solid_bbox_array[3]),
                0.5 * (solid_bbox_array[2] + solid_bbox_array[5]),
            ]
        )
        if (
            solid_bbox_array[1] > 0.0
            and np.min(np.linalg.norm(cup_centers - solid_center_xz, axis=1)) < 0.1
        ):
            deformable_cup_solid_indices.append(index)
    expected_deformable_solids = 2 * args.expected_cups
    if len(deformable_cup_solid_indices) != expected_deformable_solids:
        raise RuntimeError(
            "detected "
            f"{len(deformable_cup_solid_indices)} compliant cup solids, "
            f"expected {expected_deformable_solids}"
        )
    deformable_cup_solid_index_set = set(deformable_cup_solid_indices)
    rigid_collision_bounds_mm = [
        solid_bbox
        for index, solid_bbox in enumerate(solid_bounds_mm)
        if index not in deformable_cup_solid_index_set
    ]

    # The installation image shows the J6 flange centred on the cup array and
    # opposite the cup working direction. Select the uppermost large planar
    # mounting face sharing that centre. This is an inference, not a metrology
    # certificate, and is recorded as such in the output.
    centred_planes = [
        plane
        for plane in flange_planes
        if plane[0] > 1000.0
        and np.linalg.norm(plane[2] - array_center_xz_mm) < 5.0
        and plane[1] < com_step_mm[1]
    ]
    if not centred_planes:
        raise RuntimeError("could not infer a centred robot mounting plane")
    flange_plane_y_mm = min(centred_planes, key=lambda item: item[1])[1]
    flange_origin_step_mm = np.asarray(
        [array_center_xz_mm[0], flange_plane_y_mm, array_center_xz_mm[1]], dtype=float
    )

    # Isaac's imported FANUC J6 uses +X toward the cups.  Match the geometric
    # planner's in-plane clocking with tool +Y along STEP +X (array length)
    # and tool +Z along STEP -Z (negative array width).  The sign preserves a
    # right-handed frame; flange clocking remains provisional.
    rotation_step_from_tool = np.asarray(
        [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]], dtype=float
    )
    com_tool_m = rotation_step_from_tool.T @ (com_step_mm - flange_origin_step_mm) * 1e-3
    inertia_tool_kg_m2 = rotation_step_from_tool.T @ inertia_step_kg_m2 @ rotation_step_from_tool
    principal_moments, principal_axes = np.linalg.eigh(inertia_tool_kg_m2)
    tool_length_m = float((bbox_max_mm[1] - flange_plane_y_mm) * 1e-3)
    cup_offsets_tool_yz_m = np.column_stack(
        (
            (cup_centers[:, 0] - array_center_xz_mm[0]) * 1e-3,
            -(cup_centers[:, 1] - array_center_xz_mm[1]) * 1e-3,
        )
    )

    column_groups = np.array_split(columns, args.zones)
    zones = []
    for index, group in enumerate(column_groups):
        mask = np.isin(np.round(cup_centers[:, 0], 6), group)
        zones.append(
            {
                "zone": index,
                "column_step_x_mm": _round_list(group, 6),
                "cup_count": int(np.sum(mask)),
                "assignment_source": "equal_contiguous_columns_inferred_pending_vendor_confirmation",
            }
        )

    result = {
        "format": "gripper_step_analysis_v1",
        "source_step": str(args.step),
        "mass_kg": args.mass_kg,
        "mass_property_assumption": (
            "uniform density over all STEP solid volume scaled to supplied total mass; "
            "replace with assembly material mass properties when available"
        ),
        "step_axes": {"length": "+X", "cup_normal": "+Y", "width": "+Z"},
        "bbox_step_min_mm": _round_list(bbox_min_mm, 6),
        "bbox_step_max_mm": _round_list(bbox_max_mm, 6),
        "bbox_step_size_mm": _round_list(bbox_max_mm - bbox_min_mm, 6),
        "solid_occurrence_count": solid_count,
        "solid_bounding_boxes_step_mm": solid_bounds_mm,
        "deformable_cup_solid_indices": deformable_cup_solid_indices,
        "rigid_collision_bounding_boxes_step_mm": rigid_collision_bounds_mm,
        "collision_proxy_method": (
            "axis-aligned bounding box per rigid STEP solid; 144 compliant FG42 cup/insert "
            "solids excluded and represented by per-sealed-cup surface-gripper contacts"
        ),
        "raw_topology_bbox_step_min_mm": _round_list(topology_bbox_mm[:3], 6),
        "raw_topology_bbox_step_max_mm": _round_list(topology_bbox_mm[3:], 6),
        "raw_topology_warning": (
            "raw STEP contains non-solid topology beyond the valid solid envelope; "
            "mass, TCP, visual mesh, and collision geometry use solids only"
        ),
        "solid_volume_mm3": volume_mm3,
        "uniform_density_kg_m3": args.mass_kg / (volume_mm3 * 1e-9),
        "center_of_mass_step_mm": _round_list(com_step_mm, 6),
        "inertia_at_com_step_kg_m2": _round_list(inertia_step_kg_m2, 9),
        "flange_origin_step_mm": _round_list(flange_origin_step_mm, 6),
        "flange_inference": (
            "centred upper large planar face selected from STEP; axis and centring agree "
            "with supplied installation image; flange clocking remains provisional"
        ),
        "rotation_step_from_tool": _round_list(rotation_step_from_tool, 6),
        "tool_axis_convention": {
            "+X": "flange toward suction working plane",
            "+Y": "array length",
            "+Z": "negative array width",
        },
        "tool_length_flange_to_uncompressed_cup_extreme_m": tool_length_m,
        "center_of_mass_from_flange_tool_m": _round_list(com_tool_m, 9),
        "inertia_at_com_tool_kg_m2": _round_list(inertia_tool_kg_m2, 9),
        "principal_moments_kg_m2": _round_list(principal_moments, 9),
        "principal_axes_columns_tool": _round_list(principal_axes, 9),
        "maximum_self_gravity_moment_about_flange_nm": float(
            args.mass_kg * 9.81 * np.linalg.norm(com_tool_m)
        ),
        "lateral_com_eccentricity_from_flange_axis_m": float(np.linalg.norm(com_tool_m[1:])),
        "cup_model": "FG42",
        "cup_count": len(cup_centers),
        "cup_rows": len(rows),
        "cup_columns": len(columns),
        "cup_pitch_width_mm": float(np.median(np.diff(rows))),
        "cup_pitch_length_mm": float(np.median(np.diff(columns))),
        "cup_radius_from_step_mm": args.cup_radius_mm,
        "cup_compression_m": args.cup_compression_mm * 1e-3,
        "cup_centers_tool_yz_m": _round_list(cup_offsets_tool_yz_m, 6),
        "cup_seal_envelope_length_width_m": [
            float((np.ptp(columns) + 2.0 * args.cup_radius_mm) * 1e-3),
            float((np.ptp(rows) + 2.0 * args.cup_radius_mm) * 1e-3),
        ],
        "zones": zones,
        "limits": {
            "allowable_eccentric_payload": None,
            "allowable_peel_bending_moment_nm": None,
            "reason": "allowable failure envelope cannot be derived from geometry alone",
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    if args.stl is not None:
        args.stl.parent.mkdir(parents=True, exist_ok=True)
        mesher = BRepMesh_IncrementalMesh(shape, 0.5, False, 0.2, True)
        mesher.Perform()
        if not mesher.IsDone():
            raise RuntimeError("OCC triangulation failed")
        writer = StlAPI_Writer()
        writer.ASCIIMode = False
        if not writer.Write(shape, str(args.stl.resolve())):
            raise RuntimeError(f"failed to write STL: {args.stl}")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
