"""Audit S50 STEP topology and export CAD-space leaf meshes, never infer control zeros.

Optional preparation dependencies: cadquery-ocp-novtk 7.9.3.1.1, numpy, trimesh.
This is an evidence converter, not an articulated robot model constructor.
"""
import argparse, hashlib, json, re
from pathlib import Path
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[3]

def convert(source: Path, output: Path, repair: bool = False) -> None:
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_LabelSequence, TDF_Label
    from OCP.TDataStd import TDataStd_Name
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.Interface import Interface_Static
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.ShapeFix import ShapeFix_Shape
    import OCP

    source, output = source.resolve(), output.resolve()
    source.relative_to(ROOT)
    output.relative_to(ROOT / "assets/robots/huayan_s50_2000/derived")
    data = source.read_bytes()
    sha = hashlib.sha256(data).hexdigest()
    text = data.decode("utf8", errors="replace")
    declarations = sorted(set(re.findall(r"SI_UNIT\s*\(\s*([^,]+),\s*\.METRE\.\s*\)", text)))
    if declarations != [".MILLI."]:
        raise ValueError("UNREVIEWED_STEP_UNITS:" + str(declarations))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise ValueError("STEP_READ_FAILED")
    reader.ChangeReader().SetSystemLengthUnit(1.)
    doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
    if not reader.Transfer(doc):
        raise ValueError("XCAF_TRANSFER_FAILED")
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    free = TDF_LabelSequence()
    st.GetFreeShapes(free)
    output.mkdir(parents=True, exist_ok=True)
    nodes, parts = [], []

    def name(label):
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ""

    def matrix(loc):
        transform = loc.Transformation()
        result = np.eye(4)
        for i in range(3):
            for j in range(4):
                result[i, j] = transform.Value(i + 1, j + 1)
        result[:3, 3] *= .001
        return result.tolist()

    def xyz(point, scale=.001):
        return [point.X() * scale, point.Y() * scale, point.Z() * scale]

    def walk(label, parent_loc, path):
        world = parent_loc.Multiplied(st.GetLocation_s(label))
        definition = label
        if st.IsReference_s(label):
            definition = TDF_Label()
            if not st.GetReferredShape_s(label, definition):
                raise ValueError("XCAF_REFERENCE_MISSING")
        key = path + "/" + str(label.Tag())
        assembly = st.IsAssembly_s(definition)
        definition_name = name(definition)
        nodes.append(dict(id=key, name=name(label), definition_name=definition_name,
                          T_CAD_instance=matrix(world), assembly=assembly))
        if assembly:
            seq = TDF_LabelSequence()
            st.GetComponents_s(definition, seq, False)
            for i in range(1, seq.Length() + 1):
                walk(seq.Value(i), world, key)
            return
        shape = st.GetShape_s(definition).Moved(world)
        original_valid = bool(BRepCheck_Analyzer(shape).IsValid())
        original_bounds = Bnd_Box()
        BRepBndLib.AddOptimal_s(shape, original_bounds, False, False)
        original_bbox = np.array(original_bounds.Get()).reshape(2, 3) * .001
        repair_performed = False
        if repair:
            fixer = ShapeFix_Shape(shape)
            repair_performed = bool(fixer.Perform())
            shape = fixer.Shape()
        bounds = Bnd_Box()
        BRepBndLib.AddOptimal_s(shape, bounds, False, False)
        bbox = np.array(bounds.Get()).reshape(2, 3) * .001
        solids = 0
        ex = TopExp_Explorer(shape, TopAbs_SOLID)
        while ex.More():
            solids += 1
            ex.Next()
        BRepMesh_IncrementalMesh(shape, .10, False, .15, True)
        vertices, triangles, planes, cylinders, missing_faces = [], [], [], [], []
        ex = TopExp_Explorer(shape, TopAbs_FACE)
        face_index = 0
        while ex.More():
            face = TopoDS.Face_s(ex.Current())
            tri_loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(face, tri_loc)
            offset = len(vertices)
            if tri is not None:
                for i in range(1, tri.NbNodes() + 1):
                    vertices.append(xyz(tri.Node(i).Transformed(tri_loc.Transformation())))
                for i in range(1, tri.NbTriangles() + 1):
                    indices = list(tri.Triangle(i).Get())
                    if face.Orientation() == TopAbs_REVERSED:
                        indices.reverse()
                    triangles.append([offset + value - 1 for value in indices])
            surface = BRepAdaptor_Surface(face, True)
            props = GProp_GProps()
            BRepGProp.SurfaceProperties_s(face, props)
            common = dict(face_index=face_index, area_m2=props.Mass() * 1e-6,
                          surface_centroid_m=xyz(props.CentreOfMass()))
            if tri is None:
                missing_faces.append(dict(**common, surface_type=str(surface.GetType())))
            if surface.GetType() == GeomAbs_Plane:
                plane = surface.Plane()
                n = np.array(xyz(plane.Axis().Direction(), 1.))
                if face.Orientation() == TopAbs_REVERSED:
                    n = -n
                planes.append(dict(**common, origin_m=xyz(plane.Location()), normal=n.tolist()))
            elif surface.GetType() == GeomAbs_Cylinder:
                cylinder = surface.Cylinder()
                cylinders.append(dict(**common, origin_m=xyz(cylinder.Location()),
                                      direction=xyz(cylinder.Axis().Direction(), 1.),
                                      radius_m=cylinder.Radius() * .001))
            ex.Next()
            face_index += 1
        match = re.search(r"J([1-6])", definition_name)
        part_id = "j" + match.group(1) + "_s50" if match else "base_s50"
        if any(part["id"] == part_id for part in parts):
            raise ValueError("DUPLICATED_LEAF_NAME:" + definition_name)
        mesh = trimesh.Trimesh(vertices, triangles, process=True)
        destination = output / (part_id + "_cad.obj")
        mesh.export(destination)
        parts.append(dict(id=part_id, name=definition_name, instance=key, solids=solids,
                          source_topology_valid=original_valid,
                          shape_fix_requested=repair, shape_fix_performed=repair_performed,
                          original_geometry_bounds_m=original_bbox.tolist(),
                          geometry_bounds_change_max_m=float(np.max(np.abs(original_bbox - bbox))),
                          topology_valid=bool(BRepCheck_Analyzer(shape).IsValid()),
                          mesh=destination.relative_to(ROOT).as_posix(),
                          mesh_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
                          mesh_vertices=len(mesh.vertices), mesh_triangles=len(mesh.faces),
                          geometry_bounds_m=bbox.tolist(), mesh_bounds_m=mesh.bounds.tolist(),
                          missing_tessellation_faces=missing_faces,
                          mesh_complete=not missing_faces,
                          cad_placement=matrix(world), planar_faces=planes,
                          cylindrical_faces=cylinders))
    for i in range(1, free.Length() + 1):
        walk(free.Value(i), TopLoc_Location(), "root")
    entity_names = re.findall(r"#[0-9]+\s*=\s*([A-Z][A-Z_0-9]+)\s*\(", text)
    report = dict(schema="s50_cad_topology_audit_v1", source=source.relative_to(ROOT).as_posix(),
                  source_sha256=sha, source_bytes=len(data), source_units="millimetres",
                  normalization_scale_to_m=.001, reader="OCP " + OCP.__version__,
                  repair_method="OCCT ShapeFix_Shape.Perform" if repair else None,
                  mesh_linear_deflection_m=.0001, mesh_angular_deflection_rad=.15,
                  coordinate_semantics="CAD assembly frame; not controller zero or URDF joint frames",
                  articulated_model_ready=False, assembly_nodes=nodes, parts=parts,
                  kinematic_entity_types=sorted({x for x in entity_names if any(
                      token in x for token in ("KINEMATIC", "REVOLUTE", "MECHANISM", "ROTATION_JOINT"))}),
                  assertions=dict(source_unchanged=hashlib.sha256(source.read_bytes()).hexdigest() == sha,
                                  all_topology_valid=all(p["topology_valid"] for p in parts),
                                  seven_distinct_named_bodies=len(parts) == 7),
                  limitations=["STEP assembly placements are rigid CAD reference placements, not proof of controller zero.",
                               "Cylinder axes are geometric measurements; selecting each actual revolute axis and sign requires documented interpretation.",
                               "No per-link inertials, effort limits, joint signs, zero offsets or control references inferred."])
    (output / "cad_audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf8")
    print(json.dumps(dict(source_sha256=sha, part_count=len(parts),
                          parts=[{k:p[k] for k in ("id", "name", "solids", "topology_valid", "geometry_bounds_m", "mesh_vertices", "mesh_triangles")} for p in parts]),
                     ensure_ascii=False, indent=2))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "assets/robots/huayan_s50_2000/source/S50.STEP")
    parser.add_argument("--output", type=Path, default=ROOT / "assets/robots/huayan_s50_2000/derived")
    parser.add_argument("--repair", action="store_true", help="Record OCCT ShapeFix explicitly; use a distinct output directory.")
    args = parser.parse_args()
    convert(args.source, args.output, args.repair)
