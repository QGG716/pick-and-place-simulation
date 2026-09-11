"""Regenerate the Wantai per-solid coverage record from its source STEP.

Run once in an isolated CPU CAD environment.  Normal planning only consumes
the small hash-bound record and does not depend on OpenCascade.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import re
import sys

import numpy as np


def _bounds_values(bounds):
    lower, upper = bounds.CornerMin(), bounds.CornerMax()
    return [lower.X(), lower.Y(), lower.Z(), upper.X(), upper.Y(), upper.Z()]


def _extent_matches(actual, expected):
    # Product definitions have local frames; placed occurrence extents were
    # independently checked in the common STEP frame above.
    return np.allclose(np.sort(actual), np.sort(expected), atol=0.002, rtol=0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.project_root.resolve()
    spec = importlib.util.spec_from_file_location("wantai_geometry", root / "src/unloading_sim/tool_geometry.py")
    assert spec is not None and spec.loader is not None
    geometry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(geometry)
    BOUND_PAD_MM = geometry.BOUND_PAD_MM
    COVERAGE_SCHEMA = geometry.COVERAGE_SCHEMA
    classify_tool_solids = geometry.classify_tool_solids
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    import OCP

    step_path = root / "res/上海皖泰真空吸盘三分区.STEP"
    analysis_path = root / "assets/grippers/shanghai_wantai_three_zone/mass_properties.json"
    analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
    classified = classify_tool_solids(analysis)
    reader = STEPControl_Reader()
    if reader.ReadFile(str(step_path)) != IFSelect_RetDone or reader.TransferRoots() <= 0:
        raise RuntimeError("STEP transfer failed")
    solids = TopExp_Explorer(reader.OneShape(), TopAbs_SOLID)
    rows = []
    source_shapes = []
    while solids.More():
        shape = solids.Current()
        bounds = Bnd_Box()
        # Analytic geometry extrema, no triangle-mesh approximation and no
        # tolerance inflation; runtime applies an explicit outward pad.
        BRepBndLib.AddOptimal_s(shape, bounds, False, False)
        rows.append(_bounds_values(bounds))
        source_shapes.append(shape)
        solids.Next()
    regenerated = np.asarray(rows, dtype=float)
    stored = classified["all_bounds_step_mm"]
    if regenerated.shape != stored.shape or np.max(np.abs(regenerated - stored)) > BOUND_PAD_MM:
        raise RuntimeError("source STEP extrema differ from recorded per-solid geometry")
    print(f"Verified {len(rows)} placed solid analytic bounds", flush=True)

    # The STEP transfer map resolves the named product to its two physical
    # solid definitions.  Repeated occurrence placement is verified above.
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDF import TDF_LabelSequence
    caf = STEPCAFControl_Reader()
    caf.SetNameMode(True)
    if caf.ReadFile(str(step_path)) != IFSelect_RetDone:
        raise RuntimeError("STEP product transfer failed")
    document = TDocStd_Document(TCollection_ExtendedString("WantaiCoverage"))
    if not caf.Transfer(document):
        raise RuntimeError("STEP product document transfer failed")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    labels = TDF_LabelSequence()
    shape_tool.GetShapes(labels)
    product_shapes = []
    product_records = []
    for number in range(1, labels.Length() + 1):
        label = labels.Value(number)
        # Instances are checked above.  Product attribution needs only leaf
        # definitions; walking all assembly solids again needlessly repeats
        # expensive exact spline-extrema queries for every occurrence.
        if shape_tool.IsAssembly_s(label) or shape_tool.IsReference_s(label):
            continue
        name_attribute = TDataStd_Name()
        if not label.FindAttribute(TDataStd_Name.GetID_s(), name_attribute):
            continue
        name = name_attribute.Get().ToExtString()
        print(f"Inspecting source product {name}", flush=True)
        product = shape_tool.GetShape_s(label)
        product_solids = TopExp_Explorer(product, TopAbs_SOLID)
        while product_solids.More():
            shape = product_solids.Current()
            bounds = Bnd_Box()
            BRepBndLib.AddOptimal_s(shape, bounds, False, False)
            values = np.asarray(_bounds_values(bounds))
            extent = values[3:] - values[:3]
            if _extent_matches(extent, [43.7, 47.5, 43.7]) or _extent_matches(extent, [18.8, 30.85, 18.8]):
                product_shapes.append(extent)
                product_records.append({"product": name, "solid_extent_mm": extent.tolist()})
            product_solids.Next()
    large = [v for v in product_shapes if _extent_matches(v, [43.7, 47.5, 43.7])]
    small = [v for v in product_shapes if _extent_matches(v, [18.8, 30.85, 18.8])]
    named_bellows = any(
        record["product"].lower() == "fg42_1"
        and _extent_matches(record["solid_extent_mm"], [43.7, 47.5, 43.7])
        for record in product_records
    )
    if not large or not small or not named_bellows:
        raise RuntimeError(f"FG42 product does not contain the expected bellows/insert geometry: {product_records}")
    text = step_path.read_text(encoding="latin1")
    if "SI_UNIT ( .MILLI., .METRE. )" not in text:
        raise RuntimeError("STEP source must declare millimetres")
    result = {
        "schema": COVERAGE_SCHEMA,
        "source_step_sha256": hashlib.sha256(step_path.read_bytes()).hexdigest(),
        "analysis_sha256": hashlib.sha256(analysis_path.read_bytes()).hexdigest(),
        "generator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "source_units": "millimetre",
        "runtime_units": "metre",
        "generator": "BRepBndLib.AddOptimal_s(useTriangulation=False,useShapeTolerance=False)",
        "python_version": platform.python_version(),
        "ocp_version": getattr(OCP, "__version__", "unknown"),
        "product_attribution_verified": True,
        "product_records": product_records,
        "solid_occurrence_count": len(rows),
        "regenerated_solid_bounds_step_mm": rows,
        "bellows_solid_indices": classified["bellows_solid_indices"],
        "rigid_insert_solid_indices": classified["rigid_insert_solid_indices"],
        "maximum_bound_reconstruction_error_mm": float(np.max(np.abs(stored - regenerated))),
        "classification_assumption": "named FG42 outer bellows is compliant; small inner insert and all other source solids retained rigid; no manufacturer material certificate claimed",
        "uncertain_non_cup_solids_policy": "retained_rigid",
        "outward_pad_mm": BOUND_PAD_MM,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "regenerated_solid_bounds_step_mm"}, indent=2))


if __name__ == "__main__":
    main()
