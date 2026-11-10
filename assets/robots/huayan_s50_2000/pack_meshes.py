"""Pack CAD meshes losslessly for Git transport without changing CAD evidence.

Uses only the standard library. Original OBJ files are retained locally; their
paths and hashes in cad_audit.json remain authoritative. --unpack restores them
from the published gzip files and refuses to overwrite a different local file.
"""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ASSET = ROOT / "assets/robots/huayan_s50_2000"
DERIVED = ASSET / "derived"
MANIFEST = DERIVED / "compressed_meshes.json"


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def asset_path(relative):
    path = (ROOT / relative).resolve()
    path.relative_to(DERIVED.resolve())
    return path


def compress(data):
    stream = io.BytesIO()
    with gzip.GzipFile(filename="", mode="wb", fileobj=stream, compresslevel=9, mtime=0) as output:
        output.write(data)
    return stream.getvalue()


def pack():
    audit_path = DERIVED / "cad_audit.json"
    audit_bytes = audit_path.read_bytes()
    audit = json.loads(audit_bytes)
    meshes = []
    for part in audit["parts"]:
        raw_path = asset_path(part["mesh"])
        if raw_path.suffix != ".obj":
            raise ValueError("EXPECTED_OBJ:" + str(raw_path))
        raw = raw_path.read_bytes()
        if sha256(raw) != part["mesh_sha256"]:
            raise ValueError("CAD_MESH_HASH_MISMATCH:" + part["id"])
        packed = compress(raw)
        if gzip.decompress(packed) != raw:
            raise ValueError("LOSSLESS_ROUND_TRIP_FAILED:" + part["id"])
        if compress(raw) != packed:
            raise ValueError("NONDETERMINISTIC_GZIP:" + part["id"])
        destination = raw_path.with_suffix(".obj.gz")
        destination.write_bytes(packed)
        meshes.append(dict(id=part["id"], raw_path=part["mesh"],
                           raw_sha256=part["mesh_sha256"], raw_bytes=len(raw),
                           gzip_path=destination.relative_to(ROOT).as_posix(),
                           gzip_sha256=sha256(packed), gzip_bytes=len(packed),
                           decompressed_matches_raw=True))
    report = dict(schema="s50_lossless_mesh_transport_v1",
                  cad_audit_path=audit_path.relative_to(ROOT).as_posix(),
                  cad_audit_sha256=sha256(audit_bytes),
                  compression="gzip", compression_level=9, header_mtime=0,
                  header_filename="", meshes=meshes,
                  total_raw_bytes=sum(m["raw_bytes"] for m in meshes),
                  total_gzip_bytes=sum(m["gzip_bytes"] for m in meshes),
                  maximum_gzip_bytes=max(m["gzip_bytes"] for m in meshes),
                  all_round_trips_exact=True, original_obj_files_removed=False,
                  publication="Git tracks *.obj.gz; raw *.obj paths/hashes stay unchanged in cad_audit.json.")
    MANIFEST.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
    if audit_path.read_bytes() != audit_bytes:
        raise ValueError("CAD_AUDIT_WAS_CHANGED")
    return report


def unpack():
    report = json.loads(MANIFEST.read_text("utf8"))
    if sha256(asset_path(report["cad_audit_path"]).read_bytes()) != report["cad_audit_sha256"]:
        raise ValueError("CAD_AUDIT_HASH_MISMATCH")
    for mesh in report["meshes"]:
        packed = asset_path(mesh["gzip_path"]).read_bytes()
        if sha256(packed) != mesh["gzip_sha256"] or len(packed) != mesh["gzip_bytes"]:
            raise ValueError("PACKED_MESH_HASH_OR_SIZE_MISMATCH:" + mesh["id"])
        raw = gzip.decompress(packed)
        if sha256(raw) != mesh["raw_sha256"] or len(raw) != mesh["raw_bytes"]:
            raise ValueError("RAW_MESH_HASH_OR_SIZE_MISMATCH:" + mesh["id"])
        destination = asset_path(mesh["raw_path"])
        if destination.exists():
            if destination.read_bytes() != raw:
                raise ValueError("REFUSE_DIFFERENT_EXISTING_OBJ:" + mesh["id"])
        else:
            destination.write_bytes(raw)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--unpack", action="store_true")
    arguments = parser.parse_args()
    result = unpack() if arguments.unpack else pack()
    print(json.dumps({key: result[key] for key in (
        "total_raw_bytes", "total_gzip_bytes", "maximum_gzip_bytes", "all_round_trips_exact")}, indent=2))
