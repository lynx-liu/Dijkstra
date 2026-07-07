#!/usr/bin/env python3
"""Extract a regional subset from china.mmlp.bin and build index sidecars."""

import argparse
import struct
import subprocess
import sys
from pathlib import Path

MAGIC = b"MMLPGRPH"
NODE_RECORD = 28
EDGE_RECORD = 44


def parse_bbox(text):
    parts = [float(x.strip()) for x in text.split(",")]
    if len(parts) != 4:
        raise ValueError("bbox: minLon,minLat,maxLon,maxLat")
    return parts[0], parts[1], parts[2], parts[3]


def read_graph(path: Path):
    with path.open("rb") as f:
        if f.read(8) != MAGIC:
            raise SystemExit(f"invalid graph magic: {path}")
        version = struct.unpack("<I", f.read(4))[0]
        if version != 1:
            raise SystemExit(f"unsupported version: {version}")
        node_count, edge_count = struct.unpack("<QQ", f.read(16))
        nodes = []
        for _ in range(node_count):
            nid, lat, lon, kind = struct.unpack("<qddi", f.read(NODE_RECORD))
            nodes.append((nid, lat, lon, kind))
        edges = []
        for _ in range(edge_count):
            eid, fr, to, etype, length, speed = struct.unpack("<qqqidd", f.read(EDGE_RECORD))
            edges.append((eid, fr, to, etype, length, speed))
    return nodes, edges


def write_graph(path: Path, nodes, edges):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<I", 1))
        f.write(struct.pack("<Q", len(nodes)))
        f.write(struct.pack("<Q", len(edges)))
        for nid, lat, lon, kind in nodes:
            f.write(struct.pack("<qddi", nid, lat, lon, kind))
        for eid, fr, to, etype, length, speed in edges:
            f.write(struct.pack("<qqqidd", eid, fr, to, etype, length, speed))


def extract_bbox(nodes, edges, bbox, margin_deg: float):
    min_lon, min_lat, max_lon, max_lat = bbox
    min_lon -= margin_deg
    min_lat -= margin_deg
    max_lon += margin_deg
    max_lat += margin_deg

    kept_nodes = [
        n for n in nodes if min_lon <= n[2] <= max_lon and min_lat <= n[1] <= max_lat
    ]
    node_ids = {n[0] for n in kept_nodes}
    kept_edges = [
        e for e in edges if e[1] in node_ids and e[2] in node_ids
    ]
    return kept_nodes, kept_edges


def main() -> int:
    parser = argparse.ArgumentParser(description="Build regional mmlp graph from national bin")
    parser.add_argument("input", help="national china.mmlp.bin")
    parser.add_argument("suffix", help="region suffix, e.g. sc")
    parser.add_argument("--bbox", required=True, help="minLon,minLat,maxLon,maxLat")
    parser.add_argument("--margin-deg", type=float, default=0.15)
    parser.add_argument("--skip-index", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    src = Path(args.input)
    if not src.is_file():
        raise SystemExit(f"input not found: {src}")

    bbox = parse_bbox(args.bbox)
    base = src.name[:-4] if src.name.endswith(".bin") else src.stem
    if base.endswith(".mmlp"):
        base = base[: -len(".mmlp")]
    out = src.with_name(f"{base}_{args.suffix}.mmlp.bin")

    print(f"[region] input={src}")
    print(f"[region] bbox={bbox} margin_deg={args.margin_deg}")
    nodes, edges = read_graph(src)
    sub_nodes, sub_edges = extract_bbox(nodes, edges, bbox, args.margin_deg)
    print(f"[region] nodes={len(sub_nodes):,} edges={len(sub_edges):,}")
    write_graph(out, sub_nodes, sub_edges)
    print(f"[region] wrote {out}")

    if args.skip_index:
        return 0

    build_aux = root / "build" / "mmlp_build_aux"
    if not build_aux.is_file():
        subprocess.check_call(["cmake", "-S", str(root), "-B", str(root / "build")])
        subprocess.check_call(
            ["cmake", "--build", str(root / "build"), "--target", "mmlp_build_aux"]
        )
    subprocess.check_call([str(build_aux), str(out)])
    print("[region] index build complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
