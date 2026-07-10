#!/usr/bin/env python3
"""Extract a regional subset from china.mmlp.bin and build index sidecars.

Streams the national graph from disk so CentOS/~30GB hosts are not OOM-killed
(the old implementation loaded all ~80M nodes + ~85M edges into Python lists).
"""

from __future__ import print_function

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


def extract_bbox_streaming(path, bbox, margin_deg):
    """Two-pass stream: keep in-bbox nodes, then edges whose endpoints are kept."""
    min_lon, min_lat, max_lon, max_lat = bbox
    min_lon -= margin_deg
    min_lat -= margin_deg
    max_lon += margin_deg
    max_lat += margin_deg

    with path.open("rb") as f:
        if f.read(8) != MAGIC:
            raise SystemExit("invalid graph magic: {}".format(path))
        version = struct.unpack("<I", f.read(4))[0]
        if version != 1:
            raise SystemExit("unsupported version: {}".format(version))
        node_count, edge_count = struct.unpack("<QQ", f.read(16))

        kept_nodes = []
        node_ids = set()
        # Progress every ~8M nodes (~10%).
        step = max(node_count // 10, 1)
        for i in range(node_count):
            nid, lat, lon, kind = struct.unpack("<qddi", f.read(NODE_RECORD))
            if min_lon <= lon <= max_lon and min_lat <= lat <= max_lat:
                kept_nodes.append((nid, lat, lon, kind))
                node_ids.add(nid)
            if (i + 1) % step == 0 or i + 1 == node_count:
                print(
                    "[region] nodes scanned {}/{} kept={:,}".format(
                        i + 1, node_count, len(kept_nodes)
                    ),
                    flush=True,
                )

        kept_edges = []
        step_e = max(edge_count // 10, 1)
        for i in range(edge_count):
            eid, fr, to, etype, length, speed = struct.unpack(
                "<qqqidd", f.read(EDGE_RECORD)
            )
            if fr in node_ids and to in node_ids:
                kept_edges.append((eid, fr, to, etype, length, speed))
            if (i + 1) % step_e == 0 or i + 1 == edge_count:
                print(
                    "[region] edges scanned {}/{} kept={:,}".format(
                        i + 1, edge_count, len(kept_edges)
                    ),
                    flush=True,
                )

    return kept_nodes, kept_edges


def write_graph(path, nodes, edges):
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


def main():
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
        raise SystemExit("input not found: {}".format(src))

    bbox = parse_bbox(args.bbox)
    base = src.name[:-4] if src.name.endswith(".bin") else src.stem
    if base.endswith(".mmlp"):
        base = base[: -len(".mmlp")]
    out = src.with_name("{}_{}.mmlp.bin".format(base, args.suffix))

    print("[region] input={}".format(src), flush=True)
    print("[region] bbox={} margin_deg={}".format(bbox, args.margin_deg), flush=True)
    print("[region] streaming extract (low memory)...", flush=True)
    sub_nodes, sub_edges = extract_bbox_streaming(src, bbox, args.margin_deg)
    print(
        "[region] nodes={:,} edges={:,}".format(len(sub_nodes), len(sub_edges)),
        flush=True,
    )
    write_graph(out, sub_nodes, sub_edges)
    print("[region] wrote {}".format(out), flush=True)

    if args.skip_index:
        return 0

    build_aux = root / "build" / "mmlp_build_aux"
    if not build_aux.is_file():
        subprocess.check_call(["cmake", "-S", str(root), "-B", str(root / "build")])
        subprocess.check_call(
            ["cmake", "--build", str(root / "build"), "--target", "mmlp_build_aux"]
        )
    subprocess.check_call([str(build_aux), str(out)])
    print("[region] index build complete", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
