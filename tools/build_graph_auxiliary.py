#!/usr/bin/env python3
"""
Build auxiliary files for fast service startup (low RAM, streaming):
  china.mmlp.bin -> china.mmlp.sidx, .nidx, .eidx
from typing import Dict, List, Tuple


One-time after deploy:
  python3 tools/build_graph_auxiliary.py data/graph/china.mmlp.bin
"""


import os
import struct
import sys
import tempfile
import time

BIN_MAGIC = b"MMLPGRPH"
SIDX_MAGIC = b"MMLPSIDX"
NDX_MAGIC = b"MMLPNDX\x00"  # 8 bytes (must match C++ header)
NODE_RECORD = 28
EDGE_RECORD = 44
CELL_SIZE = 0.02


def cell_of(lat: float, lon: float) -> Tuple[int, int]:
    return (int(lat // CELL_SIZE), int(lon // CELL_SIZE))


def write_sorted_index(path: str, rows: List[Tuple[int, int]]) -> None:
    rows.sort(key=lambda x: x[0])
    with open(path, "wb") as out:
        out.write(NDX_MAGIC)
        out.write(struct.pack("<IQ", 1, len(rows)))
        for row in rows:
            out.write(struct.pack("<qQ", *row))


def lookup_node_latlon(sorted_nodes: List[Tuple[int, int, float, float]], nid: int):
    lo, hi = 0, len(sorted_nodes) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if sorted_nodes[mid][0] == nid:
            return sorted_nodes[mid][2], sorted_nodes[mid][3]
        if sorted_nodes[mid][0] < nid:
            lo = mid + 1
        else:
            hi = mid - 1
    return None


def build_auxiliary(bin_path: str) -> None:
    t0 = time.time()
    base, _ = os.path.splitext(bin_path)

    with open(bin_path, "rb") as f:
        if f.read(8) != BIN_MAGIC:
            raise SystemExit("invalid bin magic")
        version, node_count, edge_count = struct.unpack("<IQQ", f.read(20))
        if version != 1:
            raise SystemExit(f"unsupported version {version}")

        print(f"[aux] nodes={node_count:,} edges={edge_count:,}", flush=True)

        node_rows: List[Tuple[int, int, float, float]] = []
        for i in range(node_count):
            off = f.tell()
            rec = f.read(NODE_RECORD)
            nid, lat, lon, _kind = struct.unpack("<qddi", rec)
            node_rows.append((nid, off, lat, lon))
            if i and i % 10_000_000 == 0:
                print(f"[aux] nodes {100 * i / node_count:.0f}%", flush=True)

        print("[aux] sorting nodes for nidx ...", flush=True)
        node_rows.sort(key=lambda x: x[0])
        nidx_rows = [(nid, off) for nid, off, _lat, _lon in node_rows]
        write_sorted_index(base + ".nidx", nidx_rows)
        del nidx_rows

        edge_base = f.tell()
        print("[aux] scanning edges (eidx + sidx) ...", flush=True)
        cells: Dict[Tuple[int, int], List[int]] = {}
        eidx_rows: List[Tuple[int, int]] = []

        for i in range(edge_count):
            off = f.tell()
            rec = f.read(EDGE_RECORD)
            eid, fr, to, _etype, _length, _speed = struct.unpack("<qqqidd", rec)
            eidx_rows.append((eid, off))

            a = lookup_node_latlon(node_rows, fr)
            b = lookup_node_latlon(node_rows, to)
            if a and b:
                for lat, lon in (a, b, ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)):
                    gx, gy = cell_of(lat, lon)
                    cells.setdefault((gx, gy), []).append(eid)
            if i and i % 5_000_000 == 0:
                print(f"[aux] edges {100 * i / edge_count:.0f}%", flush=True)

        del node_rows
        print("[aux] writing eidx ...", flush=True)
        write_sorted_index(base + ".eidx", eidx_rows)
        del eidx_rows

    sidx_path = base + ".sidx"
    print(f"[aux] writing {sidx_path} ({len(cells):,} cells) ...", flush=True)
    with open(sidx_path, "wb") as out:
        out.write(SIDX_MAGIC)
        out.write(struct.pack("<IdI", 1, CELL_SIZE, len(cells)))
        for (gx, gy), eids in cells.items():
            out.write(struct.pack("<iiQ", gx, gy, len(eids)))
            for eid in eids:
                out.write(struct.pack("<q", eid))
    del cells

    print(f"[aux] done in {time.time() - t0:.1f}s", flush=True)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "data/graph/china.mmlp.bin"
    build_auxiliary(path)


if __name__ == "__main__":
    main()
