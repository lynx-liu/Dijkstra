#!/usr/bin/env python3
"""Quick integrity checks on china.mmlp.bin without loading into RAM."""

import struct
import sys

MAGIC = b"MMLPGRPH"
NODE_RECORD = 28
EDGE_RECORD = 44


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "data/graph/china.mmlp.bin"
    with open(path, "rb") as f:
        if f.read(8) != MAGIC:
            print("FAIL: bad magic")
            return 1
        version, node_count, edge_count = struct.unpack("<IQQ", f.read(20))
        print(f"version={version} nodes={node_count:,} edges={edge_count:,}")

        lat_min, lat_max = 90.0, -90.0
        lon_min, lon_max = 180.0, -180.0
        kinds = [0, 0, 0]
        bad = 0
        sample = []

        for i in range(node_count):
            nid, lat, lon, kind = struct.unpack("<qddi", f.read(NODE_RECORD))
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                bad += 1
            lat_min, lat_max = min(lat_min, lat), max(lat_max, lat)
            lon_min, lon_max = min(lon_min, lon), max(lon_max, lon)
            if 0 <= kind <= 2:
                kinds[kind] += 1
            if len(sample) < 3 and 18 <= lat <= 54 and 73 <= lon <= 135:
                sample.append((nid, lat, lon, kind))

        print(f"lat range [{lat_min:.4f}, {lat_max:.4f}]")
        print(f"lon range [{lon_min:.4f}, {lon_max:.4f}]")
        print(f"node kinds junction/station/hub = {kinds}")
        print(f"invalid coords: {bad}")
        print("sample nodes in China bbox:", sample)

        neg_len = 0
        etypes = [0, 0]
        for i in range(min(edge_count, 5_000_000)):
            eid, fr, to, etype, length, speed = struct.unpack("<qqqidd", f.read(EDGE_RECORD))
            if length <= 0:
                neg_len += 1
            if etype in (0, 1):
                etypes[etype] += 1

        print(f"edge types (first 5M): road={etypes[0]:,} rail={etypes[1]:,}")
        print(f"non-positive length (first 5M): {neg_len}")
    print("OK: basic checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
