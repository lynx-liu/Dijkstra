#!/usr/bin/env python3
"""Export mmlp.bin road/rail segments in a bbox to GeoJSON (for map preview)."""


import argparse
import json
import struct
import sys
import time

MAGIC = b"MMLPGRPH"
NODE_RECORD = 28  # q + d + d + i
EDGE_RECORD = 44  # q*3 + i + d*2


def parse_bbox(text: str):
    parts = [float(x.strip()) for x in text.split(",")]
    if len(parts) != 4:
        raise ValueError("bbox: minLon,minLat,maxLon,maxLat")
    return parts[0], parts[1], parts[2], parts[3]


def in_bbox(lat, lon, bbox):
    min_lon, min_lat, max_lon, max_lat = bbox
    return min_lon <= lon <= max_lon and min_lat <= lat <= max_lat


def read_header(f):
    magic = f.read(8)
    if magic != MAGIC:
        raise ValueError("not an mmlp graph file")
    version, node_count, edge_count = struct.unpack("<IQQ", f.read(20))
    if version != 1:
        raise ValueError(f"unsupported version {version}")
    return node_count, edge_count


def export_geojson(input_path: str, output_path: str, bbox, max_features: int, modes: str):
    t0 = time.time()
    with open(input_path, "rb") as f:
        node_count, edge_count = read_header(f)
        print(f"[export] nodes={node_count:,} edges={edge_count:,}", flush=True)

        nodes = {}
        for i in range(node_count):
            buf = f.read(NODE_RECORD)
            nid, lat, lon, kind = struct.unpack("<qddi", buf)
            if in_bbox(lat, lon, bbox):
                nodes[nid] = (lat, lon, kind)
            if (i + 1) % 5_000_000 == 0:
                print(f"[export] scanned nodes {i+1:,}", flush=True)

        print(f"[export] nodes in bbox: {len(nodes):,}", flush=True)

        features = []
        road_n = rail_n = 0
        for i in range(edge_count):
            buf = f.read(EDGE_RECORD)
            eid, fr, to, etype, length, speed = struct.unpack("<qqqidd", buf)
            if modes == "road" and etype != 0:
                continue
            if modes == "rail" and etype != 1:
                continue
            a = nodes.get(fr)
            b = nodes.get(to)
            if a is None or b is None:
                continue
            lat1, lon1, k1 = a
            lat2, lon2, k2 = b
            kind = int(etype)
            if kind == 0:
                road_n += 1
            else:
                rail_n += 1
            features.append(
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": [[lon1, lat1], [lon2, lat2]],
                    },
                    "properties": {
                        "id": int(eid),
                        "type": "road" if etype == 0 else "rail",
                        "length_m": round(length, 1),
                        "speed_kmh": speed,
                        "from_kind": k1,
                        "to_kind": k2,
                    },
                }
            )
            if len(features) >= max_features:
                print(f"[export] hit max_features={max_features}", flush=True)
                break
            if (i + 1) % 10_000_000 == 0:
                print(f"[export] scanned edges {i+1:,} features={len(features):,}", flush=True)

    fc = {"type": "FeatureCollection", "features": features}
    with open(output_path, "w", encoding="utf-8") as out:
        json.dump(fc, out, separators=(",", ":"))

    mb = len(json.dumps(fc)) / (1024 * 1024)
    print(
        f"[export] wrote {output_path} features={len(features):,} "
        f"(road={road_n:,} rail={rail_n:,}) ~{mb:.1f} MiB json in {time.time()-t0:.1f}s",
        flush=True,
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("-i", "--input", default="data/graph/china.mmlp.bin")
    p.add_argument("-o", "--output", required=True)
    p.add_argument("--bbox", required=True, help="minLon,minLat,maxLon,maxLat")
    p.add_argument("--max-features", type=int, default=80000)
    p.add_argument("--modes", choices=("all", "road", "rail"), default="all")
    args = p.parse_args()
    export_geojson(args.input, args.output, parse_bbox(args.bbox), args.max_features, args.modes)


if __name__ == "__main__":
    main()
