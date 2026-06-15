#!/usr/bin/env python3
"""
Export a bbox subset of .mmlp.bin to GeoJSON for map preview (numpy-accelerated scan).
"""


import argparse
import json
import struct
import sys
import time

import numpy as np

MAGIC = b"MMLPGRPH"
HEADER_SIZE = 8 + 4 + 8 + 8
NODE_DTYPE = np.dtype([("id", "<i8"), ("lat", "<f8"), ("lon", "<f8"), ("kind", "<i4")])
EDGE_DTYPE = np.dtype(
    [
        ("id", "<i8"),
        ("node_from", "<i8"),
        ("node_to", "<i8"),
        ("etype", "<i4"),
        ("length", "<f8"),
        ("speed", "<f8"),
    ]
)


def parse_bbox(text: str):
    parts = [float(x.strip()) for x in text.split(",")]
    if len(parts) != 4:
        raise ValueError("bbox: minLon,minLat,maxLon,maxLat")
    return parts[0], parts[1], parts[2], parts[3]


def load_graph_arrays(path: str):
    with open(path, "rb") as f:
        if f.read(8) != MAGIC:
            raise SystemExit("not an mmlp graph file")
        version = struct.unpack("<I", f.read(4))[0]
        if version != 1:
            raise SystemExit(f"unsupported version {version}")
        node_count, edge_count = struct.unpack("<QQ", f.read(16))
        print(f"[geojson] graph nodes={node_count:,} edges={edge_count:,}", flush=True)

        node_bytes = node_count * NODE_DTYPE.itemsize
        edge_bytes = edge_count * EDGE_DTYPE.itemsize

        print("[geojson] reading nodes array ...", flush=True)
        nodes = np.fromfile(f, dtype=NODE_DTYPE, count=node_count)
        print("[geojson] reading edges array ...", flush=True)
        edges = np.fromfile(f, dtype=EDGE_DTYPE, count=edge_count)
    return nodes, edges


def export_geojson(nodes, edges, bbox, output: str, max_edges: int):
    min_lon, min_lat, max_lon, max_lat = bbox
    t0 = time.time()

    mask = (
        (nodes["lon"] >= min_lon)
        & (nodes["lon"] <= max_lon)
        & (nodes["lat"] >= min_lat)
        & (nodes["lat"] <= max_lat)
    )
    sub = nodes[mask]
    id_arr = sub["id"]
    print(f"[geojson] nodes in bbox: {len(sub):,}", flush=True)

    coord = {int(n["id"]): (float(n["lat"]), float(n["lon"])) for n in sub}

    print("[geojson] filtering edges ...", flush=True)
    em = np.isin(edges["node_from"], id_arr) & np.isin(edges["node_to"], id_arr)
    sub_e = edges[em]
    if len(sub_e) > max_edges:
        sub_e = sub_e[:max_edges]
    print(f"[geojson] edges in bbox (capped): {len(sub_e):,}", flush=True)

    features = []
    road = rail = 0
    for e in sub_e:
        lat1, lon1 = coord[int(e["node_from"])]
        lat2, lon2 = coord[int(e["node_to"])]
        etype = int(e["etype"])
        if etype == 0:
            road += 1
            color = "#3388ff"
        else:
            rail += 1
            color = "#aa2244"
        features.append(
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[lon1, lat1], [lon2, lat2]],
                },
                "properties": {
                    "id": int(e["id"]),
                    "type": "road" if etype == 0 else "rail",
                    "length_m": round(float(e["length"]), 1),
                    "stroke": color,
                },
            }
        )
    exported = len(sub_e)

    hub = 0
    for n in sub:
        if int(n["kind"]) == 2:
            hub += 1
            features.append(
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [float(n["lon"]), float(n["lat"])],
                    },
                    "properties": {"id": int(n["id"]), "type": "hub"},
                }
            )

    collection = {
        "type": "FeatureCollection",
        "properties": {
            "bbox": ",".join(map(str, bbox)),
            "nodes_in_bbox": int(len(sub)),
            "edges_exported": exported,
            "road_segments": road,
            "rail_segments": rail,
            "hubs": hub,
            "truncated": exported >= max_edges,
        },
        "features": features,
    }

    with open(output, "w", encoding="utf-8") as out:
        json.dump(collection, out, ensure_ascii=False)

    print(
        f"[geojson] wrote {output} road={road} rail={rail} hub={hub} "
        f"in {time.time()-t0:.1f}s",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", default="data/graph/china.mmlp.bin")
    parser.add_argument("-o", "--output", required=True)
    parser.add_argument("--bbox", required=True)
    parser.add_argument("--max-edges", type=int, default=12000)
    args = parser.parse_args()

    bbox = parse_bbox(args.bbox)
    t0 = time.time()
    nodes, edges = load_graph_arrays(args.input)
    export_geojson(nodes, edges, bbox, args.output, args.max_edges)
    print(f"[geojson] total {time.time()-t0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
