#!/usr/bin/env python3
"""
Build MultimodalGraph binary (.mmlp.bin) from OpenStreetMap PBF/XML.
Memory-safe nationwide build via SQLite staging.

Requires: pip install osmium
"""


import argparse
import math
import os
import sqlite3
import struct
import sys
import time
from typing import Optional, Tuple

try:
    import osmium
except ImportError:
    print("ERROR: pyosmium not installed. Run: pip3 install osmium", file=sys.stderr)
    sys.exit(1)

MAGIC = b"MMLPGRPH"
VERSION = 1

HIGHWAY_TYPES = frozenset(
    {
        "motorway",
        "trunk",
        "primary",
        "secondary",
        "tertiary",
        "unclassified",
        "residential",
        "service",
        "motorway_link",
        "trunk_link",
        "primary_link",
        "secondary_link",
    }
)

EARTH_RADIUS_M = 6371000.0
HUB_RADIUS_M = 500.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    rlat1, rlon1, rlat2, rlon2 = map(math.radians, (lat1, lon1, lat2, lon2))
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1
    h = (
        math.sin(dlat / 2) ** 2
        + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def parse_maxspeed_kmh(value: Optional[str]) -> float:
    if not value:
        return 0.0
    v = value.strip().lower()
    try:
        if v.endswith("mph"):
            return float(v[:-3].strip()) * 1.60934
        if v.endswith("km/h") or v.endswith("kph"):
            return float(v.split()[0])
        if v.endswith("kmh"):
            return float(v[:-3])
        return float(v.split()[0])
    except ValueError:
        return 0.0


def parse_bbox(text: str) -> Tuple[float, float, float, float]:
    parts = [float(x.strip()) for x in text.split(",")]
    if len(parts) != 4:
        raise ValueError("bbox must be minLon,minLat,maxLon,maxLat")
    return parts[0], parts[1], parts[2], parts[3]


def in_bbox(lat: float, lon: float, bbox: Optional[Tuple[float, float, float, float]]) -> bool:
    if bbox is None:
        return True
    min_lon, min_lat, max_lon, max_lat = bbox
    return min_lon <= lon <= max_lon and min_lat <= lat <= max_lat


def open_db(db_path: str) -> sqlite3.Connection:
    if os.path.exists(db_path):
        os.remove(db_path)
    conn = sqlite3.connect(db_path)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA synchronous=NORMAL")
    conn.execute("PRAGMA temp_store=MEMORY")
    conn.executescript(
        """
        CREATE TABLE needed_node (id INTEGER PRIMARY KEY);
        CREATE TABLE way_row (
            way_id INTEGER PRIMARY KEY,
            edge_type INTEGER NOT NULL,
            speed REAL NOT NULL,
            refs TEXT NOT NULL
        );
        CREATE TABLE node_row (
            id INTEGER PRIMARY KEY,
            lat REAL NOT NULL,
            lon REAL NOT NULL,
            is_station INTEGER NOT NULL DEFAULT 0,
            kind INTEGER NOT NULL DEFAULT 0
        );
        CREATE TABLE edge_row (
            id INTEGER PRIMARY KEY,
            node_from INTEGER NOT NULL,
            node_to INTEGER NOT NULL,
            etype INTEGER NOT NULL,
            length REAL NOT NULL,
            speed REAL NOT NULL
        );
        CREATE INDEX idx_edge_from ON edge_row(node_from);
        """
    )
    return conn


class WayPass(osmium.SimpleHandler):
    def __init__(self, conn: sqlite3.Connection, bbox: Optional[Tuple[float, float, float, float]]):
        super().__init__()
        self.conn = conn
        self.bbox = bbox
        self.way_count = 0

    def way(self, w):
        tags = w.tags
        highway = tags.get("highway")
        railway = tags.get("railway")
        edge_type = None
        if highway in HIGHWAY_TYPES:
            if tags.get("area") == "yes":
                return
            edge_type = 0
        elif railway == "rail":
            edge_type = 1
        else:
            return

        refs = [int(n.ref) for n in w.nodes]
        if len(refs) < 2:
            return

        speed = parse_maxspeed_kmh(tags.get("maxspeed"))
        refs_txt = ",".join(str(r) for r in refs)
        self.conn.execute(
            "INSERT INTO way_row VALUES (?,?,?,?)",
            (w.id, edge_type, speed, refs_txt),
        )
        self.conn.executemany(
            "INSERT OR IGNORE INTO needed_node VALUES (?)",
            [(r,) for r in refs],
        )
        self.way_count += 1
        if self.way_count % 500000 == 0:
            self.conn.commit()
            print(f"[osm_to_graph] pass1 ways={self.way_count:,}", flush=True)


class NodePass(osmium.SimpleHandler):
    def __init__(self, conn: sqlite3.Connection, bbox: Optional[Tuple[float, float, float, float]]):
        super().__init__()
        self.conn = conn
        self.bbox = bbox
        self.node_count = 0

    def node(self, n):
        if not n.location.valid():
            return
        lat, lon = n.location.lat, n.location.lon
        if not in_bbox(lat, lon, self.bbox):
            return
        nid = int(n.id)
        row = self.conn.execute("SELECT 1 FROM needed_node WHERE id=?", (nid,)).fetchone()
        if row is None:
            return
        is_station = 1 if n.tags.get("railway") == "station" else 0
        kind = 1 if is_station else 0
        self.conn.execute(
            "INSERT OR REPLACE INTO node_row VALUES (?,?,?,?,?)",
            (nid, lat, lon, is_station, kind),
        )
        self.node_count += 1
        if self.node_count % 2000000 == 0:
            self.conn.commit()
            print(f"[osm_to_graph] pass2 nodes={self.node_count:,}", flush=True)


def materialize_edges(conn: sqlite3.Connection) -> Tuple[int, int]:
    cur = conn.execute("SELECT way_id, edge_type, speed, refs FROM way_row")
    edge_count = 0
    way_done = 0
    batch = []
    for way_id, edge_type, speed, refs_txt in cur:
        refs = [int(x) for x in refs_txt.split(",") if x]
        coords = []
        for nid in refs:
            row = conn.execute(
                "SELECT lat, lon FROM node_row WHERE id=?", (nid,)
            ).fetchone()
            if row is None:
                coords = []
                break
            coords.append((nid, row[0], row[1]))
        if len(coords) < 2:
            continue

        for i in range(len(coords) - 1):
            n1, lat1, lon1 = coords[i]
            n2, lat2, lon2 = coords[i + 1]
            seg_len = haversine_m(lat1, lon1, lat2, lon2)
            if seg_len < 1.0:
                continue
            seg_id = way_id * 10000 + i
            batch.append((seg_id, n1, n2, edge_type, seg_len, speed))
            edge_count += 1
            if len(batch) >= 100000:
                conn.executemany(
                    "INSERT INTO edge_row VALUES (?,?,?,?,?,?)", batch
                )
                batch.clear()

        way_done += 1
        if way_done % 500000 == 0:
            conn.commit()
            print(f"[osm_to_graph] pass3 ways={way_done:,} edges={edge_count:,}", flush=True)

    if batch:
        conn.executemany("INSERT INTO edge_row VALUES (?,?,?,?,?,?)", batch)
    conn.commit()
    node_count = conn.execute("SELECT COUNT(*) FROM node_row").fetchone()[0]
    return node_count, edge_count


def mark_hubs_sqlite(conn: sqlite3.Connection) -> None:
    print(f"[osm_to_graph] marking hubs (grid) ...", flush=True)
    cell_deg = HUB_RADIUS_M / 111000.0
    road_nodes = conn.execute(
        """
        SELECT DISTINCT n.id, n.lat, n.lon FROM node_row n
        JOIN edge_row e ON n.id = e.node_from OR n.id = e.node_to
        WHERE e.etype = 0
        """
    ).fetchall()
    grid = {}
    for nid, lat, lon in road_nodes:
        key = (int(lat / cell_deg), int(lon / cell_deg))
        grid.setdefault(key, []).append((lat, lon))

    stations = conn.execute(
        "SELECT id, lat, lon FROM node_row WHERE is_station=1"
    ).fetchall()
    hub_ids = []
    for nid, lat, lon in stations:
        cx, cy = int(lat / cell_deg), int(lon / cell_deg)
        found = False
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for rlat, rlon in grid.get((cx + dx, cy + dy), []):
                    if haversine_m(lat, lon, rlat, rlon) <= HUB_RADIUS_M:
                        hub_ids.append(nid)
                        found = True
                        break
                if found:
                    break
            if found:
                break
    if hub_ids:
        conn.executemany(
            "UPDATE node_row SET kind=2 WHERE id=?",
            [(i,) for i in hub_ids],
        )
        conn.commit()
    print(f"[osm_to_graph] hubs marked: {len(hub_ids)}", flush=True)


def export_binary(conn: sqlite3.Connection, path: str) -> None:
    node_count = conn.execute("SELECT COUNT(*) FROM node_row").fetchone()[0]
    edge_count = conn.execute("SELECT COUNT(*) FROM edge_row").fetchone()[0]
    print(
        f"[osm_to_graph] export nodes={node_count:,} edges={edge_count:,}",
        flush=True,
    )

    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "wb") as out:
        out.write(MAGIC)
        out.write(struct.pack("<I", VERSION))
        out.write(struct.pack("<Q", node_count))
        out.write(struct.pack("<Q", edge_count))

        cur = conn.execute("SELECT id, lat, lon, kind FROM node_row ORDER BY id")
        for nid, lat, lon, kind in cur:
            out.write(struct.pack("<q", nid))
            out.write(struct.pack("<d", lat))
            out.write(struct.pack("<d", lon))
            out.write(struct.pack("<i", kind))

        cur = conn.execute(
            "SELECT id, node_from, node_to, etype, length, speed FROM edge_row ORDER BY id"
        )
        for eid, fr, to, etype, length, speed in cur:
            out.write(struct.pack("<q", eid))
            out.write(struct.pack("<q", fr))
            out.write(struct.pack("<q", to))
            out.write(struct.pack("<i", etype))
            out.write(struct.pack("<d", length))
            out.write(struct.pack("<d", speed))

    size_mb = os.path.getsize(path) / (1024 * 1024)
    print(f"[osm_to_graph] wrote {path} ({size_mb:.2f} MiB)", flush=True)


def build_graph(pbf_path: str, bbox: Optional[Tuple[float, float, float, float]], db_path: str):
    conn = open_db(db_path)
    try:
        print(f"[osm_to_graph] pass 1/4: scan ways in {pbf_path}", flush=True)
        wpass = WayPass(conn, bbox)
        wpass.apply_file(pbf_path)
        conn.commit()
        print(f"[osm_to_graph] pass1 done ways={wpass.way_count:,}", flush=True)

        print("[osm_to_graph] pass 2/4: scan nodes", flush=True)
        npass = NodePass(conn, bbox)
        npass.apply_file(pbf_path, locations=True)
        conn.commit()
        print(f"[osm_to_graph] pass2 done nodes={npass.node_count:,}", flush=True)

        print("[osm_to_graph] pass 3/4: build edge segments", flush=True)
        materialize_edges(conn)

        print("[osm_to_graph] pass 4/4: hubs + export", flush=True)
        mark_hubs_sqlite(conn)
    finally:
        pass
    return conn


def main():
    parser = argparse.ArgumentParser(description="Convert OSM PBF to mmlp binary graph")
    parser.add_argument("--input", "-i", required=True)
    parser.add_argument("--output", "-o", required=True)
    parser.add_argument("--bbox", help="minLon,minLat,maxLon,maxLat")
    parser.add_argument(
        "--db",
        help="SQLite staging file (default: <output>.staging.sqlite)",
    )
    parser.add_argument(
        "--keep-db",
        action="store_true",
        help="Keep staging sqlite after success",
    )
    args = parser.parse_args()

    bbox = parse_bbox(args.bbox) if args.bbox else None
    db_path = args.db or (args.output + ".staging.sqlite")
    t0 = time.time()

    conn = build_graph(args.input, bbox, db_path)
    export_binary(conn, args.output)
    conn.close()

    if not args.keep_db and os.path.exists(db_path):
        os.remove(db_path)

    print(f"[osm_to_graph] done in {time.time() - t0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
