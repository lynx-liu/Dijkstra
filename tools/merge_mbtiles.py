#!/usr/bin/env python3
"""Merge Shortbread MBTiles overlays into a base file (China + Central Asia)."""

# Compatible with CentOS 7 system Python 3.6 (no future annotations).

import argparse
import os
import shutil
import sqlite3
import sys
from typing import List, Optional, Tuple


def parse_bounds(value: Optional[str]) -> Optional[Tuple[float, float, float, float]]:
    if not value:
        return None
    parts = [p.strip() for p in value.split(",")]
    if len(parts) != 4:
        return None
    try:
        return float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
    except ValueError:
        return None


def meta_get(conn: sqlite3.Connection, name: str) -> Optional[str]:
    row = conn.execute("SELECT value FROM metadata WHERE name=?", (name,)).fetchone()
    return None if row is None else str(row[0])


def meta_set(conn: sqlite3.Connection, name: str, value: str) -> None:
    conn.execute(
        "INSERT INTO metadata(name, value) VALUES(?, ?) "
        "ON CONFLICT(name) DO UPDATE SET value=excluded.value",
        (name, value),
    )


def union_bounds(a: Optional[str], b: Optional[str]) -> Optional[str]:
    ba, bb = parse_bounds(a), parse_bounds(b)
    if ba is None:
        return b
    if bb is None:
        return a
    west = min(ba[0], bb[0])
    south = min(ba[1], bb[1])
    east = max(ba[2], bb[2])
    north = max(ba[3], bb[3])
    return f"{west:.6f},{south:.6f},{east:.6f},{north:.6f}"


def merge_one(dst: sqlite3.Connection, src_path: str) -> int:
    print(f"[merge_mbtiles] + {src_path}", flush=True)
    src = sqlite3.connect(f"file:{src_path}?mode=ro", uri=True)
    try:
        before = dst.execute("SELECT COUNT(*) FROM tiles").fetchone()[0]
        dst.execute("ATTACH DATABASE ? AS src", (src_path,))
        # Prefer overlay content on conflict (CA detail over China edge stubs).
        dst.execute(
            "INSERT OR REPLACE INTO tiles(zoom_level, tile_column, tile_row, tile_data) "
            "SELECT zoom_level, tile_column, tile_row, tile_data FROM src.tiles"
        )
        dst.commit()
        after = dst.execute("SELECT COUNT(*) FROM tiles").fetchone()[0]
        bounds = union_bounds(meta_get(dst, "bounds"), meta_get(src, "bounds"))
        if bounds:
            meta_set(dst, "bounds", bounds)
        for key in ("minzoom", "maxzoom"):
            try:
                cur = int(meta_get(dst, key) or "0")
                oth = int(meta_get(src, key) or str(cur))
            except ValueError:
                continue
            meta_set(dst, key, str(min(cur, oth) if key == "minzoom" else max(cur, oth)))
        dst.execute("DETACH DATABASE src")
        dst.commit()
        added = after - before
        print(f"[merge_mbtiles]   tiles now={after:,} (+{added:,})", flush=True)
        return added
    finally:
        src.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True, help="Base mbtiles (e.g. china.mbtiles)")
    parser.add_argument("--overlay", action="append", default=[], help="Overlay mbtiles (repeat)")
    parser.add_argument("--output", required=True, help="Output mbtiles path")
    args = parser.parse_args()

    if not os.path.isfile(args.base):
        print(f"ERROR: base not found: {args.base}", file=sys.stderr)
        return 1
    overlays: List[str] = []
    for path in args.overlay:
        if not os.path.isfile(path):
            print(f"ERROR: overlay not found: {path}", file=sys.stderr)
            return 1
        overlays.append(path)
    if not overlays:
        print("ERROR: need at least one --overlay", file=sys.stderr)
        return 1

    out_dir = os.path.dirname(args.output) or "."
    os.makedirs(out_dir, exist_ok=True)
    tmp = args.output + ".tmp"
    if os.path.exists(tmp):
        os.remove(tmp)

    print(f"[merge_mbtiles] copy base -> {tmp}", flush=True)
    shutil.copy2(args.base, tmp)

    conn = sqlite3.connect(tmp)
    try:
        conn.execute("PRAGMA journal_mode=OFF")
        conn.execute("PRAGMA synchronous=OFF")
        # Ensure metadata has a PRIMARY KEY / UNIQUE on name for ON CONFLICT.
        cols = {
            r[1] for r in conn.execute("PRAGMA table_info(metadata)").fetchall()
        }
        if "name" in cols:
            try:
                conn.execute(
                    "CREATE UNIQUE INDEX IF NOT EXISTS metadata_name_idx ON metadata(name)"
                )
            except sqlite3.Error:
                pass
        for path in overlays:
            merge_one(conn, path)
        meta_set(conn, "name", "China + Central Asia Shortbread")
        bounds = meta_get(conn, "bounds")
        if bounds:
            parts = parse_bounds(bounds)
            if parts:
                lon = (parts[0] + parts[2]) / 2
                lat = (parts[1] + parts[3]) / 2
                meta_set(conn, "center", f"{lon:.6f},{lat:.6f},5")
        conn.commit()
    finally:
        conn.close()

    os.replace(tmp, args.output)
    size_gb = os.path.getsize(args.output) / (1024**3)
    print(f"[merge_mbtiles] wrote {args.output} ({size_gb:.2f} GiB)", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
