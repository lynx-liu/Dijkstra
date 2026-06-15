#!/usr/bin/env python3
"""Serve raster tiles from a local MBTiles file (offline street map)."""


import os
import sqlite3
import threading
from typing import Any, Dict, Optional


class MBTilesStore:
    def __init__(self, path: str):
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        self.path = path
        self._lock = threading.Lock()
        self._conn = sqlite3.connect(path, check_same_thread=False)
        self.meta = self._read_meta()

    def _read_meta(self) -> Dict[str, Any]:
        meta: Dict[str, Any] = {"available": True, "path": self.path}
        try:
            rows = self._conn.execute("SELECT name, value FROM metadata").fetchall()
            for name, value in rows:
                if name in ("minzoom", "maxzoom", "bounds", "center", "name", "format"):
                    meta[name] = value
        except sqlite3.Error:
            pass
        return meta

    def get_tile(self, z: int, x: int, y: int) -> Optional[bytes]:
        # Leaflet XYZ -> TMS row
        y_tms = (1 << z) - 1 - y
        with self._lock:
            row = self._conn.execute(
                "SELECT tile_data FROM tiles "
                "WHERE zoom_level=? AND tile_column=? AND tile_row=? LIMIT 1",
                (z, x, y_tms),
            ).fetchone()
        if row is None:
            return None
        return row[0]

    def close(self) -> None:
        with self._lock:
            self._conn.close()


def resolve_mbtiles_path() -> Optional[str]:
    for key in ("MMLP_MBTILES",):
        val = os.environ.get(key, "").strip()
        if val and os.path.isfile(val):
            return val
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates = [
        os.path.join(root, "data/map/china.mbtiles"),
        os.path.join(root, "data/map/region.mbtiles"),
        os.path.join(root, "data/map/offline.mbtiles"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return None
