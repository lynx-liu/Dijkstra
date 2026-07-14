#!/usr/bin/env python3
"""Serve vector/raster tiles from one or more local MBTiles files (offline street map)."""

# Compatible with CentOS 7 system Python 3.6 (no future annotations).

import os
import sqlite3
import threading
from typing import Any, Dict, List, Optional, Sequence


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
                if name in ("minzoom", "maxzoom", "bounds", "center", "name", "format", "type"):
                    meta[name] = value
        except sqlite3.Error:
            pass
        fmt = str(meta.get("format") or "png").lower()
        meta["format"] = fmt
        meta["vector"] = fmt in ("pbf", "mvt", "vector")
        for k in ("minzoom", "maxzoom"):
            if k in meta:
                try:
                    meta[k] = int(meta[k])
                except Exception:
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


def _parse_bounds(value: Any) -> Optional[List[float]]:
    if value is None:
        return None
    parts = [p.strip() for p in str(value).split(",")]
    if len(parts) != 4:
        return None
    try:
        return [float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])]
    except ValueError:
        return None


def _layer_slug(path: str) -> str:
    name = os.path.splitext(os.path.basename(path))[0].lower()
    # china.mbtiles -> china; kazakhstan.mbtiles -> kazakhstan
    out = []
    for ch in name:
        if ch.isalnum() or ch in "-_":
            out.append(ch)
        else:
            out.append("_")
    slug = "".join(out).strip("_") or "layer"
    return slug


class MultiMBTilesStore:
    """Several Shortbread mbtiles. MapLibre should load each as its own source
    (overlapping country extracts cannot be represented by a single tile blob)."""

    def __init__(self, paths: Sequence[str]):
        if not paths:
            raise ValueError("MultiMBTilesStore needs at least one path")
        self.stores = [MBTilesStore(p) for p in paths]
        self.path = ";".join(paths)
        self.layer_ids: List[str] = []
        seen = set()
        for store in self.stores:
            slug = _layer_slug(store.path)
            base = slug
            n = 2
            while slug in seen:
                slug = f"{base}{n}"
                n += 1
            seen.add(slug)
            self.layer_ids.append(slug)
        self._by_id = {lid: store for lid, store in zip(self.layer_ids, self.stores)}
        self.meta = self._merge_meta()

    def _merge_meta(self) -> Dict[str, Any]:
        base = dict(self.stores[0].meta)
        base["path"] = self.path
        base["layers"] = [s.path for s in self.stores]
        base["layer_ids"] = [
            {"id": lid, "index": i, "path": store.path}
            for i, (lid, store) in enumerate(zip(self.layer_ids, self.stores))
        ]
        # Client must composite sources; picking one blob drops the other country.
        base["composite"] = len(self.stores) > 1
        west = south = east = north = None
        min_z: Optional[int] = None
        max_z: Optional[int] = None
        for store in self.stores:
            b = _parse_bounds(store.meta.get("bounds"))
            if b is not None:
                west = b[0] if west is None else min(west, b[0])
                south = b[1] if south is None else min(south, b[1])
                east = b[2] if east is None else max(east, b[2])
                north = b[3] if north is None else max(north, b[3])
            mz = store.meta.get("minzoom")
            xz = store.meta.get("maxzoom")
            if isinstance(mz, int):
                min_z = mz if min_z is None else min(min_z, mz)
            if isinstance(xz, int):
                max_z = xz if max_z is None else max(max_z, xz)
        if None not in (west, south, east, north):
            base["bounds"] = f"{west:.6f},{south:.6f},{east:.6f},{north:.6f}"
            base["center"] = f"{(west + east) / 2:.6f},{(south + north) / 2:.6f},5"
        if min_z is not None:
            base["minzoom"] = min_z
        if max_z is not None:
            base["maxzoom"] = max_z
        if len(self.stores) > 1:
            base["name"] = "China + Central Asia + Russia Shortbread"
        return base

    def get_tile(self, z: int, x: int, y: int, layer_id: Optional[str] = None) -> Optional[bytes]:
        if layer_id is not None:
            store = self._by_id.get(layer_id)
            if store is None:
                return None
            return store.get_tile(z, x, y)
        # Legacy single-blob fallback (lossy across overlapping extracts).
        best: Optional[bytes] = None
        for store in self.stores:
            data = store.get_tile(z, x, y)
            if data is None:
                continue
            if best is None or len(data) > len(best):
                best = data
        return best

    def close(self) -> None:
        for store in self.stores:
            store.close()


def resolve_mbtiles_paths() -> List[str]:
    """Ordered mbtiles paths: optional merged file, else China + CA + Russia overlays."""
    env = os.environ.get("MMLP_MBTILES", "").strip()
    if env:
        parts = [p.strip() for p in env.split(":") if p.strip()]
        return [p for p in parts if os.path.isfile(p)]

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    merged = os.path.join(root, "data/map/china_central_asia.mbtiles")
    if os.path.isfile(merged):
        return [merged]

    paths: List[str] = []
    china = os.path.join(root, "data/map/china.mbtiles")
    if os.path.isfile(china):
        paths.append(china)

    ca_dir = os.path.join(root, "data/map/central_asia")
    for name in (
        "kazakhstan.mbtiles",
        "kyrgyzstan.mbtiles",
        "tajikistan.mbtiles",
        "turkmenistan.mbtiles",
        "uzbekistan.mbtiles",
    ):
        path = os.path.join(ca_dir, name)
        if os.path.isfile(path):
            paths.append(path)

    ru_dir = os.path.join(root, "data/map/russia")
    if os.path.isdir(ru_dir):
        for name in sorted(os.listdir(ru_dir)):
            if name.endswith(".mbtiles"):
                paths.append(os.path.join(ru_dir, name))

    if paths:
        return paths

    for rel in ("data/map/region.mbtiles", "data/map/offline.mbtiles"):
        path = os.path.join(root, rel)
        if os.path.isfile(path):
            return [path]
    return []


def resolve_mbtiles_path() -> Optional[str]:
    paths = resolve_mbtiles_paths()
    return paths[0] if paths else None


def open_mbtiles_store() -> Optional[MultiMBTilesStore]:
    paths = resolve_mbtiles_paths()
    if not paths:
        return None
    return MultiMBTilesStore(paths)
