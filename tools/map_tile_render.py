#!/usr/bin/env python3
"""Render PNG map tiles from .mmlp.bin graph (offline street-style, no external CDN)."""


import io
import math
import threading
from collections import OrderedDict
from typing import Any, Dict, List, Optional, Tuple

from tools.map_roads import GraphMmap, collect_edge_ids_in_bbox, segment_in_bbox

try:
    from PIL import Image, ImageDraw
except ImportError as e:
    raise ImportError("Pillow required: pip3 install Pillow") from e

TILE_SIZE = 256
MIN_ZOOM = 8
MAX_ZOOM = 16


def tile_bounds(z: int, x: int, y: int) -> Tuple[float, float, float, float]:
    n = 2.0**z
    lon_min = x / n * 360.0 - 180.0
    lon_max = (x + 1) / n * 360.0 - 180.0

    def _y_to_lat(tile_y: float) -> float:
        lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * tile_y / n)))
        return math.degrees(lat_rad)

    lat_max = _y_to_lat(float(y))
    lat_min = _y_to_lat(float(y + 1))
    return lon_min, lat_min, lon_max, lat_max


def _max_edges_for_zoom(z: int) -> int:
    if z <= 10:
        return 6000
    if z <= 13:
        return 12000
    return 20000


def _line_width(z: int, is_rail: bool) -> int:
    if is_rail:
        return max(1, z - 9)
    if z <= 11:
        return 1
    if z <= 13:
        return 2
    return 3


class GraphTileRenderer:
    """On-demand street-style tiles from graph indexes."""

    def __init__(self, graph_path: str, cache_size: int = 512):
        self._graph_path = graph_path
        self._graph: Optional[GraphMmap] = None
        self._lock = threading.Lock()
        self._cache: OrderedDict[Tuple[int, int, int], bytes] = OrderedDict()
        self._cache_max = cache_size

    def _ensure(self) -> GraphMmap:
        if self._graph is None:
            self._graph = GraphMmap(self._graph_path)
        return self._graph

    def meta(self) -> Dict[str, Any]:
        return {
            "available": True,
            "format": "png",
            "source": "graph_render",
            "name": "本地生成的街道图",
            "minzoom": MIN_ZOOM,
            "maxzoom": MAX_ZOOM,
            "tilesize": TILE_SIZE,
            "note": "由路网离线渲染，非卫星影像",
        }

    def close(self) -> None:
        if self._graph is not None:
            self._graph.close()
            self._graph = None

    def _cache_get(self, key: Tuple[int, int, int]) -> Optional[bytes]:
        if key not in self._cache:
            return None
        self._cache.move_to_end(key)
        return self._cache[key]

    def _cache_put(self, key: Tuple[int, int, int], data: bytes) -> None:
        self._cache[key] = data
        self._cache.move_to_end(key)
        while len(self._cache) > self._cache_max:
            self._cache.popitem(last=False)

    def render_tile(self, z: int, x: int, y: int) -> Optional[bytes]:
        if z < MIN_ZOOM or z > MAX_ZOOM:
            return None
        key = (z, x, y)
        cached = self._cache_get(key)
        if cached is not None:
            return cached

        with self._lock:
            cached = self._cache_get(key)
            if cached is not None:
                return cached
            png = self._render_tile_unlocked(z, x, y)
            if png is not None:
                self._cache_put(key, png)
            return png

    def _render_tile_unlocked(self, z: int, x: int, y: int) -> bytes:
        lon_min, lat_min, lon_max, lat_max = tile_bounds(z, x, y)
        graph = self._ensure()
        sidx_path = graph.base + ".sidx"
        edge_ids = collect_edge_ids_in_bbox(sidx_path, lon_min, lat_min, lon_max, lat_max)

        # OSM-like land background
        img = Image.new("RGB", (TILE_SIZE, TILE_SIZE), (237, 232, 224))
        draw = ImageDraw.Draw(img)

        lon_span = max(lon_max - lon_min, 1e-12)
        lat_span = max(lat_max - lat_min, 1e-12)

        def to_px(lat: float, lon: float) -> Tuple[float, float]:
            px = (lon - lon_min) / lon_span * TILE_SIZE
            py = (lat_max - lat) / lat_span * TILE_SIZE
            return px, py

        max_edges = _max_edges_for_zoom(z)
        rails: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
        roads: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []

        for eid in edge_ids:
            if len(roads) + len(rails) >= max_edges:
                break
            ep = graph.edge_endpoints(eid)
            if ep is None:
                continue
            _fr, _to, etype = ep
            a = graph.node_latlon(ep[0])
            b = graph.node_latlon(ep[1])
            if a is None or b is None:
                continue
            lat1, lon1 = a
            lat2, lon2 = b
            if not segment_in_bbox(lat1, lon1, lat2, lon2, lon_min, lat_min, lon_max, lat_max):
                continue
            p1 = to_px(lat1, lon1)
            p2 = to_px(lat2, lon2)
            if etype == 0:
                roads.append((p1, p2))
            else:
                rails.append((p1, p2))

        # Rail under roads
        for p1, p2 in rails:
            w = _line_width(z, True)
            draw.line([p1, p2], fill=(180, 60, 40), width=w + 1)
        for p1, p2 in rails:
            w = _line_width(z, True)
            draw.line([p1, p2], fill=(220, 90, 60), width=max(1, w))

        for p1, p2 in roads:
            w = _line_width(z, False)
            draw.line([p1, p2], fill=(180, 170, 155), width=w + 2)
        for p1, p2 in roads:
            w = _line_width(z, False)
            draw.line([p1, p2], fill=(255, 255, 255), width=w)

        buf = io.BytesIO()
        img.save(buf, format="PNG", optimize=True)
        return buf.getvalue()
