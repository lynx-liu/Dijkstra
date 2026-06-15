#!/usr/bin/env python3
"""
Export road/rail GeoJSON for map basemap from .mmlp.bin + auxiliary indexes (no full graph RAM).
"""


import math
import mmap
import struct
from typing import Any, Dict, List, Optional, Set, Tuple

BIN_MAGIC = b"MMLPGRPH"
NDX_MAGIC = b"MMLPNDX\x00"
SIDX_MAGIC = b"MMLPSIDX"
NODE_RECORD = 28
EDGE_RECORD = 44
INDEX_HEADER = 20  # magic(8) + version(4) + count(8)


def _cell_of(lat: float, lon: float, cell_size: float) -> Tuple[int, int]:
    return int(math.floor(lat / cell_size)), int(math.floor(lon / cell_size))


class GraphMmap:
    def __init__(self, bin_path: str):
        self.bin_path = bin_path
        dot = bin_path.rfind(".")
        self.base = bin_path if dot == -1 else bin_path[:dot]
        self._bin = open(bin_path, "rb")
        self._nidx = open(self.base + ".nidx", "rb")
        self._eidx = open(self.base + ".eidx", "rb")
        self.bin_mm = mmap.mmap(self._bin.fileno(), 0, access=mmap.ACCESS_READ)
        self.nidx_mm = mmap.mmap(self._nidx.fileno(), 0, access=mmap.ACCESS_READ)
        self.eidx_mm = mmap.mmap(self._eidx.fileno(), 0, access=mmap.ACCESS_READ)

        if self.bin_mm[:8] != BIN_MAGIC:
            raise ValueError("invalid graph magic")
        version, node_count, edge_count = struct.unpack_from("<IQQ", self.bin_mm, 8)
        if version != 1:
            raise ValueError(f"unsupported graph version {version}")
        self.node_count = int(node_count)
        self.edge_count = int(edge_count)
        self.edge_base = 28 + self.node_count * NODE_RECORD

        if self.nidx_mm[:8] != NDX_MAGIC or self.eidx_mm[:8] != NDX_MAGIC:
            raise ValueError("invalid index magic")
        self.nidx_rows = struct.unpack_from("<Q", self.nidx_mm, 12)[0]
        self.eidx_rows = struct.unpack_from("<Q", self.eidx_mm, 12)[0]
        self.nidx_table = INDEX_HEADER
        self.eidx_table = INDEX_HEADER

    def close(self) -> None:
        for mm in (self.bin_mm, self.nidx_mm, self.eidx_mm):
            if mm is not None:
                mm.close()
        for f in (self._bin, self._nidx, self._eidx):
            if f is not None:
                f.close()

    def _find_offset(self, mm: mmap.mmap, table_off: int, row_count: int, target: int) -> Optional[int]:
        lo, hi = 0, row_count - 1
        while lo <= hi:
            mid = (lo + hi) // 2
            off = table_off + mid * 16
            row_id, row_off = struct.unpack_from("<qQ", mm, off)
            if row_id == target:
                return int(row_off)
            if row_id < target:
                lo = mid + 1
            else:
                hi = mid - 1
        return None

    def node_latlon(self, node_id: int) -> Optional[Tuple[float, float]]:
        off = self._find_offset(self.nidx_mm, self.nidx_table, self.nidx_rows, node_id)
        if off is None or off + NODE_RECORD > len(self.bin_mm):
            return None
        nid, lat, lon, _kind = struct.unpack_from("<qddi", self.bin_mm, off)
        if nid != node_id:
            return None
        return float(lat), float(lon)

    def edge_endpoints(self, edge_id: int) -> Optional[Tuple[int, int, int]]:
        off = self._find_offset(self.eidx_mm, self.eidx_table, self.eidx_rows, edge_id)
        if off is None or off + EDGE_RECORD > len(self.bin_mm):
            return None
        eid, fr, to, etype = struct.unpack_from("<qqqi", self.bin_mm, off)
        if eid != edge_id:
            return None
        return int(fr), int(to), int(etype)


def collect_edge_ids_in_bbox(
    sidx_path: str, min_lon: float, min_lat: float, max_lon: float, max_lat: float
) -> Set[int]:
    with open(sidx_path, "rb") as f:
        if f.read(8) != SIDX_MAGIC:
            raise ValueError("invalid sidx magic")
        version, cell_size, cell_count = struct.unpack("<IdI", f.read(16))
        if version != 1:
            raise ValueError(f"unsupported sidx version {version}")

        min_gx, min_gy = _cell_of(min_lat, min_lon, cell_size)
        max_gx, max_gy = _cell_of(max_lat, max_lon, cell_size)

        edge_ids: Set[int] = set()
        for _ in range(cell_count):
            gx, gy, n = struct.unpack("<iiQ", f.read(16))
            payload = f.read(8 * n)
            if gx < min_gx or gx > max_gx or gy < min_gy or gy > max_gy:
                continue
            for i in range(n):
                eid = struct.unpack_from("<q", payload, i * 8)[0]
                edge_ids.add(int(eid))
        return edge_ids


def segment_in_bbox(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
    min_lon: float,
    min_lat: float,
    max_lon: float,
    max_lat: float,
) -> bool:
    if (
        min_lon <= lon1 <= max_lon
        and min_lat <= lat1 <= max_lat
        or min_lon <= lon2 <= max_lon
        and min_lat <= lat2 <= max_lat
    ):
        return True
    mid_lat = 0.5 * (lat1 + lat2)
    mid_lon = 0.5 * (lon1 + lon2)
    return min_lon <= mid_lon <= max_lon and min_lat <= mid_lat <= max_lat


def _default_max_features(min_lon: float, min_lat: float, max_lon: float, max_lat: float) -> int:
    span = max(max_lon - min_lon, max_lat - min_lat, 0.01)
    if span <= 0.08:
        return 35000
    if span <= 0.25:
        return 25000
    if span <= 0.6:
        return 18000
    return 12000


def export_roads_geojson(
    graph: GraphMmap,
    min_lon: float,
    min_lat: float,
    max_lon: float,
    max_lat: float,
    max_features: int = 0,
) -> Dict[str, Any]:
    if max_features <= 0:
        max_features = _default_max_features(min_lon, min_lat, max_lon, max_lat)

    sidx_path = graph.base + ".sidx"
    edge_ids = collect_edge_ids_in_bbox(sidx_path, min_lon, min_lat, max_lon, max_lat)

    # Bucket by grid so sampling covers the whole bbox (not just one dense cluster).
    grid_n = 12
    lon_span = max(max_lon - min_lon, 1e-9)
    lat_span = max(max_lat - min_lat, 1e-9)
    buckets: Dict[Tuple[int, int], List[Dict[str, Any]]] = {}
    road_n = rail_n = 0

    for eid in edge_ids:
        ep = graph.edge_endpoints(eid)
        if ep is None:
            continue
        fr, to, etype = ep
        a = graph.node_latlon(fr)
        b = graph.node_latlon(to)
        if a is None or b is None:
            continue
        lat1, lon1 = a
        lat2, lon2 = b
        if not segment_in_bbox(lat1, lon1, lat2, lon2, min_lon, min_lat, max_lon, max_lat):
            continue
        mid_lat = 0.5 * (lat1 + lat2)
        mid_lon = 0.5 * (lon1 + lon2)
        gx = min(grid_n - 1, int((mid_lon - min_lon) / lon_span * grid_n))
        gy = min(grid_n - 1, int((mid_lat - min_lat) / lat_span * grid_n))
        if etype == 0:
            road_n += 1
            feat_type = "road"
        else:
            rail_n += 1
            feat_type = "rail"
        feat = {
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": [[lon1, lat1], [lon2, lat2]],
            },
            "properties": {
                "id": eid,
                "type": feat_type,
            },
        }
        buckets.setdefault((gx, gy), []).append(feat)

    features: List[Dict[str, Any]] = []
    while len(features) < max_features:
        added = False
        for key in sorted(buckets.keys()):
            bucket = buckets[key]
            if not bucket:
                continue
            features.append(bucket.pop(0))
            added = True
            if len(features) >= max_features:
                break
        if not added:
            break

    return {
        "type": "FeatureCollection",
        "properties": {
            "bbox": [min_lon, min_lat, max_lon, max_lat],
            "features": len(features),
            "road": road_n,
            "rail": rail_n,
            "truncated": len(features) >= max_features,
            "source": "local_graph",
        },
        "features": features,
    }


def bbox_from_points(
    points: List[Tuple[float, float]], padding_frac: float = 0.35, min_pad_deg: float = 0.06
) -> Tuple[float, float, float, float]:
    if not points:
        return 87.45, 43.75, 87.75, 43.95
    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)
    span_lat = max(max_lat - min_lat, 0.012)
    span_lon = max(max_lon - min_lon, 0.012)
    pad_lat = max(min_pad_deg, span_lat * padding_frac)
    pad_lon = max(min_pad_deg, span_lon * padding_frac)
    return min_lon - pad_lon, min_lat - pad_lat, max_lon + pad_lon, max_lat + pad_lat
