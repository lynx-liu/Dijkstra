#!/usr/bin/env python3
"""
HTTP wrapper for mmlp_service (streaming fleet meeting prediction).
"""


import argparse
import json
import mimetypes
import os
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, unquote, urlencode, urlparse
import urllib.error
import urllib.request

try:
    from http.server import ThreadingHTTPServer
except ImportError:
    class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True

# map_roads lives in tools/
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from tools.map_roads import GraphMmap, bbox_from_points, export_roads_geojson
from tools.map_tiles import MBTilesStore, resolve_mbtiles_path
from tools.map_tile_render import GraphTileRenderer


# Offline fallback when Nominatim is unreachable (China / 新疆常用地名).
_LOCAL_PLACES = (
    {"name": "乌鲁木齐", "lat": 43.8256, "lon": 87.6168},
    {"name": "乌鲁木齐市", "lat": 43.8256, "lon": 87.6168},
    {"name": "北京", "lat": 39.9042, "lon": 116.4074},
    {"name": "上海", "lat": 31.2304, "lon": 121.4737},
    {"name": "广州", "lat": 23.1291, "lon": 113.2644},
    {"name": "深圳", "lat": 22.5431, "lon": 114.0579},
    {"name": "成都", "lat": 30.5728, "lon": 104.0668},
    {"name": "西安", "lat": 34.3416, "lon": 108.9398},
    {"name": "兰州", "lat": 36.0611, "lon": 103.8343},
    {"name": "喀什", "lat": 39.4704, "lon": 75.9896},
    {"name": "伊犁", "lat": 43.9219, "lon": 81.3240},
    {"name": "克拉玛依", "lat": 45.5795, "lon": 84.8892},
    {"name": "吐鲁番", "lat": 42.9513, "lon": 89.1898},
    {"name": "哈密", "lat": 42.8185, "lon": 93.5142},
    {"name": "阿克苏", "lat": 41.1717, "lon": 80.2651},
    {"name": "库尔勒", "lat": 41.7269, "lon": 86.1746},
    {"name": "石河子", "lat": 44.3054, "lon": 86.0806},
    {"name": "昌吉", "lat": 44.0146, "lon": 87.3040},
    {"name": "阿勒泰", "lat": 47.8484, "lon": 88.1387},
    {"name": "塔城", "lat": 46.7454, "lon": 82.9789},
)


def _local_geocode(query: str, limit: int) -> List[Dict[str, Any]]:
    q = query.strip()
    if not q:
        return []
    out: List[Dict[str, Any]] = []
    for row in _LOCAL_PLACES:
        name = row["name"]
        if q in name or name in q:
            out.append({"name": name, "lat": row["lat"], "lon": row["lon"], "source": "local"})
    return out[:limit]


def geocode_places(query: str, limit: int = 5) -> List[Dict[str, Any]]:
    q = query.strip()
    if not q:
        return []
    limit = max(1, min(limit, 10))
    url = "https://nominatim.openstreetmap.org/search?" + urlencode(
        {
            "q": q,
            "format": "json",
            "limit": str(limit),
            "countrycodes": "cn",
            "accept-language": "zh",
        }
    )
    try:
        req = urllib.request.Request(
            url,
            headers={"User-Agent": "MMLP/1.0 (fleet map; contact: local deploy)"},
        )
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        results: List[Dict[str, Any]] = []
        for row in data:
            try:
                results.append(
                    {
                        "name": row.get("display_name") or q,
                        "lat": float(row["lat"]),
                        "lon": float(row["lon"]),
                        "source": "nominatim",
                    }
                )
            except (KeyError, TypeError, ValueError):
                continue
        if results:
            return results
    except (urllib.error.URLError, urllib.error.HTTPError, json.JSONDecodeError, ValueError) as e:
        sys.stderr.write("[geocode] nominatim failed: %s\n" % e)
    return _local_geocode(q, limit)


def _drain_stderr(proc: subprocess.Popen) -> None:
    if not proc.stderr:
        return
    for line in proc.stderr:
        sys.stderr.write("[mmlp] " + line)


def _iso_utc(ts: Optional[float] = None) -> str:
    dt = datetime.fromtimestamp(ts if ts is not None else time.time(), tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%SZ")


class MapState:
    """In-memory fleet + meeting snapshot for the live map."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.vehicles: Dict[str, Dict[str, Any]] = {}
        self.focal = ""
        self.meetings: List[Dict[str, Any]] = []
        self.destination: Optional[Dict[str, Any]] = None
        self.arrivals: List[Dict[str, Any]] = []
        self.sort_by = "duration"
        self.mode = "none"
        self.updated_at = 0.0

    @staticmethod
    def _vehicle_row(v: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "id": v.get("id", ""),
            "lat": v.get("lat"),
            "lon": v.get("lon"),
            "speed": v.get("speed", 0),
            "time": v.get("time") or v.get("observedAt") or "",
            "type": v.get("type", "truck"),
        }

    def upsert_vehicle(self, v: Dict[str, Any]) -> None:
        vid = v.get("id")
        if not vid:
            return
        with self._lock:
            self._upsert_vehicle_unlocked(v)

    def _upsert_vehicle_unlocked(self, v: Dict[str, Any]) -> None:
        vid = v.get("id")
        if not vid:
            return
        self.vehicles[str(vid)] = self._vehicle_row(v)
        self.updated_at = time.time()

    def update_single(self, vehicle: Dict[str, Any], result: Dict[str, Any]) -> None:
        with self._lock:
            self._upsert_vehicle_unlocked(vehicle)
            self.mode = "single"
            self.focal = str(result.get("focal", vehicle.get("id", "")))
            if result.get("found"):
                self.meetings = [result]
            else:
                self.meetings = []
            self.destination = None
            self.arrivals = []
            self.updated_at = time.time()

    def update_batch(self, vehicles: List[Dict[str, Any]], result: Dict[str, Any]) -> None:
        with self._lock:
            for v in vehicles:
                self._upsert_vehicle_unlocked(v)
            self.mode = "batch"
            self.focal = str(result.get("focal", vehicles[0].get("id", "") if vehicles else ""))
            self.meetings = list(result.get("meetings", []))
            self.destination = None
            self.arrivals = []
            self.updated_at = time.time()

    def update_arrival(
        self,
        vehicles: Optional[List[Dict[str, Any]]],
        result: Dict[str, Any],
    ) -> None:
        with self._lock:
            if vehicles:
                for v in vehicles:
                    self._upsert_vehicle_unlocked(v)
            self.mode = "arrival"
            self.focal = ""
            self.meetings = []
            self.destination = dict(result.get("destination") or {})
            self.arrivals = list(result.get("vehicles") or [])
            self.sort_by = str(result.get("sortBy") or "duration")
            self.updated_at = time.time()

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "vehicles": list(self.vehicles.values()),
                "focal": self.focal,
                "meetings": self.meetings,
                "destination": self.destination,
                "arrivals": self.arrivals,
                "sortBy": self.sort_by,
                "mode": self.mode,
                "updatedAt": _iso_utc(self.updated_at),
            }


class RoadBasemap:
    """Local road/rail GeoJSON from graph indexes (no external map tiles)."""

    def __init__(self, graph_path: str) -> None:
        self._graph_path = graph_path
        self._lock = threading.Lock()
        self._graph: Optional[GraphMmap] = None
        self._cache: Dict[str, Tuple[float, Dict[str, Any]]] = {}
        self._cache_max = 24

    def _ensure(self) -> GraphMmap:
        if self._graph is None:
            self._graph = GraphMmap(self._graph_path)
        return self._graph

    def _cache_get(self, key: str) -> Optional[Dict[str, Any]]:
        row = self._cache.get(key)
        if row is None:
            return None
        return row[1]

    def _cache_put(self, key: str, body: Dict[str, Any]) -> None:
        self._cache[key] = (time.time(), body)
        if len(self._cache) > self._cache_max:
            oldest = min(self._cache.items(), key=lambda x: x[1][0])[0]
            del self._cache[oldest]

    def export_bbox(
        self,
        min_lon: float,
        min_lat: float,
        max_lon: float,
        max_lat: float,
        max_features: int = 0,
    ) -> Dict[str, Any]:
        key = f"{min_lon:.5f},{min_lat:.5f},{max_lon:.5f},{max_lat:.5f}:{max_features}"
        cached = self._cache_get(key)
        if cached is not None:
            return cached
        with self._lock:
            cached = self._cache_get(key)
            if cached is not None:
                return cached
            graph = self._ensure()
            body = export_roads_geojson(
                graph, min_lon, min_lat, max_lon, max_lat, max_features=max_features
            )
            self._cache_put(key, body)
            return body

    def export_auto(self, map_state: MapState) -> Dict[str, Any]:
        snap = map_state.snapshot()
        points: List[Tuple[float, float]] = []
        for v in snap.get("vehicles", []):
            lat, lon = v.get("lat"), v.get("lon")
            if lat is not None and lon is not None:
                points.append((float(lat), float(lon)))
        for m in snap.get("meetings", []):
            if m.get("found") and m.get("lat") is not None and m.get("lon") is not None:
                points.append((float(m["lat"]), float(m["lon"])))
                for pt in m.get("routeSelf") or []:
                    if isinstance(pt, list) and len(pt) >= 2:
                        points.append((float(pt[0]), float(pt[1])))
                for pt in m.get("routePartner") or []:
                    if isinstance(pt, list) and len(pt) >= 2:
                        points.append((float(pt[0]), float(pt[1])))
        min_lon, min_lat, max_lon, max_lat = bbox_from_points(points)
        return self.export_bbox(min_lon, min_lat, max_lon, max_lat)

    def close(self) -> None:
        if self._graph is not None:
            self._graph.close()
            self._graph = None


class MmlpCore:
    def __init__(self, graph: str, padding_m: float, binary: str, load_mode: str):
        env = os.environ.copy()
        env["MMLP_LOAD_MODE"] = load_mode
        cmd = [binary, "--graph", graph, "--padding-m", str(padding_m), "--load-mode", load_mode]
        self._ready = False
        self._load_error: Optional[str] = None
        self._lock = threading.Lock()
        self._ready_cond = threading.Condition()
        self.map_state = MapState()
        self.road_basemap: Optional[RoadBasemap] = None
        self.tile_renderer: Optional[GraphTileRenderer] = None
        self.mbtiles: Optional[MBTilesStore] = None

        hint = "~5 min for full" if load_mode == "full" else "usually under 1 min for index"
        sys.stderr.write(
            f"[http] loading graph (mode={load_mode}), API port opens after ready ({hint})...\n"
        )
        sys.stderr.flush()

        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            bufsize=1,
            env=env,
        )
        threading.Thread(target=_drain_stderr, args=(self._proc,), daemon=True).start()

        self._wait_service_ready()
        if self._load_error:
            raise RuntimeError(self._load_error)

        # Defer mmap basemap until after mmlp_service ready (full graph needs all RAM).
        self.road_basemap = RoadBasemap(graph)
        self.tile_renderer = GraphTileRenderer(graph)
        mbtiles_path = resolve_mbtiles_path()
        if mbtiles_path:
            try:
                self.mbtiles = MBTilesStore(mbtiles_path)
                sys.stderr.write(f"[http] offline map tiles: {mbtiles_path}\n")
            except Exception as e:
                sys.stderr.write(f"[http] mbtiles load failed: {e}\n")

    def _wait_service_ready(self) -> None:
        assert self._proc.stdout is not None
        while True:
            line = self._proc.stdout.readline()
            if not line:
                self._load_error = "mmlp_service exited during startup"
                with self._ready_cond:
                    self._ready_cond.notify_all()
                return
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue
            if msg.get("ok"):
                with self._ready_cond:
                    self._ready = True
                    self._ready_cond.notify_all()
                sys.stderr.write("[http] mmlp_service ready — you can POST /api/vehicle now\n")
                sys.stderr.flush()
                return
            if msg.get("error"):
                self._load_error = msg.get("error", "startup failed")
                with self._ready_cond:
                    self._ready_cond.notify_all()
                return

    def wait_until_ready(self, timeout_sec: float = 600.0) -> bool:
        deadline = time.time() + timeout_sec
        with self._ready_cond:
            while not self.is_ready() and time.time() < deadline:
                if self._load_error:
                    return False
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self._ready_cond.wait(timeout=min(1.0, remaining))
        return self.is_ready()

    def is_ready(self) -> bool:
        return self._ready and self._load_error is None

    def status(self) -> dict:
        if self._load_error:
            return {"status": "error", "error": self._load_error}
        if not self._ready:
            return {
                "status": "loading",
                "message": "service starting, retry shortly or watch server log",
            }
        return {"status": "ok"}

    def _request(self, payload: dict) -> dict:
        line = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        with self._lock:
            assert self._proc.stdin and self._proc.stdout
            self._proc.stdin.write(line + "\n")
            self._proc.stdin.flush()
            out = self._proc.stdout.readline()
        if not out:
            raise RuntimeError("mmlp_service closed stdout")
        return json.loads(out)

    def ingest(self, payload: dict) -> dict:
        return self._request(payload)

    def close(self):
        if self._proc.stdin:
            self._proc.stdin.close()
        self._proc.terminate()
        if self.road_basemap is not None:
            self.road_basemap.close()
        if self.tile_renderer is not None:
            self.tile_renderer.close()
        if self.mbtiles is not None:
            self.mbtiles.close()


class ReuseThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


def reserve_listen_socket(host: str, port: int) -> socket.socket:
    """Bind port before slow graph load so two starts cannot race on 8080."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((host, port))
    except OSError as e:
        sock.close()
        raise OSError(
            f"cannot bind {host}:{port} ({e}). "
            f"Another mmlp HTTP may be running — try: curl http://127.0.0.1:{port}/health "
            f"or: pkill -f mmlp_http_server; sleep 1; bash tools/start_http_server.sh"
        ) from e
    sock.listen(128)
    sys.stderr.write(f"[http] listening on {host}:{port} (graph loading in background)...\n")
    sys.stderr.flush()
    return sock


def make_handler(core: MmlpCore, web_root: str):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

        def _json(self, code: int, body: dict):
            data = json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _serve_file(self, rel_path: str) -> bool:
            rel_path = rel_path.lstrip("/")
            if ".." in rel_path.split("/"):
                return False
            full = os.path.join(web_root, rel_path)
            if not os.path.isfile(full):
                return False
            mime, _ = mimetypes.guess_type(full)
            if mime is None:
                mime = "application/octet-stream"
            with open(full, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return True

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/health":
                code = 200 if core.is_ready() else 503
                self._json(code, core.status())
                return
            if path == "/api/map/state":
                self._json(200, core.map_state.snapshot())
                return
            if path == "/api/map/tiles/meta":
                if core.mbtiles is not None:
                    meta = dict(core.mbtiles.meta)
                    meta["source"] = "mbtiles"
                    self._json(200, meta)
                else:
                    self._json(200, core.tile_renderer.meta())
                return
            tile_parts = path.split("/")
            if len(tile_parts) == 7 and tile_parts[1:4] == ["api", "map", "tiles"]:
                fn = tile_parts[6]
                ext = None
                if fn.endswith(".png"):
                    ext = "png"
                elif fn.endswith(".pbf"):
                    ext = "pbf"
                if ext is not None:
                    try:
                        z = int(tile_parts[4])
                        x = int(tile_parts[5])
                        y = int(fn[: -len("." + ext)])
                    except ValueError:
                        self._json(400, {"error": "bad tile path"})
                        return

                    data = None
                    mbtiles_vector = (
                        core.mbtiles is not None
                        and bool(core.mbtiles.meta.get("vector"))
                    )
                    if core.mbtiles is not None and (ext == "pbf" or not mbtiles_vector):
                        data = core.mbtiles.get_tile(z, x, y)

                    if data is None and ext == "png":
                        data = core.tile_renderer.render_tile(z, x, y)

                    if data is None:
                        self.send_response(204)
                        self.end_headers()
                        return

                    if ext == "pbf":
                        mime = "application/vnd.mapbox-vector-tile"
                    else:
                        fmt = "png"
                        if core.mbtiles is not None:
                            fmt = (core.mbtiles.meta.get("format") or "png").lower()
                        mime = "image/png" if fmt == "png" else "image/jpeg"

                    self.send_response(200)
                    self.send_header("Content-Type", mime)
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Cache-Control", "public, max-age=86400")
                    if ext == "pbf" and len(data) >= 2 and data[0] == 0x1F and data[1] == 0x8B:
                        self.send_header("Content-Encoding", "gzip")
                    self.end_headers()
                    self.wfile.write(data)
                    return
            if path == "/api/map/geocode":
                qs = parse_qs(urlparse(self.path).query)
                q = (qs.get("q") or qs.get("query") or [""])[0].strip()
                if not q:
                    self._json(400, {"error": "need q= place name"})
                    return
                try:
                    limit = int((qs.get("limit") or ["5"])[0] or "5")
                except ValueError:
                    limit = 5
                try:
                    results = geocode_places(q, limit=limit)
                    self._json(200, {"query": q, "results": results})
                except Exception as e:
                    self._json(500, {"error": str(e)})
                return
            if path == "/api/map/roads":
                qs = parse_qs(urlparse(self.path).query)
                try:
                    if qs.get("auto", ["0"])[0] in ("1", "true", "yes"):
                        body = core.road_basemap.export_auto(core.map_state)
                    else:
                        min_lon = float(qs["minLon"][0])
                        min_lat = float(qs["minLat"][0])
                        max_lon = float(qs["maxLon"][0])
                        max_lat = float(qs["maxLat"][0])
                        max_features = int(qs.get("maxFeatures", ["0"])[0] or "0")
                        body = core.road_basemap.export_bbox(
                            min_lon, min_lat, max_lon, max_lat, max_features=max_features
                        )
                    self._json(200, body)
                except (KeyError, IndexError, ValueError) as e:
                    self._json(
                        400,
                        {
                            "error": "use ?auto=1 or minLon,minLat,maxLon,maxLat",
                            "detail": str(e),
                        },
                    )
                except Exception as e:
                    self._json(500, {"error": str(e)})
                return
            if path in ("/", "/map", "/map/"):
                if self._serve_file("live.html"):
                    return
            if path.startswith("/web/"):
                if self._serve_file(unquote(path[len("/web/") :])):
                    return
            self._json(404, {"error": "not found"})

        def _read_json_body(self) -> dict:
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length > 0 else b"{}"
            return json.loads(raw.decode("utf-8"))

        def do_POST(self):
            if not core.is_ready():
                if not core.wait_until_ready(600.0):
                    self._json(503, core.status())
                    return

            if self.path == "/api/vehicle":
                try:
                    payload = self._read_json_body()
                except json.JSONDecodeError as e:
                    self._json(400, {"error": "invalid json: " + str(e)})
                    return
                try:
                    result = core.ingest(payload)
                    core.map_state.update_single(payload, result)
                    self._json(200, result)
                except Exception as e:
                    self._json(500, {"error": str(e)})
                return

            if self.path == "/api/meetings/lead":
                try:
                    payload = self._read_json_body()
                except json.JSONDecodeError as e:
                    self._json(400, {"error": "invalid json: " + str(e)})
                    return
                vehicles = payload.get("vehicles")
                if not isinstance(vehicles, list) or len(vehicles) == 0:
                    self._json(400, {"error": "need non-empty vehicles array"})
                    return
                req = {"action": "meet_with_lead", "vehicles": vehicles}
                try:
                    result = core.ingest(req)
                    core.map_state.update_batch(vehicles, result)
                    self._json(200, result)
                except Exception as e:
                    self._json(500, {"error": str(e)})
                return

            if self.path in ("/api/destinations/arrive", "/api/destination/arrive"):
                try:
                    payload = self._read_json_body()
                except json.JSONDecodeError as e:
                    self._json(400, {"error": "invalid json: " + str(e)})
                    return
                for key in ("lat", "lon", "arriveBy"):
                    if key not in payload:
                        self._json(
                            400,
                            {
                                "error": "need lat, lon, arriveBy (ISO UTC, e.g. 2026-06-01T12:00:00Z)",
                            },
                        )
                        return
                req: Dict[str, Any] = {
                    "action": "destination_arrival",
                    "lat": payload["lat"],
                    "lon": payload["lon"],
                    "arriveBy": payload["arriveBy"],
                }
                if "type" in payload:
                    req["type"] = payload["type"]
                if "sortBy" in payload:
                    req["sortBy"] = payload["sortBy"]
                vehicles = payload.get("vehicles")
                if isinstance(vehicles, list) and len(vehicles) > 0:
                    req["vehicles"] = vehicles
                try:
                    result = core.ingest(req)
                    veh_list = vehicles if isinstance(vehicles, list) else None
                    core.map_state.update_arrival(veh_list, result)
                    self._json(200, result)
                except Exception as e:
                    self._json(500, {"error": str(e)})
                return

            self._json(404, {"error": "not found"})

    return Handler


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    web_root = os.path.join(root, "web")
    parser = argparse.ArgumentParser(description="MMLP HTTP meeting service")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--graph", default=os.path.join(root, "data/graph/china.mmlp.bin"))
    parser.add_argument("--padding-m", type=float, default=150000.0)
    parser.add_argument("--binary", default=os.path.join(root, "build/mmlp_service"))
    parser.add_argument(
        "--load-mode",
        default=os.environ.get("MMLP_LOAD_MODE", "index"),
        choices=("index", "full", "region"),
        help="index=load .sidx only (fast); full=entire graph in RAM (~5min)",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.binary):
        print("ERROR: build mmlp_service first", file=sys.stderr)
        sys.exit(1)

    listen_sock: Optional[socket.socket] = None
    try:
        listen_sock = reserve_listen_socket(args.host, args.port)
        core = MmlpCore(args.graph, args.padding_m, args.binary, args.load_mode)
    except OSError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    except RuntimeError as e:
        print(f"ERROR: failed to start: {e}", file=sys.stderr)
        sys.exit(1)

    server = ReuseThreadingHTTPServer(
        (args.host, args.port), make_handler(core, web_root), bind_and_activate=False
    )
    server.socket = listen_sock
    server.server_bind = lambda: None  # type: ignore[method-assign]
    server.server_activate()
    print(f"http://127.0.0.1:{args.port}/map  (live fleet map)")
    print(f"http://127.0.0.1:{args.port}/api/vehicle  (GET /health)")
    print(f"http://127.0.0.1:{args.port}/api/meetings/lead  (batch vs first vehicle)")
    print(f"http://127.0.0.1:{args.port}/api/destinations/arrive  (vehicles reaching destination by time)")
    print("Service is ready — POST returns meeting results, not loading.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        core.close()
        if listen_sock is not None:
            listen_sock.close()


if __name__ == "__main__":
    main()
