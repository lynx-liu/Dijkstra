#!/usr/bin/env python3
"""
HTTP wrapper for mmlp_service (streaming fleet meeting prediction).
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import unquote, urlparse


def _drain_stderr(proc: subprocess.Popen) -> None:
    if not proc.stderr:
        return
    for line in proc.stderr:
        sys.stderr.write("[mmlp] " + line)


class MapState:
    """In-memory fleet + meeting snapshot for the live map."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.vehicles: dict[str, dict[str, Any]] = {}
        self.focal = ""
        self.meetings: list[dict[str, Any]] = []
        self.mode = "none"
        self.updated_at = 0.0

    @staticmethod
    def _vehicle_row(v: dict[str, Any]) -> dict[str, Any]:
        return {
            "id": v.get("id", ""),
            "lat": v.get("lat"),
            "lon": v.get("lon"),
            "speed": v.get("speed", 0),
            "timestamp": v.get("timestamp", 0),
            "type": v.get("type", "truck"),
        }

    def upsert_vehicle(self, v: dict[str, Any]) -> None:
        vid = v.get("id")
        if not vid:
            return
        with self._lock:
            self._upsert_vehicle_unlocked(v)

    def _upsert_vehicle_unlocked(self, v: dict[str, Any]) -> None:
        vid = v.get("id")
        if not vid:
            return
        self.vehicles[str(vid)] = self._vehicle_row(v)
        self.updated_at = time.time()

    def update_single(self, vehicle: dict[str, Any], result: dict[str, Any]) -> None:
        with self._lock:
            self._upsert_vehicle_unlocked(vehicle)
            self.mode = "single"
            self.focal = str(result.get("focal", vehicle.get("id", "")))
            if result.get("found"):
                self.meetings = [result]
            else:
                self.meetings = []
            self.updated_at = time.time()

    def update_batch(self, vehicles: list[dict[str, Any]], result: dict[str, Any]) -> None:
        with self._lock:
            for v in vehicles:
                self._upsert_vehicle_unlocked(v)
            self.mode = "batch"
            self.focal = str(result.get("focal", vehicles[0].get("id", "") if vehicles else ""))
            self.meetings = list(result.get("meetings", []))
            self.updated_at = time.time()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "vehicles": list(self.vehicles.values()),
                "focal": self.focal,
                "meetings": self.meetings,
                "mode": self.mode,
                "updatedAt": self.updated_at,
            }


class MmlpCore:
    def __init__(self, graph: str, padding_m: float, binary: str, load_mode: str):
        env = os.environ.copy()
        env["MMLP_LOAD_MODE"] = load_mode
        cmd = [binary, "--graph", graph, "--padding-m", str(padding_m), "--load-mode", load_mode]
        self._ready = False
        self._load_error: str | None = None
        self._lock = threading.Lock()
        self._ready_cond = threading.Condition()
        self.map_state = MapState()

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
            text=True,
            bufsize=1,
            env=env,
        )
        threading.Thread(target=_drain_stderr, args=(self._proc,), daemon=True).start()

        self._wait_service_ready()
        if self._load_error:
            raise RuntimeError(self._load_error)

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
        line = json.dumps(payload, separators=(",", ":"))
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

    try:
        core = MmlpCore(args.graph, args.padding_m, args.binary, args.load_mode)
    except RuntimeError as e:
        print(f"ERROR: failed to start: {e}", file=sys.stderr)
        sys.exit(1)

    server = ThreadingHTTPServer((args.host, args.port), make_handler(core, web_root))
    print(f"http://127.0.0.1:{args.port}/map  (live fleet map)")
    print(f"http://127.0.0.1:{args.port}/api/vehicle  (GET /health)")
    print(f"http://127.0.0.1:{args.port}/api/meetings/lead  (batch vs first vehicle)")
    print("Service is ready — POST returns meeting results, not loading.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        core.close()


if __name__ == "__main__":
    main()
