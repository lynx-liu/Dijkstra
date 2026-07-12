#!/usr/bin/env python3
"""Benchmark POST /api/destinations/arrive and print timing + reachable count.

Default payload ships in-repo (Guangzhou 98-vehicle case). Works on CentOS
without Cursor agent transcripts.
"""

from __future__ import print_function

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timedelta
from pathlib import Path

# Python 3.6+ compatible (CentOS 7).


def default_payload_path():
    return Path(__file__).resolve().parent / "bench_dest_arrive_guangzhou98.json"


def load_payload(path):
    path = Path(path) if path else default_payload_path()
    if not path.is_file():
        raise SystemExit(
            "payload not found: {0}\nPass --payload FILE".format(path)
        )
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def refresh_times(payload, horizon_hours):
    """Shift vehicle.time / arriveBy to 'now' so stale fixtures stay valid."""
    now = datetime.utcnow().replace(microsecond=0)
    vt = now - timedelta(minutes=5)
    ab = vt + timedelta(hours=horizon_hours)
    vt_s = vt.strftime("%Y-%m-%dT%H:%M:%SZ")
    ab_s = ab.strftime("%Y-%m-%dT%H:%M:%SZ")
    out = dict(payload)
    out["arriveBy"] = ab_s
    vehicles = []
    for v in payload.get("vehicles") or []:
        nv = dict(v)
        nv["time"] = vt_s
        vehicles.append(nv)
    out["vehicles"] = vehicles
    return out


def post(url, body, timeout):
    data = json.dumps(body, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
    )
    t0 = time.time()
    resp = urllib.request.urlopen(req, timeout=timeout)
    try:
        out = json.loads(resp.read().decode("utf-8"))
    finally:
        resp.close()
    return time.time() - t0, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8080/api/destinations/arrive")
    ap.add_argument(
        "--payload",
        type=Path,
        default=None,
        help="JSON request body (default: tools/bench_dest_arrive_guangzhou98.json)",
    )
    ap.add_argument("--label", default="full98")
    ap.add_argument("--filter-gd", action="store_true", help="Guangdong vehicles only")
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument(
        "--horizon-hours",
        type=float,
        default=24.0,
        help="arriveBy = vehicle_time + this many hours (after refresh)",
    )
    ap.add_argument(
        "--keep-times",
        action="store_true",
        help="Do not rewrite vehicle.time / arriveBy to now",
    )
    args = ap.parse_args()

    payload = load_payload(args.payload)
    if not args.keep_times:
        payload = refresh_times(payload, args.horizon_hours)
    if args.filter_gd:
        payload = dict(payload)
        payload["vehicles"] = [
            v
            for v in payload["vehicles"]
            if 20 <= v["lat"] <= 26 and 109 <= v["lon"] <= 118
        ]

    print(
        "payload vehicles={0} dest=({1},{2}) arriveBy={3}".format(
            len(payload.get("vehicles") or []),
            payload.get("lat"),
            payload.get("lon"),
            payload.get("arriveBy"),
        ),
        file=sys.stderr,
    )

    for i in range(args.runs):
        label = args.label if args.runs == 1 else "{0}_{1}".format(args.label, i + 1)
        try:
            elapsed, out = post(args.url, payload, args.timeout)
            n = len(out.get("vehicles", []))
            print("{0}: {1:.3f}s reachable={2}".format(label, elapsed, n))
        except urllib.error.URLError as e:
            print("{0}: FAILED ({1})".format(label, e), file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
