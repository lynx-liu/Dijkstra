#!/usr/bin/env python3
"""Benchmark POST /api/destinations/arrive and print timing + reachable count."""

import argparse
import json
import re
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path


def load_payload(path):
    if path and path.is_file():
        return json.loads(path.read_text(encoding="utf-8"))
    transcript = Path(
        "/root/.cursor/projects/data-Dijkstra/agent-transcripts/"
        "b07447c1-e7bc-4f1f-beee-a6ec4fb0e798/"
        "b07447c1-e7bc-4f1f-beee-a6ec4fb0e798.jsonl"
    )
    for line in transcript.read_text(encoding="utf-8").splitlines():
        if "测试一下:" not in line or "粤BM585W" not in line:
            continue
        obj = json.loads(line)
        for part in obj.get("message", {}).get("content", []):
            text = part.get("text", "")
            if "粤BM585W" in text:
                m = re.search(r"测试一下:\n(\{.*\})\n</user_query>", text, re.S)
                if m:
                    return json.loads(m.group(1))
    raise SystemExit("payload not found; pass --payload FILE")


def post(url, body, timeout):
    req = urllib.request.Request(
        url,
        data=json.dumps(body, ensure_ascii=False).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    t0 = time.perf_counter()
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        out = json.loads(resp.read())
    return time.perf_counter() - t0, out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8080/api/destinations/arrive")
    ap.add_argument("--payload", type=Path)
    ap.add_argument("--label", default="full98")
    ap.add_argument("--filter-gd", action="store_true", help="Guangdong vehicles only")
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=120.0)
    args = ap.parse_args()

    payload = load_payload(args.payload)
    if args.filter_gd:
        payload = dict(payload)
        payload["vehicles"] = [
            v
            for v in payload["vehicles"]
            if 20 <= v["lat"] <= 26 and 109 <= v["lon"] <= 118
        ]

    for i in range(args.runs):
        label = f"{args.label}" if args.runs == 1 else f"{args.label}_{i+1}"
        try:
            elapsed, out = post(args.url, payload, args.timeout)
            n = len(out.get("vehicles", []))
            print(f"{label}: {elapsed:.3f}s reachable={n}")
        except urllib.error.URLError as e:
            print(f"{label}: FAILED ({e})", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
