#!/usr/bin/env python3
"""Generate C++ region registry from config/china_regions.json."""
from __future__ import print_function

import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JSON_PATH = os.path.join(ROOT, "config", "china_regions.json")
OUT_PATH = os.path.join(ROOT, "include", "mmlp", "china_regions.generated.hpp")


def main():
    with open(JSON_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)
    regions = sorted(data["regions"], key=lambda r: (r.get("priority", 999), r["suffix"]))
    lines = [
        "// Auto-generated from config/china_regions.json — do not edit.",
        "#pragma once",
        "",
        "#include <cstddef>",
        "",
        "namespace mmlp {",
        "namespace region_detail {",
        "",
        "struct RegionDef {",
        "  const char* suffix;",
        "  const char* name;",
        "  double minLon;",
        "  double minLat;",
        "  double maxLon;",
        "  double maxLat;",
        "  double marginDeg;",
        "  bool hwyOverlay;",
        "  int priority;",
        "};",
        "",
        "inline constexpr RegionDef kChinaRegions[] = {",
    ]
    for r in regions:
        bbox = r["bbox"]
        hwy = "true" if r.get("hwy_overlay") else "false"
        margin = r.get("margin", 0.12)
        pri = r.get("priority", 999)
        lines.append(
            '  {"%s", "%s", %.6f, %.6f, %.6f, %.6f, %.4f, %s, %d},'
            % (r["suffix"], r["name"], bbox[0], bbox[1], bbox[2], bbox[3], margin, hwy, pri)
        )
    lines += [
        "};",
        "",
        "inline constexpr std::size_t kChinaRegionCount = "
        "sizeof(kChinaRegions) / sizeof(kChinaRegions[0]);",
        "",
        "}  // namespace region_detail",
        "}  // namespace mmlp",
        "",
    ]
    with open(OUT_PATH, "w", encoding="utf-8") as out:
        out.write("\n".join(lines))
    print("wrote %s (%d regions)" % (OUT_PATH, len(regions)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
