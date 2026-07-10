#include "mmlp/region_registry.hpp"

#include "mmlp/china_regions.generated.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

namespace mmlp {
namespace {

using region_detail::kChinaRegions;
using region_detail::kChinaRegionCount;

bool pointInBox(double lat, double lon, const region_detail::RegionDef& r) {
  return lon >= r.minLon && lon <= r.maxLon && lat >= r.minLat && lat <= r.maxLat;
}

}  // namespace

const RegionBBoxView* chinaRegions() {
  static RegionBBoxView views[64];
  static bool init = false;
  if (!init) {
    for (std::size_t i = 0; i < kChinaRegionCount; ++i) {
      views[i].suffix = kChinaRegions[i].suffix;
      views[i].name = kChinaRegions[i].name;
      views[i].minLon = kChinaRegions[i].minLon;
      views[i].minLat = kChinaRegions[i].minLat;
      views[i].maxLon = kChinaRegions[i].maxLon;
      views[i].maxLat = kChinaRegions[i].maxLat;
    }
    init = true;
  }
  return views;
}

std::size_t chinaRegionCount() { return kChinaRegionCount; }

bool isMetroOverlaySuffix(const char* suffix) {
  return suffix != nullptr &&
         (std::strcmp(suffix, "hk") == 0 || std::strcmp(suffix, "mo") == 0 ||
          std::strcmp(suffix, "prd") == 0);
}

const char* regionSuffixForPoint(double lat, double lon) {
  // Prefer province/municipality tiles over metro overlays (hk/mo/prd). Overlays
  // win on raw priority but are too small for cross-province corridor exits.
  const region_detail::RegionDef* best = nullptr;
  const region_detail::RegionDef* bestMetro = nullptr;
  for (std::size_t i = 0; i < kChinaRegionCount; ++i) {
    const region_detail::RegionDef& r = kChinaRegions[i];
    if (!pointInBox(lat, lon, r)) {
      continue;
    }
    if (isMetroOverlaySuffix(r.suffix)) {
      if (bestMetro == nullptr || r.priority < bestMetro->priority) {
        bestMetro = &r;
      }
      continue;
    }
    if (best == nullptr || r.priority < best->priority) {
      best = &r;
    }
  }
  if (best != nullptr) {
    return best->suffix;
  }
  return bestMetro != nullptr ? bestMetro->suffix : nullptr;
}

std::string regionalGraphPath(const std::string& nationalPath, const std::string& suffix) {
  const std::size_t slash = nationalPath.find_last_of('/');
  const std::string dir = slash == std::string::npos ? "" : nationalPath.substr(0, slash + 1);
  const std::string name =
      slash == std::string::npos ? nationalPath : nationalPath.substr(slash + 1);
  std::string base = name;
  if (base.size() > 4 && base.substr(base.size() - 4) == ".bin") {
    base = base.substr(0, base.size() - 4);
  }
  if (base.size() > 5 && base.substr(base.size() - 5) == ".mmlp") {
    base = base.substr(0, base.size() - 5);
  }
  return dir + base + "_" + suffix + ".mmlp.bin";
}

bool regionalGraphFileExists(const std::string& nationalPath, const std::string& suffix) {
  std::ifstream probe(regionalGraphPath(nationalPath, suffix));
  return probe.good();
}

std::string resolveRegionalSuffix(const std::string& nationalPath, double lat, double lon) {
  // Same metro-skip policy as regionSuffixForPoint: Shenzhen must resolve to gd
  // (not hk/prd) so home first-legs exit toward the real provincial border.
  const region_detail::RegionDef* best = nullptr;
  const region_detail::RegionDef* bestMetro = nullptr;
  for (std::size_t i = 0; i < kChinaRegionCount; ++i) {
    const region_detail::RegionDef& r = kChinaRegions[i];
    if (!pointInBox(lat, lon, r)) {
      continue;
    }
    if (!regionalGraphFileExists(nationalPath, r.suffix)) {
      continue;
    }
    if (isMetroOverlaySuffix(r.suffix)) {
      if (bestMetro == nullptr || r.priority < bestMetro->priority) {
        bestMetro = &r;
      }
      continue;
    }
    if (best == nullptr || r.priority < best->priority) {
      best = &r;
    }
  }
  if (best != nullptr) {
    return best->suffix;
  }
  return bestMetro != nullptr ? bestMetro->suffix : std::string{};
}

bool pointInRegionSuffix(const std::string& suffix, double lat, double lon) {
  for (std::size_t i = 0; i < kChinaRegionCount; ++i) {
    const region_detail::RegionDef& r = kChinaRegions[i];
    if (suffix != r.suffix) {
      continue;
    }
    return pointInBox(lat, lon, r);
  }
  return false;
}

const RegionBBoxView* regionBBoxForSuffix(const std::string& suffix) {
  const RegionBBoxView* views = chinaRegions();
  for (std::size_t i = 0; i < kChinaRegionCount; ++i) {
    if (suffix == views[i].suffix) {
      return &views[i];
    }
  }
  return nullptr;
}

void regionBorderToward(const RegionBBoxView& box, double lat, double lon, double& outLat,
                        double& outLon) {
  // Prefer the ray from the region center toward the target, intersecting the
  // bbox boundary. Axis-aligned clamp alone often lands on a corner (e.g.
  // Guangdong NW corner for Chengdu), which is frequently off-network.
  const double cx = 0.5 * (box.minLon + box.maxLon);
  const double cy = 0.5 * (box.minLat + box.maxLat);
  const double dx = lon - cx;
  const double dy = lat - cy;
  if (std::fabs(dx) < 1e-12 && std::fabs(dy) < 1e-12) {
    outLon = cx;
    outLat = cy;
    return;
  }
  // If target already inside, keep it (overlapping provinces / dest in home).
  if (lon >= box.minLon && lon <= box.maxLon && lat >= box.minLat && lat <= box.maxLat) {
    outLon = lon;
    outLat = lat;
    return;
  }
  double tMin = 1e300;
  auto consider = [&](double t, double x, double y) {
    if (t > 1e-9 && t < tMin) {
      // Must land on the segment (within bbox, with tiny slack).
      if (x >= box.minLon - 1e-6 && x <= box.maxLon + 1e-6 && y >= box.minLat - 1e-6 &&
          y <= box.maxLat + 1e-6) {
        tMin = t;
        outLon = std::min(box.maxLon, std::max(box.minLon, x));
        outLat = std::min(box.maxLat, std::max(box.minLat, y));
      }
    }
  };
  if (std::fabs(dx) > 1e-12) {
    consider((box.minLon - cx) / dx, box.minLon, cy + dy * ((box.minLon - cx) / dx));
    consider((box.maxLon - cx) / dx, box.maxLon, cy + dy * ((box.maxLon - cx) / dx));
  }
  if (std::fabs(dy) > 1e-12) {
    consider((box.minLat - cy) / dy, cx + dx * ((box.minLat - cy) / dy), box.minLat);
    consider((box.maxLat - cy) / dy, cx + dx * ((box.maxLat - cy) / dy), box.maxLat);
  }
  if (tMin > 1e299) {
    // Fallback: axis-aligned clamp.
    outLon = std::min(box.maxLon, std::max(box.minLon, lon));
    outLat = std::min(box.maxLat, std::max(box.minLat, lat));
  }
}

std::vector<std::string> corridorRegionSuffixes(const std::string& nationalGraphPath, double lat0,
                                               double lon0, double lat1, double lon1,
                                               int samples) {
  std::vector<std::string> out;
  if (samples < 2) {
    samples = 2;
  }
  std::string prev;
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const double lat = lat0 + (lat1 - lat0) * t;
    const double lon = lon0 + (lon1 - lon0) * t;
    const std::string suffix = resolveRegionalSuffix(nationalGraphPath, lat, lon);
    if (suffix.empty() || suffix == prev) {
      continue;
    }
    out.push_back(suffix);
    prev = suffix;
  }
  return out;
}

}  // namespace mmlp
