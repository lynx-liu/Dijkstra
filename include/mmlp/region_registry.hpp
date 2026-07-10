#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace mmlp {

struct RegionBBoxView {
  const char* suffix = nullptr;
  const char* name = nullptr;
  double minLon = 0.0;
  double minLat = 0.0;
  double maxLon = 0.0;
  double maxLat = 0.0;
};

// All configured China regions (province/metro tiles), priority order.
const RegionBBoxView* chinaRegions();
std::size_t chinaRegionCount();

// Best matching region suffix for a point. Prefers province/municipality tiles
// over metro overlays (hk/mo/prd) when both contain the point.
const char* regionSuffixForPoint(double lat, double lon);

// Prefer provincial regional graph file that exists on disk (skips metro
// overlays when a province tile also matches).
std::string resolveRegionalSuffix(const std::string& nationalGraphPath, double lat, double lon);

std::string regionalGraphPath(const std::string& nationalPath, const std::string& suffix);

bool regionalGraphFileExists(const std::string& nationalPath, const std::string& suffix);

// True when (lat,lon) lies inside the configured bbox for suffix.
bool pointInRegionSuffix(const std::string& suffix, double lat, double lon);

// Region bbox for suffix, or nullptr if unknown.
const RegionBBoxView* regionBBoxForSuffix(const std::string& suffix);

// Closest point on the region bbox boundary to (lat,lon) — used as a
// cross-province "exit" when routing on a home-region full.ch.
void regionBorderToward(const RegionBBoxView& box, double lat, double lon, double& outLat,
                        double& outLon);

// Ordered unique region suffixes along the geodesic from A to B (inclusive),
// only regions that have a graph file under nationalGraphPath.
std::vector<std::string> corridorRegionSuffixes(const std::string& nationalGraphPath, double lat0,
                                               double lon0, double lat1, double lon1,
                                               int samples = 24);

}  // namespace mmlp
