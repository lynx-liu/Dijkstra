#include "mmlp/geo.hpp"

#include <algorithm>
#include <cmath>

namespace mmlp {

namespace {

constexpr double kEarthRadiusMeters = 6371000.0;

}  // namespace

Vec2 latLonToLocalMeters(const LatLon& point, const LatLon& origin) {
  const double latRad = point.lat * M_PI / 180.0;
  const double lonRad = point.lon * M_PI / 180.0;
  const double originLatRad = origin.lat * M_PI / 180.0;
  const double originLonRad = origin.lon * M_PI / 180.0;
  const double x = (lonRad - originLonRad) * std::cos(originLatRad) * kEarthRadiusMeters;
  const double y = (latRad - originLatRad) * kEarthRadiusMeters;
  return {x, y};
}

LatLon localMetersToLatLon(const Vec2& local, const LatLon& origin) {
  const double originLatRad = origin.lat * M_PI / 180.0;
  const double originLonRad = origin.lon * M_PI / 180.0;
  const double latRad = originLatRad + local.y / kEarthRadiusMeters;
  const double lonRad = originLonRad + local.x / (kEarthRadiusMeters * std::cos(originLatRad));
  return {latRad * 180.0 / M_PI, lonRad * 180.0 / M_PI};
}

GeoBBox bboxAroundSegment(const LatLon& a, const LatLon& b, double paddingMeters) {
  const double dLat = paddingMeters / 111000.0;
  const double midLat = 0.5 * (a.lat + b.lat);
  const double cosLat = std::max(0.2, std::cos(midLat * M_PI / 180.0));
  const double dLon = paddingMeters / (111000.0 * cosLat);
  GeoBBox box;
  box.minLat = std::min(a.lat, b.lat) - dLat;
  box.maxLat = std::max(a.lat, b.lat) + dLat;
  box.minLon = std::min(a.lon, b.lon) - dLon;
  box.maxLon = std::max(a.lon, b.lon) + dLon;
  return box;
}

double pointToSegmentDistanceLatLon(const LatLon& point, const LatLon& segA,
                                     const LatLon& segB) {
  const LatLon origin = point;
  const Vec2 p = latLonToLocalMeters(point, origin);
  const Vec2 a = latLonToLocalMeters(segA, origin);
  const Vec2 b = latLonToLocalMeters(segB, origin);
  return pointToSegmentDistanceMeters(p, a, b, nullptr);
}

double haversineMeters(const LatLon& a, const LatLon& b) {
  const double dLat = (b.lat - a.lat) * M_PI / 180.0;
  const double dLon = (b.lon - a.lon) * M_PI / 180.0;
  const double lat1 = a.lat * M_PI / 180.0;
  const double lat2 = b.lat * M_PI / 180.0;
  const double h =
      std::sin(dLat / 2) * std::sin(dLat / 2) +
      std::cos(lat1) * std::cos(lat2) * std::sin(dLon / 2) * std::sin(dLon / 2);
  return 2.0 * kEarthRadiusMeters * std::asin(std::min(1.0, std::sqrt(h)));
}

double pointToSegmentDistanceMeters(const Vec2& p, const Vec2& a, const Vec2& b,
                                    double* tOut) {
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double apx = p.x - a.x;
  const double apy = p.y - a.y;
  const double abLen2 = abx * abx + aby * aby;
  double t = 0.0;
  if (abLen2 > 1e-9) {
    t = (apx * abx + apy * aby) / abLen2;
    t = std::max(0.0, std::min(1.0, t));
  }
  const double cx = a.x + t * abx;
  const double cy = a.y + t * aby;
  const double dx = p.x - cx;
  const double dy = p.y - cy;
  if (tOut) {
    *tOut = t;
  }
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace mmlp
