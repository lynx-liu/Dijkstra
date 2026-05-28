#pragma once

#include "mmlp/bbox.hpp"

namespace mmlp {

struct LatLon {
  double lat = 0.0;
  double lon = 0.0;
};

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

Vec2 latLonToLocalMeters(const LatLon& point, const LatLon& origin);
LatLon localMetersToLatLon(const Vec2& local, const LatLon& origin);

double haversineMeters(const LatLon& a, const LatLon& b);

GeoBBox bboxAroundSegment(const LatLon& a, const LatLon& b, double paddingMeters);

double pointToSegmentDistanceLatLon(const LatLon& point, const LatLon& segA, const LatLon& segB);

// Closest point on segment ab to p; returns distance (m) and t in [0,1] along segment.
double pointToSegmentDistanceMeters(const Vec2& p, const Vec2& a, const Vec2& b, double* tOut);

}  // namespace mmlp
