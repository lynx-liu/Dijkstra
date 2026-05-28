#pragma once

#include "mmlp/bbox.hpp"
#include "mmlp/graph.hpp"
#include "mmlp/routing.hpp"
#include "mmlp/types.hpp"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mmlp {

// Uniform grid for fast nearest-edge map matching (nationwide graphs).
class SpatialIndex {
 public:
  void build(const MultimodalGraph& graph, double cellSizeDeg = 0.02);

  // Load prebuilt .sidx (seconds at startup vs minutes building from full graph).
  bool loadFromFile(const std::string& path, std::string* error = nullptr);

  bool nearestEdge(const MultimodalGraph& graph, double lat, double lon, VehicleType type,
                   GraphPosition& out, double* distanceMeters = nullptr) const;

  void collectEdgesInBBox(const GeoBBox& bbox, std::unordered_set<int64_t>& edgeIds) const;

  // Edges with at least one endpoint within radiusMeters of (lat, lon).
  void collectEdgesInRadius(const MultimodalGraph& graph, double lat, double lon,
                            double radiusMeters, std::unordered_set<int64_t>& edgeIds) const;

 private:
  double cellSizeDeg_ = 0.02;
  std::unordered_map<int64_t, std::vector<int64_t>> cells_;
  static int64_t cellKey(int gx, int gy);
  void cellOf(double lat, double lon, int& gx, int& gy) const;
};

}  // namespace mmlp
