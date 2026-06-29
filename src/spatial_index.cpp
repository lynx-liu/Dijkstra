#include "mmlp/spatial_index.hpp"

#include "mmlp/geo.hpp"
#include "mmlp/graph_store.hpp"
#include "mmlp/matching.hpp"
#include "mmlp/motion.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>

namespace mmlp {

int64_t SpatialIndex::cellKey(int gx, int gy) {
  return (static_cast<int64_t>(gx) << 32) ^ static_cast<uint32_t>(gy);
}

void SpatialIndex::cellOf(double lat, double lon, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor(lat / cellSizeDeg_));
  gy = static_cast<int>(std::floor(lon / cellSizeDeg_));
}

void SpatialIndex::build(const MultimodalGraph& graph, double cellSizeDeg) {
  cellSizeDeg_ = cellSizeDeg;
  cells_.clear();
  cells_.reserve(graph.edges().size() / 8 + 1);
  const std::size_t n = graph.edges().size();
  const std::size_t reportEvery = std::max<std::size_t>(1, n / 20);
  std::size_t i = 0;
  for (const Edge& edge : graph.edges()) {
    ++i;
    const Node* from = graph.findNode(edge.from);
    const Node* to = graph.findNode(edge.to);
    if (from == nullptr || to == nullptr) {
      continue;
    }
    int fx, fy, tx, ty;
    cellOf(from->lat, from->lon, fx, fy);
    cellOf(to->lat, to->lon, tx, ty);
    cells_[cellKey(fx, fy)].push_back(edge.id);
    if (fx != tx || fy != ty) {
      cells_[cellKey(tx, ty)].push_back(edge.id);
      const double midLat = 0.5 * (from->lat + to->lat);
      const double midLon = 0.5 * (from->lon + to->lon);
      int mx, my;
      cellOf(midLat, midLon, mx, my);
      cells_[cellKey(mx, my)].push_back(edge.id);
    }
    if (i % reportEvery == 0) {
      std::cerr << "[mmlp] spatial index " << (100 * i / n) << "%\n" << std::flush;
    }
  }
  std::cerr << "[mmlp] spatial index ready cells=" << cells_.size() << "\n" << std::flush;
}

void SpatialIndex::collectEdgesInRadius(const MultimodalGraph& graph, double lat, double lon,
                                        double radiusMeters,
                                        std::unordered_set<int64_t>& edgeIds) const {
  const double dLat = radiusMeters / 111000.0;
  const double cosLat = std::max(0.2, std::cos(lat * 3.141592653589793 / 180.0));
  const double dLon = radiusMeters / (111000.0 * cosLat);
  GeoBBox box;
  box.minLat = lat - dLat;
  box.maxLat = lat + dLat;
  box.minLon = lon - dLon;
  box.maxLon = lon + dLon;
  collectEdgesInBBox(box, edgeIds);

  const LatLon center{lat, lon};
  for (auto it = edgeIds.begin(); it != edgeIds.end();) {
    const Edge* edge = graph.findEdge(*it);
    if (edge == nullptr) {
      it = edgeIds.erase(it);
      continue;
    }
    const Node* from = graph.findNode(edge->from);
    const Node* to = graph.findNode(edge->to);
    if (from == nullptr || to == nullptr) {
      it = edgeIds.erase(it);
      continue;
    }
    const double df = haversineMeters(center, {from->lat, from->lon});
    const double dt = haversineMeters(center, {to->lat, to->lon});
    if (df > radiusMeters && dt > radiusMeters) {
      it = edgeIds.erase(it);
    } else {
      ++it;
    }
  }
}

void SpatialIndex::collectEdgesInBBox(const GeoBBox& bbox,
                                    std::unordered_set<int64_t>& edgeIds) const {
  int minGx = 0;
  int minGy = 0;
  int maxGx = 0;
  int maxGy = 0;
  cellOf(bbox.minLat, bbox.minLon, minGx, minGy);
  cellOf(bbox.maxLat, bbox.maxLon, maxGx, maxGy);
  for (int gx = minGx; gx <= maxGx; ++gx) {
    for (int gy = minGy; gy <= maxGy; ++gy) {
      const auto it = cells_.find(cellKey(gx, gy));
      if (it == cells_.end()) {
        continue;
      }
      for (int64_t edgeId : it->second) {
        edgeIds.insert(edgeId);
      }
    }
  }
}

bool SpatialIndex::nearestEdge(const MultimodalGraph& graph, double lat, double lon,
                               VehicleType type, GraphPosition& out,
                               double* distanceMeters) const {
  int gx, gy;
  cellOf(lat, lon, gx, gy);

  GraphPosition best;
  best.valid = false;
  double bestDist = std::numeric_limits<double>::infinity();
  const LatLon query{lat, lon};
  const Vec2 p = latLonToLocalMeters(query, query);

  for (int dx = -2; dx <= 2; ++dx) {
    for (int dy = -2; dy <= 2; ++dy) {
      const auto it = cells_.find(cellKey(gx + dx, gy + dy));
      if (it == cells_.end()) {
        continue;
      }
      for (int64_t edgeId : it->second) {
        const Edge* edge = graph.findEdge(edgeId);
        if (edge == nullptr) {
          continue;
        }
        if (type == VehicleType::TRUCK && edge->type != EdgeType::ROAD) {
          continue;
        }
        if (type == VehicleType::TRAIN && edge->type != EdgeType::RAIL) {
          continue;
        }
        const Node* from = graph.findNode(edge->from);
        const Node* to = graph.findNode(edge->to);
        if (from == nullptr || to == nullptr) {
          continue;
        }
        const Vec2 a = latLonToLocalMeters({from->lat, from->lon}, query);
        const Vec2 b = latLonToLocalMeters({to->lat, to->lon}, query);
        double t = 0.0;
        const double dist = pointToSegmentDistanceMeters(p, a, b, &t);
        if (dist < bestDist) {
          bestDist = dist;
          best.valid = true;
          best.edgeId = edge->id;
          best.nodeId = edge->from;
          best.alongMeters = t * edge->length;
        }
      }
    }
  }

  if (!best.valid || bestDist > kMaxSnapDistanceMeters) {
    return false;
  }

  const Edge* edge = graph.findEdge(best.edgeId);
  if (edge != nullptr) {
    if (best.alongMeters <= 1.0) {
      best.nodeId = edge->from;
      best.edgeId = 0;
      best.alongMeters = 0.0;
    } else if (best.alongMeters >= edge->length - 1.0) {
      best.nodeId = edge->to;
      best.edgeId = 0;
      best.alongMeters = 0.0;
    }
  }

  out = best;
  if (distanceMeters) {
    *distanceMeters = bestDist;
  }
  return true;
}

bool SpatialIndex::nearestEdgeMmap(const GraphFileStore& store, double lat, double lon,
                                   VehicleType type, GraphPosition& out,
                                   double* distanceMeters) const {
  int gx = 0;
  int gy = 0;
  cellOf(lat, lon, gx, gy);

  GraphPosition best;
  best.valid = false;
  double bestDist = std::numeric_limits<double>::infinity();
  const LatLon query{lat, lon};
  const Vec2 p = latLonToLocalMeters(query, query);

  for (int dx = -2; dx <= 2; ++dx) {
    for (int dy = -2; dy <= 2; ++dy) {
      const auto it = cells_.find(cellKey(gx + dx, gy + dy));
      if (it == cells_.end()) {
        continue;
      }
      for (int64_t edgeId : it->second) {
        EdgeType edgeType = EdgeType::ROAD;
        double length = 0.0;
        double speedLimit = 0.0;
        if (!store.readEdge(edgeId, edgeType, length, speedLimit)) {
          continue;
        }
        if (type == VehicleType::TRUCK && edgeType != EdgeType::ROAD) {
          continue;
        }
        if (type == VehicleType::TRAIN && edgeType != EdgeType::RAIL) {
          continue;
        }
        double flat = 0.0;
        double flon = 0.0;
        double tlat = 0.0;
        double tlon = 0.0;
        if (!store.edgeEndpointLatLon(edgeId, flat, flon, tlat, tlon)) {
          continue;
        }
        const Vec2 a = latLonToLocalMeters({flat, flon}, query);
        const Vec2 b = latLonToLocalMeters({tlat, tlon}, query);
        double t = 0.0;
        const double dist = pointToSegmentDistanceMeters(p, a, b, &t);
        if (dist < bestDist) {
          bestDist = dist;
          best.valid = true;
          best.edgeId = edgeId;
          int64_t from = 0;
          int64_t to = 0;
          if (store.edgeEndpoints(edgeId, from, to)) {
            best.nodeId = from;
          }
          best.alongMeters = t * length;
        }
      }
    }
  }

  if (!best.valid || bestDist > kMaxSnapDistanceMeters) {
    return false;
  }

  int64_t from = 0;
  int64_t to = 0;
  double length = 0.0;
  EdgeType edgeType = EdgeType::ROAD;
  double speedLimit = 0.0;
  if (store.edgeEndpoints(best.edgeId, from, to) &&
      store.readEdge(best.edgeId, edgeType, length, speedLimit)) {
    if (best.alongMeters <= 1.0) {
      best.nodeId = from;
      best.edgeId = 0;
      best.alongMeters = 0.0;
    } else if (best.alongMeters >= length - 1.0) {
      best.nodeId = to;
      best.edgeId = 0;
      best.alongMeters = 0.0;
    }
  }

  out = best;
  if (distanceMeters) {
    *distanceMeters = bestDist;
  }
  return true;
}

bool SpatialIndex::loadFromFile(const std::string& path, std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return fail("cannot open: " + path);
  }
  char magic[8];
  constexpr char kSidxMagic[8] = {'M', 'M', 'L', 'P', 'S', 'I', 'D', 'X'};
  if (!in.read(magic, 8) || std::memcmp(magic, kSidxMagic, 8) != 0) {
    return fail("invalid sidx magic");
  }
  uint32_t version = 0;
  double cellSize = 0.02;
  uint32_t cellCount = 0;
  if (!in.read(reinterpret_cast<char*>(&version), 4) ||
      !in.read(reinterpret_cast<char*>(&cellSize), 8) ||
      !in.read(reinterpret_cast<char*>(&cellCount), 4)) {
    return fail("truncated sidx header");
  }
  if (version != 1) {
    return fail("unsupported sidx version");
  }

  cellSizeDeg_ = cellSize;
  cells_.clear();
  cells_.reserve(cellCount + 1);

  for (uint32_t c = 0; c < cellCount; ++c) {
    int32_t gx = 0;
    int32_t gy = 0;
    uint64_t n = 0;
    if (!in.read(reinterpret_cast<char*>(&gx), 4) ||
        !in.read(reinterpret_cast<char*>(&gy), 4) ||
        !in.read(reinterpret_cast<char*>(&n), 8)) {
      return fail("truncated sidx cell");
    }
    auto& bucket = cells_[cellKey(gx, gy)];
    bucket.resize(static_cast<std::size_t>(n));
    for (uint64_t i = 0; i < n; ++i) {
      if (!in.read(reinterpret_cast<char*>(&bucket[static_cast<std::size_t>(i)]), 8)) {
        return fail("truncated sidx edge id");
      }
    }
  }

  std::cerr << "[mmlp] spatial index loaded from " << path << " cells=" << cells_.size() << "\n"
            << std::flush;
  return true;
}

}  // namespace mmlp
