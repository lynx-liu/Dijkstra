#include "mmlp/matching.hpp"

#include "mmlp/geo.hpp"
#include "mmlp/graph_store.hpp"
#include "mmlp/motion.hpp"
#include "mmlp/spatial_index.hpp"

#include <cmath>
#include <limits>

namespace mmlp {

namespace {

bool edgeAllowedForVehicle(const Edge& edge, VehicleType type) {
  if (type == VehicleType::TRUCK) {
    return edge.type == EdgeType::ROAD;
  }
  return edge.type == EdgeType::RAIL;
}

}  // namespace

GraphPosition matchVehicleToGraph(const MultimodalGraph& graph, const VehicleInfo& vehicle,
                                  const SpatialIndex* index) {
  if (index != nullptr) {
    GraphPosition snapped;
    if (index->nearestEdge(graph, vehicle.lat, vehicle.lon, vehicle.type, snapped)) {
      return snapped;
    }
    // Regional subgraphs only contain a fraction of indexed edges; scan local graph next.
  }

  GraphPosition best;
  best.valid = false;
  double bestDist = std::numeric_limits<double>::infinity();

  const LatLon query{vehicle.lat, vehicle.lon};
  LatLon origin = query;
  if (!graph.nodes().empty()) {
    origin = {graph.nodes().front().lat, graph.nodes().front().lon};
  }

  const Vec2 p = latLonToLocalMeters(query, origin);

  for (const Edge& edge : graph.edges()) {
    if (!edgeAllowedForVehicle(edge, vehicle.type)) {
      continue;
    }
    const Node* from = graph.findNode(edge.from);
    const Node* to = graph.findNode(edge.to);
    if (from == nullptr || to == nullptr) {
      continue;
    }
    const Vec2 a = latLonToLocalMeters({from->lat, from->lon}, origin);
    const Vec2 b = latLonToLocalMeters({to->lat, to->lon}, origin);
    double t = 0.0;
    const double dist = pointToSegmentDistanceMeters(p, a, b, &t);
    if (dist < bestDist) {
      bestDist = dist;
      best.valid = true;
      best.edgeId = edge.id;
      best.nodeId = edge.from;
      best.alongMeters = t * edge.length;
    }
  }

  if (!best.valid || bestDist > kMaxSnapDistanceMeters) {
    GraphPosition invalid;
    invalid.valid = false;
    return invalid;
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

  return best;
}

GraphPosition matchVehicleToGraphIndexed(const GraphFileStore& store, const SpatialIndex& index,
                                         const VehicleInfo& vehicle) {
  GraphPosition snapped;
  if (index.nearestEdgeMmap(store, vehicle.lat, vehicle.lon, vehicle.type, snapped)) {
    return snapped;
  }
  GraphPosition invalid;
  invalid.valid = false;
  return invalid;
}

}  // namespace mmlp
