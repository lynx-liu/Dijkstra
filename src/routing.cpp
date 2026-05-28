#include "mmlp/routing.hpp"

#include "mmlp/geo.hpp"
#include "mmlp/motion.hpp"

#include <limits>
#include <queue>
#include <tuple>

namespace mmlp {

namespace {

bool edgeAllowedForVehicle(EdgeType edgeType, VehicleType vehicleType) {
  if (vehicleType == VehicleType::TRUCK) {
    return edgeType == EdgeType::ROAD;
  }
  return edgeType == EdgeType::RAIL;
}

double getNodeTime(const TimeField& field, int64_t nodeId) {
  const auto it = field.atNode.find(nodeId);
  if (it == field.atNode.end()) {
    return kInfTime;
  }
  return it->second;
}

}  // namespace

TimeField computeTimeField(const MultimodalGraph& graph, const GraphPosition& start,
                           double speedMs, VehicleType type, const PredictParam& param,
                           double maxTime, const LatLon* goal) {
  TimeField field;
  if (!start.valid) {
    return field;
  }

  const double maxSpeedMs = std::max(speedMs, 5.0);
  auto heuristic = [&](int64_t nodeId) -> double {
    if (goal == nullptr) {
      return 0.0;
    }
    const Node* node = graph.findNode(nodeId);
    if (node == nullptr) {
      return 0.0;
    }
    return haversineMeters({node->lat, node->lon}, *goal) / maxSpeedMs;
  };

  using QueueItem = std::tuple<double, double, int64_t>;  // f, t, node
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

  auto relax = [&](int64_t nodeId, double t) {
    if (t > maxTime + 1e-9) {
      return;
    }
    const auto it = field.atNode.find(nodeId);
    if (it != field.atNode.end() && it->second <= t + 1e-9) {
      return;
    }
    field.atNode[nodeId] = t;
    pq.push({t + heuristic(nodeId), t, nodeId});
  };

  if (start.edgeId == 0) {
    relax(start.nodeId, 0.0);
  } else {
    const Edge* edge = graph.findEdge(start.edgeId);
    if (edge == nullptr) {
      return field;
    }
    const double toFrom = travelTimeSeconds(start.alongMeters, speedMs, type, param);
    const double toTo =
        travelTimeSeconds(std::max(0.0, edge->length - start.alongMeters), speedMs, type, param);
    relax(edge->from, toFrom);
    relax(edge->to, toTo);
  }

  const std::size_t visitCap =
      param.maxVisitedNodes > 0 ? param.maxVisitedNodes : std::numeric_limits<std::size_t>::max();

  while (!pq.empty()) {
    if (field.atNode.size() >= visitCap) {
      break;
    }
    const auto [f, t, u] = pq.top();
    pq.pop();
    (void)f;
    const auto it = field.atNode.find(u);
    if (it == field.atNode.end() || t > it->second + 1e-6) {
      continue;
    }

    for (const AdjacencyEdge& adj : graph.neighbors(u)) {
      const Edge* edge = graph.findEdge(adj.edgeId);
      if (edge == nullptr || !edgeAllowedForVehicle(edge->type, type)) {
        continue;
      }
      const double eff = edgeEffectiveSpeedMs(adj, type, speedMs);
      const double w = travelTimeSeconds(adj.length, eff, type, param);
      relax(adj.to, t + w);
    }
  }

  return field;
}

double timeAtPosition(const MultimodalGraph& graph, const TimeField& field,
                      const GraphPosition& pos, double speedMs, VehicleType type,
                      const PredictParam& param) {
  if (!pos.valid) {
    return kInfTime;
  }

  if (pos.edgeId == 0) {
    return getNodeTime(field, pos.nodeId);
  }

  const Edge* edge = graph.findEdge(pos.edgeId);
  if (edge == nullptr) {
    return kInfTime;
  }

  const double fromSide =
      getNodeTime(field, edge->from) +
      travelTimeSeconds(pos.alongMeters, speedMs, type, param);
  const double toSide = getNodeTime(field, edge->to) +
                        travelTimeSeconds(std::max(0.0, edge->length - pos.alongMeters), speedMs,
                                          type, param);
  return std::min(fromSide, toSide);
}

GraphPosition positionAtTime(const MultimodalGraph& graph, const GraphPosition& start,
                             double speedMs, VehicleType type, const PredictParam& param,
                             double timeSeconds) {
  if (!start.valid || timeSeconds <= 1e-9) {
    return start;
  }

  const TimeField field =
      computeTimeField(graph, start, speedMs, type, param, timeSeconds);

  double bestTime = -1.0;
  int64_t bestNode = 0;
  for (const auto& kv : field.atNode) {
    if (kv.second <= timeSeconds + 1e-6 && kv.second >= bestTime) {
      bestTime = kv.second;
      bestNode = kv.first;
    }
  }

  if (bestTime < 0.0) {
    return start;
  }

  if (timeSeconds - bestTime < 0.05) {
    GraphPosition onNode;
    onNode.valid = true;
    onNode.nodeId = bestNode;
    return onNode;
  }

  for (const AdjacencyEdge& adj : graph.neighbors(bestNode)) {
    const double tOther = getNodeTime(field, adj.to);
    if (tOther >= kInfTime / 2.0) {
      continue;
    }
    const Edge* edge = graph.findEdge(adj.edgeId);
    if (edge == nullptr) {
      continue;
    }

    const double need = timeSeconds - tOther;
    if (need < -1e-6) {
      continue;
    }

    double lo = 0.0;
    double hi = edge->length;
    for (int i = 0; i < 48; ++i) {
      const double mid = 0.5 * (lo + hi);
      double along = 0.0;
      if (edge->from == adj.to) {
        along = mid;
      } else {
        along = edge->length - mid;
      }
      const double tMid =
          tOther + travelTimeSeconds(along, speedMs, type, param);
      if (tMid < timeSeconds) {
        lo = mid;
      } else {
        hi = mid;
      }
    }

    GraphPosition onEdge;
    onEdge.valid = true;
    onEdge.edgeId = edge->id;
    onEdge.nodeId = edge->from;
    if (edge->from == adj.to) {
      onEdge.alongMeters = lo;
    } else {
      onEdge.alongMeters = edge->length - lo;
    }
    return onEdge;
  }

  GraphPosition onNode;
  onNode.valid = true;
  onNode.nodeId = bestNode;
  return onNode;
}

GraphPosition advancePosition(const MultimodalGraph& graph, const GraphPosition& start,
                              double speedMs, VehicleType type, const PredictParam& param,
                              double deltaSeconds) {
  return positionAtTime(graph, start, speedMs, type, param, deltaSeconds);
}

LatLon positionLatLon(const MultimodalGraph& graph, const GraphPosition& pos) {
  if (!pos.valid) {
    return {};
  }

  if (pos.edgeId == 0) {
    const Node* node = graph.findNode(pos.nodeId);
    if (node == nullptr) {
      return {};
    }
    return {node->lat, node->lon};
  }

  const Edge* edge = graph.findEdge(pos.edgeId);
  const Node* from = graph.findNode(edge->from);
  const Node* to = graph.findNode(edge->to);
  if (edge == nullptr || from == nullptr || to == nullptr || edge->length <= 1e-6) {
    return {from ? from->lat : 0.0, from ? from->lon : 0.0};
  }

  const double t = std::max(0.0, std::min(1.0, pos.alongMeters / edge->length));
  return {from->lat + t * (to->lat - from->lat), from->lon + t * (to->lon - from->lon)};
}

}  // namespace mmlp
