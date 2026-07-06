#include "mmlp/routing.hpp"

#include "mmlp/csr_graph.hpp"
#include "mmlp/geo.hpp"
#include "mmlp/graph_store.hpp"
#include "mmlp/motion.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

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

void simplifyRoutePolyline(RoutePolyline& route, std::size_t maxPoints) {
  if (maxPoints < 2 || route.points.size() <= maxPoints) {
    return;
  }
  const std::size_t n = route.points.size();
  std::vector<LatLon> out;
  out.reserve(maxPoints);
  for (std::size_t i = 0; i < maxPoints; ++i) {
    const std::size_t idx = i * (n - 1) / (maxPoints - 1);
    out.push_back(route.points[idx]);
  }
  route.points = std::move(out);
}

RouteToGoal computeRouteToGoal(const MultimodalGraph& graph, const GraphPosition& start,
                               const GraphPosition& goal, double speedMs, VehicleType type,
                               const PredictParam& param, double maxTime) {
  RouteToGoal result;
  if (!start.valid || !goal.valid || maxTime <= 0.0) {
    return result;
  }
  RoutePolyline& route = result.polyline;

  const LatLon goalLoc = positionLatLon(graph, goal);
  const double maxSpeedMs = std::max(speedMs, 5.0);

  const Edge* goalEdge = goal.edgeId != 0 ? graph.findEdge(goal.edgeId) : nullptr;
  const int64_t goalNodeOnly = goal.edgeId == 0 ? goal.nodeId : 0;

  using QueueItem = std::tuple<double, double, int64_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;
  std::unordered_map<int64_t, double> dist;
  std::unordered_map<int64_t, int64_t> parent;
  dist.reserve(8192);
  parent.reserve(8192);

  auto heuristic = [&](int64_t nodeId) -> double {
    const Node* node = graph.findNode(nodeId);
    if (node == nullptr) {
      return 0.0;
    }
    return haversineMeters({node->lat, node->lon}, goalLoc) / maxSpeedMs;
  };

  auto relax = [&](int64_t nodeId, double t, int64_t fromNode) {
    if (t > maxTime + 1e-9) {
      return;
    }
    const auto it = dist.find(nodeId);
    if (it != dist.end() && it->second <= t + 1e-9) {
      return;
    }
    dist[nodeId] = t;
    if (fromNode != 0) {
      parent[nodeId] = fromNode;
    }
    pq.push({t + heuristic(nodeId), t, nodeId});
  };

  std::unordered_set<int64_t> seedNodes;
  if (start.edgeId == 0) {
    relax(start.nodeId, 0.0, 0);
    seedNodes.insert(start.nodeId);
  } else {
    const Edge* edge = graph.findEdge(start.edgeId);
    if (edge == nullptr) {
      return result;
    }
    const double toFrom = travelTimeSeconds(start.alongMeters, speedMs, type, param);
    const double toTo =
        travelTimeSeconds(std::max(0.0, edge->length - start.alongMeters), speedMs, type, param);
    relax(edge->from, toFrom, 0);
    relax(edge->to, toTo, 0);
    seedNodes.insert(edge->from);
    seedNodes.insert(edge->to);
  }

  const std::size_t visitCap =
      param.maxVisitedNodes > 0 ? param.maxVisitedNodes : std::numeric_limits<std::size_t>::max();

  double bestGoalTime = kInfTime;
  int64_t bestGoalNode = 0;

  auto updateGoalFromNode = [&](int64_t nodeId, double nodeT) {
    if (goalNodeOnly != 0 && nodeId == goalNodeOnly && nodeT < bestGoalTime) {
      bestGoalTime = nodeT;
      bestGoalNode = nodeId;
      return;
    }
    if (goalEdge == nullptr) {
      return;
    }
    if (nodeId == goalEdge->from) {
      const double tGoal =
          nodeT + travelTimeSeconds(goal.alongMeters, speedMs, type, param);
      if (tGoal < bestGoalTime) {
        bestGoalTime = tGoal;
        bestGoalNode = goalEdge->from;
      }
    }
    if (nodeId == goalEdge->to) {
      const double tGoal =
          nodeT + travelTimeSeconds(std::max(0.0, goalEdge->length - goal.alongMeters), speedMs,
                                    type, param);
      if (tGoal < bestGoalTime) {
        bestGoalTime = tGoal;
        bestGoalNode = goalEdge->to;
      }
    }
  };

  while (!pq.empty()) {
    if (dist.size() >= visitCap) {
      break;
    }
    const auto [f, t, u] = pq.top();
    if (bestGoalTime < kInfTime / 2.0 && f >= bestGoalTime - 1e-6) {
      break;
    }
    pq.pop();
    const auto it = dist.find(u);
    if (it == dist.end() || t > it->second + 1e-6) {
      continue;
    }

    updateGoalFromNode(u, t);
    if (goalNodeOnly != 0 && u == goalNodeOnly && t <= bestGoalTime + 1e-6) {
      break;
    }

    for (const AdjacencyEdge& adj : graph.neighbors(u)) {
      if (!edgeAllowedForVehicle(adj.type, type)) {
        continue;
      }
      const double eff = edgeEffectiveSpeedMs(adj, type, speedMs);
      const double w = travelTimeSeconds(adj.length, eff, type, param);
      relax(adj.to, t + w, u);
    }
  }

  if (bestGoalNode == 0 || bestGoalTime >= kInfTime / 2.0) {
    return result;
  }
  result.travelTimeSec = bestGoalTime;

  std::vector<int64_t> nodes;
  int64_t cur = bestGoalNode;
  while (cur != 0) {
    nodes.push_back(cur);
    const auto pit = parent.find(cur);
    if (pit == parent.end() || seedNodes.count(cur) != 0) {
      break;
    }
    cur = pit->second;
  }
  std::reverse(nodes.begin(), nodes.end());

  if (start.edgeId != 0) {
    route.points.push_back(positionLatLon(graph, start));
  }
  for (int64_t nodeId : nodes) {
    const Node* node = graph.findNode(nodeId);
    if (node != nullptr) {
      route.points.push_back({node->lat, node->lon});
    }
  }
  const LatLon endLoc = positionLatLon(graph, goal);
  if (route.points.empty() ||
      haversineMeters(route.points.back(), endLoc) > 1.0) {
    route.points.push_back(endLoc);
  }
  return result;
}

RoutePolyline computeRoutePolyline(const MultimodalGraph& graph, const GraphPosition& start,
                                   const GraphPosition& goal, double speedMs, VehicleType type,
                                   const PredictParam& param, double maxTime) {
  return computeRouteToGoal(graph, start, goal, speedMs, type, param, maxTime).polyline;
}

RoutedTimeField computeRoutedTimeFieldFromGoal(const MultimodalGraph& graph,
                                               const GraphPosition& goal, double speedMs,
                                               VehicleType type, const PredictParam& param,
                                               double maxTime,
                                               const std::unordered_set<int64_t>* targetNodes,
                                               const LatLon* goalLatLon, double maxRadiusM) {
  RoutedTimeField field;
  if (!goal.valid || maxTime <= 0.0) {
    return field;
  }

  const Edge* goalEdge = goal.edgeId != 0 ? graph.findEdge(goal.edgeId) : nullptr;

  using QueueItem = std::tuple<double, int64_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

  std::unordered_set<int64_t> pendingTargets;
  if (targetNodes != nullptr) {
    pendingTargets = *targetNodes;
  }

  auto relax = [&](int64_t nodeId, double t, int64_t parentNode) {
    if (t > maxTime + 1e-9) {
      return;
    }
    const auto it = field.atNode.find(nodeId);
    if (it != field.atNode.end() && it->second <= t + 1e-9) {
      return;
    }
    field.atNode[nodeId] = t;
    if (parentNode != 0) {
      field.parentTowardGoal[nodeId] = parentNode;
    }
    pq.push({t, nodeId});
  };

  if (goal.edgeId == 0) {
    relax(goal.nodeId, 0.0, 0);
  } else if (goalEdge != nullptr) {
    relax(goalEdge->from, 0.0, 0);
    relax(goalEdge->to, 0.0, 0);
  }

  const std::size_t visitCap =
      param.maxVisitedNodes > 0 ? param.maxVisitedNodes : std::numeric_limits<std::size_t>::max();

  while (!pq.empty()) {
    if (field.atNode.size() >= visitCap) {
      break;
    }
    if (targetNodes != nullptr && pendingTargets.empty()) {
      break;
    }
    const auto [t, u] = pq.top();
    pq.pop();
    const auto it = field.atNode.find(u);
    if (it == field.atNode.end() || t > it->second + 1e-6) {
      continue;
    }
    if (pendingTargets.count(u) > 0) {
      pendingTargets.erase(u);
    }

    for (const AdjacencyEdge& adj : graph.neighbors(u)) {
      if (!edgeAllowedForVehicle(adj.type, type)) {
        continue;
      }
      if (goalLatLon != nullptr && maxRadiusM > 0.0) {
        const Node* node = graph.findNode(adj.to);
        if (node != nullptr &&
            haversineMeters(*goalLatLon, {node->lat, node->lon}) > maxRadiusM) {
          continue;
        }
      }
      const double eff = edgeEffectiveSpeedMs(adj, type, speedMs);
      const double w = travelTimeSeconds(adj.length, eff, type, param);
      relax(adj.to, t + w, u);
    }
  }

  return field;
}

RouteToGoal routeFromRoutedField(const MultimodalGraph& graph, const RoutedTimeField& field,
                                 const GraphPosition& start, const GraphPosition& goal,
                                 double speedMs, VehicleType type, const PredictParam& param) {
  RouteToGoal result;
  if (!start.valid || !goal.valid) {
    return result;
  }

  auto nodeTime = [&](int64_t nodeId) -> double {
    const auto it = field.atNode.find(nodeId);
    if (it == field.atNode.end()) {
      return kInfTime;
    }
    return it->second;
  };

  double travel = kInfTime;
  if (start.edgeId == 0) {
    travel = nodeTime(start.nodeId);
  } else {
    const Edge* edge = graph.findEdge(start.edgeId);
    if (edge != nullptr) {
      const double toFrom =
          nodeTime(edge->from) +
          travelTimeSeconds(start.alongMeters, speedMs, type, param);
      const double toTo =
          nodeTime(edge->to) +
          travelTimeSeconds(std::max(0.0, edge->length - start.alongMeters), speedMs, type,
                            param);
      travel = std::min(toFrom, toTo);
    }
  }

  if (travel >= kInfTime / 2.0) {
    return result;
  }
  result.travelTimeSec = travel;

  int64_t startNode = 0;
  if (start.edgeId == 0) {
    startNode = start.nodeId;
  } else {
    const Edge* edge = graph.findEdge(start.edgeId);
    if (edge == nullptr) {
      return result;
    }
    const double toFrom =
        nodeTime(edge->from) +
        travelTimeSeconds(start.alongMeters, speedMs, type, param);
    const double toTo =
        nodeTime(edge->to) +
        travelTimeSeconds(std::max(0.0, edge->length - start.alongMeters), speedMs, type, param);
    startNode = (toFrom <= toTo) ? edge->from : edge->to;
  }

  std::vector<int64_t> nodes;
  int64_t cur = startNode;
  while (cur != 0) {
    nodes.push_back(cur);
    const auto pit = field.parentTowardGoal.find(cur);
    if (pit == field.parentTowardGoal.end()) {
      break;
    }
    cur = pit->second;
  }

  RoutePolyline& route = result.polyline;
  if (start.edgeId != 0) {
    route.points.push_back(positionLatLon(graph, start));
  }
  for (int64_t nodeId : nodes) {
    const Node* node = graph.findNode(nodeId);
    if (node != nullptr) {
      route.points.push_back({node->lat, node->lon});
    }
  }
  const LatLon endLoc = positionLatLon(graph, goal);
  if (route.points.empty() || haversineMeters(route.points.back(), endLoc) > 1.0) {
    route.points.push_back(endLoc);
  }
  return result;
}

LatLon positionLatLonStore(const GraphFileStore& store, const GraphPosition& pos) {
  if (!pos.valid) {
    return {};
  }
  if (pos.edgeId == 0) {
    double lat = 0.0;
    double lon = 0.0;
    if (!store.nodeLatLon(pos.nodeId, lat, lon)) {
      return {};
    }
    return {lat, lon};
  }
  double flat = 0.0;
  double flon = 0.0;
  double tlat = 0.0;
  double tlon = 0.0;
  EdgeType type = EdgeType::ROAD;
  double length = 0.0;
  double speedLimit = 0.0;
  if (!store.edgeEndpointLatLon(pos.edgeId, flat, flon, tlat, tlon) ||
      !store.readEdge(pos.edgeId, type, length, speedLimit) || length <= 1e-6) {
    return {flat, flon};
  }
  const double t = std::max(0.0, std::min(1.0, pos.alongMeters / length));
  return {flat + t * (tlat - flat), flon + t * (tlon - flon)};
}

RoutedTimeField computeRoutedTimeFieldFromGoalCsr(
    const GraphFileStore& store, const CsrGraph& csr, const GraphPosition& goal, double speedMs,
    VehicleType type, const PredictParam& param, double maxTime,
    const std::unordered_set<int64_t>* allowedEdgeIds,
    const std::unordered_set<int64_t>* targetNodes, const LatLon* goalLatLon,
    double maxRadiusM) {
  RoutedTimeField field;
  if (!goal.valid || maxTime <= 0.0 || !csr.isOpen()) {
    return field;
  }

  using QueueItem = std::tuple<double, int64_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

  std::unordered_set<int64_t> pendingTargets;
  if (targetNodes != nullptr) {
    pendingTargets = *targetNodes;
  }

  auto relax = [&](int64_t nodeId, double t, int64_t parentNode) {
    if (t > maxTime + 1e-9) {
      return;
    }
    const auto it = field.atNode.find(nodeId);
    if (it != field.atNode.end() && it->second <= t + 1e-9) {
      return;
    }
    field.atNode[nodeId] = t;
    if (parentNode != 0) {
      field.parentTowardGoal[nodeId] = parentNode;
    }
    pq.push({t, nodeId});
  };

  if (goal.edgeId == 0) {
    relax(goal.nodeId, 0.0, 0);
  } else {
    int64_t from = 0;
    int64_t to = 0;
    if (store.edgeEndpoints(goal.edgeId, from, to)) {
      relax(from, 0.0, 0);
      relax(to, 0.0, 0);
    }
  }

  const std::size_t visitCap =
      param.maxVisitedNodes > 0 ? param.maxVisitedNodes : std::numeric_limits<std::size_t>::max();

  while (!pq.empty()) {
    if (field.atNode.size() >= visitCap) {
      break;
    }
    if (targetNodes != nullptr && pendingTargets.empty()) {
      break;
    }
    const auto [t, u] = pq.top();
    pq.pop();
    const auto it = field.atNode.find(u);
    if (it == field.atNode.end() || t > it->second + 1e-6) {
      continue;
    }
    if (pendingTargets.count(u) > 0) {
      pendingTargets.erase(u);
    }

    csr.forEachNeighbor(store, u, allowedEdgeIds, [&](const CsrArc& adj) {
      if (!edgeAllowedForVehicle(adj.type, type)) {
        return;
      }
      if (goalLatLon != nullptr && maxRadiusM > 0.0) {
        double lat = 0.0;
        double lon = 0.0;
        if (!store.nodeLatLon(adj.toNodeId, lat, lon)) {
          return;
        }
        if (haversineMeters(*goalLatLon, {lat, lon}) > maxRadiusM) {
          return;
        }
      }
      const double eff = csrArcEffectiveSpeedMs(adj.speedLimit, type, speedMs);
      const double w = travelTimeSeconds(adj.length, eff, type, param);
      relax(adj.toNodeId, t + w, u);
    });
  }

  return field;
}

std::vector<double> computeRoutedDistFromGoalCsrDense(
    const GraphFileStore& store, const CsrGraph& csr, int64_t goalNodeId, double speedMs,
    VehicleType type, const PredictParam& param, double maxTime,
    const std::unordered_set<int64_t>* targetNodes, const LatLon* goalLatLon,
    double maxRadiusM, std::vector<int64_t>* parentNodeByRow,
    std::vector<int64_t>* parentEdgeByRow) {
  const std::size_t n = csr.nodeCount();
  std::vector<double> dist(n, kInfTime);
  if (goalNodeId == 0 || maxTime <= 0.0 || !csr.isOpen() || n == 0) {
    return dist;
  }
  const int goalRow = csr.nodeRow(store, goalNodeId);
  if (goalRow < 0) {
    return dist;
  }
  if (parentNodeByRow != nullptr) {
    parentNodeByRow->assign(n, 0);
  }
  if (parentEdgeByRow != nullptr) {
    parentEdgeByRow->assign(n, 0);
  }

  using QueueItem = std::tuple<double, int64_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

  std::unordered_set<int64_t> pendingTargets;
  if (targetNodes != nullptr) {
    pendingTargets = *targetNodes;
  }

  const std::size_t visitCap =
      param.maxVisitedNodes > 0 ? param.maxVisitedNodes : std::numeric_limits<std::size_t>::max();
  std::size_t visited = 0;

  auto relax = [&](int64_t nodeId, double t, int64_t parentNode, int64_t parentEdge) {
    if (t > maxTime + 1e-9) {
      return;
    }
    const int row = csr.nodeRow(store, nodeId);
    if (row < 0) {
      return;
    }
    if (t >= dist[static_cast<std::size_t>(row)] - 1e-9) {
      return;
    }
    const bool firstReach = dist[static_cast<std::size_t>(row)] >= kInfTime / 2.0;
    dist[static_cast<std::size_t>(row)] = t;
    if (parentNodeByRow != nullptr && parentNode != 0) {
      (*parentNodeByRow)[static_cast<std::size_t>(row)] = parentNode;
    }
    if (parentEdgeByRow != nullptr && parentEdge != 0) {
      (*parentEdgeByRow)[static_cast<std::size_t>(row)] = parentEdge;
    }
    if (firstReach) {
      ++visited;
    }
    pq.push({t, nodeId});
  };

  relax(goalNodeId, 0.0, 0, 0);

  while (!pq.empty()) {
    if (visited >= visitCap) {
      break;
    }
    if (targetNodes != nullptr && pendingTargets.empty()) {
      break;
    }
    const auto [t, u] = pq.top();
    pq.pop();
    const int uRow = csr.nodeRow(store, u);
    if (uRow < 0 || t > dist[static_cast<std::size_t>(uRow)] + 1e-6) {
      continue;
    }
    if (pendingTargets.count(u) > 0) {
      pendingTargets.erase(u);
    }

    csr.forEachNeighbor(store, u, nullptr, [&](const CsrArc& adj) {
      if (!edgeAllowedForVehicle(adj.type, type)) {
        return;
      }
      if (goalLatLon != nullptr && maxRadiusM > 0.0) {
        double lat = 0.0;
        double lon = 0.0;
        if (!store.nodeLatLon(adj.toNodeId, lat, lon)) {
          return;
        }
        if (haversineMeters(*goalLatLon, {lat, lon}) > maxRadiusM) {
          return;
        }
      }
      const double eff = csrArcEffectiveSpeedMs(adj.speedLimit, type, speedMs);
      const double w = travelTimeSeconds(adj.length, eff, type, param);
      relax(adj.toNodeId, t + w, u, adj.edgeId);
    });
  }

  return dist;
}

std::vector<double> computeRoutedDistFromGoalCsrDense(
    const GraphFileStore& store, const CsrGraph& csr, const GraphPosition& goal, double speedMs,
    VehicleType type, const PredictParam& param, double maxTime,
    const std::unordered_set<int64_t>* targetNodes, const LatLon* goalLatLon,
    double maxRadiusM, std::vector<int64_t>* parentNodeByRow,
    std::vector<int64_t>* parentEdgeByRow, int64_t* goalNodeIdOut) {
  if (!goal.valid) {
    return {};
  }
  if (goal.edgeId == 0) {
    if (goalNodeIdOut != nullptr) {
      *goalNodeIdOut = goal.nodeId;
    }
    return computeRoutedDistFromGoalCsrDense(store, csr, goal.nodeId, speedMs, type, param, maxTime,
                                             targetNodes, goalLatLon, maxRadiusM, parentNodeByRow,
                                             parentEdgeByRow);
  }

  int64_t from = 0;
  int64_t to = 0;
  if (!store.edgeEndpoints(goal.edgeId, from, to)) {
    return {};
  }
  if (goalNodeIdOut != nullptr) {
    *goalNodeIdOut = from;
  }

  const std::size_t n = csr.nodeCount();
  std::vector<double> dist(n, kInfTime);
  if (maxTime <= 0.0 || !csr.isOpen() || n == 0) {
    return dist;
  }
  if (parentNodeByRow != nullptr) {
    parentNodeByRow->assign(n, 0);
  }
  if (parentEdgeByRow != nullptr) {
    parentEdgeByRow->assign(n, 0);
  }

  using QueueItem = std::tuple<double, int64_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

  std::unordered_set<int64_t> pendingTargets;
  if (targetNodes != nullptr) {
    pendingTargets = *targetNodes;
  }

  const std::size_t visitCap =
      param.maxVisitedNodes > 0 ? param.maxVisitedNodes : std::numeric_limits<std::size_t>::max();
  std::size_t visited = 0;

  auto relax = [&](int64_t nodeId, double t, int64_t parentNode, int64_t parentEdge) {
    if (t > maxTime + 1e-9) {
      return;
    }
    const int row = csr.nodeRow(store, nodeId);
    if (row < 0) {
      return;
    }
    if (t >= dist[static_cast<std::size_t>(row)] - 1e-9) {
      return;
    }
    const bool firstReach = dist[static_cast<std::size_t>(row)] >= kInfTime / 2.0;
    dist[static_cast<std::size_t>(row)] = t;
    if (parentNodeByRow != nullptr && parentNode != 0) {
      (*parentNodeByRow)[static_cast<std::size_t>(row)] = parentNode;
    }
    if (parentEdgeByRow != nullptr && parentEdge != 0) {
      (*parentEdgeByRow)[static_cast<std::size_t>(row)] = parentEdge;
    }
    if (firstReach) {
      ++visited;
    }
    pq.push({t, nodeId});
  };

  relax(from, 0.0, 0, 0);
  relax(to, 0.0, 0, 0);

  while (!pq.empty()) {
    if (visited >= visitCap) {
      break;
    }
    if (targetNodes != nullptr && pendingTargets.empty()) {
      break;
    }
    const auto [t, u] = pq.top();
    pq.pop();
    const int uRow = csr.nodeRow(store, u);
    if (uRow < 0 || t > dist[static_cast<std::size_t>(uRow)] + 1e-6) {
      continue;
    }
    if (pendingTargets.count(u) > 0) {
      pendingTargets.erase(u);
    }

    csr.forEachNeighbor(store, u, nullptr, [&](const CsrArc& adj) {
      if (!edgeAllowedForVehicle(adj.type, type)) {
        return;
      }
      if (goalLatLon != nullptr && maxRadiusM > 0.0) {
        double lat = 0.0;
        double lon = 0.0;
        if (!store.nodeLatLon(adj.toNodeId, lat, lon)) {
          return;
        }
        if (haversineMeters(*goalLatLon, {lat, lon}) > maxRadiusM) {
          return;
        }
      }
      const double eff = csrArcEffectiveSpeedMs(adj.speedLimit, type, speedMs);
      const double w = travelTimeSeconds(adj.length, eff, type, param);
      relax(adj.toNodeId, t + w, u, adj.edgeId);
    });
  }

  return dist;
}

RoutePolyline polylineFromGoalCsrParents(const GraphFileStore& store, const CsrGraph& csr,
                                         const std::vector<int64_t>& parentNodeByRow,
                                         const std::vector<int64_t>& parentEdgeByRow,
                                         int64_t startNodeId, int64_t goalNodeId) {
  RoutePolyline route;
  if (startNodeId == 0 || goalNodeId == 0 || parentNodeByRow.empty()) {
    return route;
  }

  auto appendPt = [&](const LatLon& p) {
    if (route.points.empty() || haversineMeters(route.points.back(), p) > 1.0) {
      route.points.push_back(p);
    }
  };

  int64_t cur = startNodeId;
  double clat = 0.0;
  double clon = 0.0;
  if (store.nodeLatLon(cur, clat, clon)) {
    appendPt({clat, clon});
  }

  while (cur != 0 && cur != goalNodeId) {
    const int row = csr.nodeRow(store, cur);
    if (row < 0 || static_cast<std::size_t>(row) >= parentNodeByRow.size()) {
      break;
    }
    const int64_t parent = parentNodeByRow[static_cast<std::size_t>(row)];
    if (parent == 0) {
      break;
    }
    const int64_t edgeId =
        static_cast<std::size_t>(row) < parentEdgeByRow.size()
            ? parentEdgeByRow[static_cast<std::size_t>(row)]
            : 0;
    if (edgeId != 0) {
      double flat = 0.0;
      double flon = 0.0;
      double tlat = 0.0;
      double tlon = 0.0;
      if (store.edgeEndpointLatLon(edgeId, flat, flon, tlat, tlon)) {
        double plat = 0.0;
        double plon = 0.0;
        if (store.nodeLatLon(parent, plat, plon)) {
          const double toFlat = haversineMeters({clat, clon}, {flat, flon});
          const double toTlat = haversineMeters({clat, clon}, {tlat, tlon});
          const double pToFlat = haversineMeters({plat, plon}, {flat, flon});
          const double pToTlat = haversineMeters({plat, plon}, {tlat, tlon});
          if (toFlat + pToTlat <= toTlat + pToFlat) {
            appendPt({flat, flon});
            appendPt({tlat, tlon});
          } else {
            appendPt({tlat, tlon});
            appendPt({flat, flon});
          }
        } else {
          appendPt({flat, flon});
          appendPt({tlat, tlon});
        }
      }
    } else if (store.nodeLatLon(parent, clat, clon)) {
      appendPt({clat, clon});
    }
    cur = parent;
    if (!store.nodeLatLon(cur, clat, clon)) {
      break;
    }
  }

  if (cur == goalNodeId) {
    double glat = 0.0;
    double glon = 0.0;
    if (store.nodeLatLon(goalNodeId, glat, glon)) {
      appendPt({glat, glon});
    }
  }
  return route;
}

RouteToGoal routeFromRoutedFieldCsr(const GraphFileStore& store, const CsrGraph& csr,
                                    const RoutedTimeField& field, const GraphPosition& start,
                                    const GraphPosition& goal, double speedMs, VehicleType type,
                                    const PredictParam& param) {
  RouteToGoal result;
  if (!start.valid || !goal.valid) {
    return result;
  }

  const double travel = travelTimeFromRoutedFieldCsr(store, field, start, speedMs, type, param);
  if (travel >= kInfTime / 2.0) {
    return result;
  }
  result.travelTimeSec = travel;

  auto nodeTime = [&](int64_t nodeId) -> double {
    const auto it = field.atNode.find(nodeId);
    if (it == field.atNode.end()) {
      return kInfTime;
    }
    return it->second;
  };

  int64_t startNode = 0;
  if (start.edgeId == 0) {
    startNode = start.nodeId;
  } else {
    int64_t from = 0;
    int64_t to = 0;
    EdgeType edgeType = EdgeType::ROAD;
    double length = 0.0;
    double speedLimit = 0.0;
    if (!store.edgeEndpoints(start.edgeId, from, to) ||
        !store.readEdge(start.edgeId, edgeType, length, speedLimit)) {
      return result;
    }
    const double toFrom =
        nodeTime(from) + travelTimeSeconds(start.alongMeters, speedMs, type, param);
    const double toTo =
        nodeTime(to) +
        travelTimeSeconds(std::max(0.0, length - start.alongMeters), speedMs, type, param);
    startNode = (toFrom <= toTo) ? from : to;
  }

  std::vector<int64_t> nodes;
  int64_t cur = startNode;
  while (cur != 0) {
    nodes.push_back(cur);
    const auto pit = field.parentTowardGoal.find(cur);
    if (pit == field.parentTowardGoal.end()) {
      break;
    }
    cur = pit->second;
  }

  RoutePolyline& route = result.polyline;
  if (start.edgeId != 0) {
    route.points.push_back(positionLatLonStore(store, start));
  }
  for (int64_t nodeId : nodes) {
    double lat = 0.0;
    double lon = 0.0;
    if (store.nodeLatLon(nodeId, lat, lon)) {
      route.points.push_back({lat, lon});
    }
  }
  const LatLon endLoc = positionLatLonStore(store, goal);
  if (route.points.empty() || haversineMeters(route.points.back(), endLoc) > 1.0) {
    route.points.push_back(endLoc);
  }
  (void)csr;
  return result;
}

double travelTimeFromRoutedFieldCsr(const GraphFileStore& store, const RoutedTimeField& field,
                                    const GraphPosition& start, double speedMs, VehicleType type,
                                    const PredictParam& param) {
  if (!start.valid) {
    return kInfTime;
  }

  auto nodeTime = [&](int64_t nodeId) -> double {
    const auto it = field.atNode.find(nodeId);
    if (it == field.atNode.end()) {
      return kInfTime;
    }
    return it->second;
  };

  if (start.edgeId == 0) {
    return nodeTime(start.nodeId);
  }

  int64_t from = 0;
  int64_t to = 0;
  EdgeType edgeType = EdgeType::ROAD;
  double length = 0.0;
  double speedLimit = 0.0;
  if (!store.edgeEndpoints(start.edgeId, from, to) ||
      !store.readEdge(start.edgeId, edgeType, length, speedLimit)) {
    return kInfTime;
  }
  const double toFrom =
      nodeTime(from) + travelTimeSeconds(start.alongMeters, speedMs, type, param);
  const double toTo =
      nodeTime(to) +
      travelTimeSeconds(std::max(0.0, length - start.alongMeters), speedMs, type, param);
  return std::min(toFrom, toTo);
}

}  // namespace mmlp
