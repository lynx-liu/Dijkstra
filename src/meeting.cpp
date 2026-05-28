#include "mmlp/meeting.hpp"

#include "mmlp/geo.hpp"
#include "mmlp/matching.hpp"
#include "mmlp/motion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <queue>
#include <sstream>
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

GraphPosition makeEdgePosition(const Edge& edge, double alongMeters) {
  GraphPosition pos;
  pos.valid = true;
  pos.edgeId = edge.id;
  pos.nodeId = edge.from;
  pos.alongMeters = std::max(0.0, std::min(edge.length, alongMeters));
  return pos;
}

void considerCandidate(double meetDuration, const GraphPosition& where,
                       const MultimodalGraph& graph, double& bestDuration,
                       GraphPosition& bestWhere) {
  if (meetDuration > bestDuration || meetDuration < 0.0) {
    return;
  }
  bestDuration = meetDuration;
  bestWhere = where;
}

using DistMap = std::unordered_map<int64_t, double>;
using TimePQ = std::priority_queue<std::tuple<double, double, int64_t>,
                                   std::vector<std::tuple<double, double, int64_t>>,
                                   std::greater<std::tuple<double, double, int64_t>>>;

double pairRoutingHorizonSec(const PreparedVehicle& a, const PreparedVehicle& b,
                             const PredictParam& param) {
  const double dist =
      haversineMeters({a.info.lat, a.info.lon}, {b.info.lat, b.info.lon});
  const double vmin = std::max(5.0, std::min(a.speedMs, b.speedMs));
  const double byDist = dist / vmin * 1.35 + 240.0;
  return std::min(param.maxTime, std::max(480.0, byDist));
}

void considerMeetAtNode(int64_t nodeId, const DistMap& distA, const DistMap& distB,
                        double& bestDuration, GraphPosition& bestWhere) {
  const auto itA = distA.find(nodeId);
  const auto itB = distB.find(nodeId);
  if (itA == distA.end() || itB == distB.end()) {
    return;
  }
  GraphPosition onNode;
  onNode.valid = true;
  onNode.nodeId = nodeId;
  if (std::max(itA->second, itB->second) < bestDuration) {
    bestDuration = std::max(itA->second, itB->second);
    bestWhere = onNode;
  }
}

void seedTimeSources(const MultimodalGraph& graph, const GraphPosition& start, double speedMs,
                     VehicleType type, const PredictParam& param, double maxTime, DistMap& dist,
                     TimePQ& pq, const LatLon* goal = nullptr) {
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
  auto relax = [&](int64_t nodeId, double t) {
    if (t > maxTime + 1e-9) {
      return;
    }
    const auto it = dist.find(nodeId);
    if (it != dist.end() && it->second <= t + 1e-9) {
      return;
    }
    dist[nodeId] = t;
    pq.push({t + heuristic(nodeId), t, nodeId});
  };

  if (start.edgeId == 0) {
    relax(start.nodeId, 0.0);
    return;
  }
  const Edge* edge = graph.findEdge(start.edgeId);
  if (edge == nullptr) {
    return;
  }
  relax(edge->from, travelTimeSeconds(start.alongMeters, speedMs, type, param));
  relax(edge->to,
        travelTimeSeconds(std::max(0.0, edge->length - start.alongMeters), speedMs, type, param));
}

bool expandSide(const MultimodalGraph& graph, TimePQ& pq, DistMap& dist, const DistMap& other,
                double speedMs, VehicleType type, const PredictParam& param, double maxTime,
                std::size_t visitCap, const LatLon* goal, double& bestDuration,
                GraphPosition& bestWhere) {
  if (pq.empty()) {
    return false;
  }
  if (dist.size() >= visitCap) {
    return false;
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

  const auto [f, t, u] = pq.top();
  pq.pop();
  (void)f;
  const auto it = dist.find(u);
  if (it == dist.end() || t > it->second + 1e-6) {
    return true;
  }

  considerMeetAtNode(u, dist, other, bestDuration, bestWhere);

  for (const AdjacencyEdge& adj : graph.neighbors(u)) {
    const Edge* edge = graph.findEdge(adj.edgeId);
    if (edge == nullptr || !edgeAllowedForVehicle(edge->type, type)) {
      continue;
    }
    const double eff = edgeEffectiveSpeedMs(adj, type, speedMs);
    const double w = travelTimeSeconds(adj.length, eff, type, param);
    const double nt = t + w;
    if (nt > maxTime + 1e-9) {
      continue;
    }
    const auto nit = dist.find(adj.to);
    if (nit != dist.end() && nit->second <= nt + 1e-9) {
      continue;
    }
    dist[adj.to] = nt;
    pq.push({nt + heuristic(adj.to), nt, adj.to});
    considerMeetAtNode(adj.to, dist, other, bestDuration, bestWhere);
  }
  return true;
}

void scanMutualReachEdges(const MultimodalGraph& graph, const TimeField& fieldA,
                          const TimeField& fieldB, const PreparedVehicle& a,
                          const PreparedVehicle& b, int64_t alignTime,
                          const PredictParam& param,
                          const std::unordered_set<int64_t>& mutualReach,
                          double& bestDuration, GraphPosition& bestWhere) {
  std::unordered_set<int64_t> seenEdges;
  seenEdges.reserve(std::min<std::size_t>(mutualReach.size() * 4, 200000));

  const int samples = mutualReach.size() > 2000 ? 8 : 24;

  auto considerNode = [&](int64_t nodeId) {
    GraphPosition onNode;
    onNode.valid = true;
    onNode.nodeId = nodeId;
    const double tA = timeAtPosition(graph, fieldA, onNode, a.speedMs, a.info.type, param);
    const double tB = timeAtPosition(graph, fieldB, onNode, b.speedMs, b.info.type, param);
    if (tA >= kInfTime / 2.0 || tB >= kInfTime / 2.0) {
      return;
    }
    considerCandidate(std::max(tA, tB), onNode, graph, bestDuration, bestWhere);
  };

  for (int64_t nodeId : mutualReach) {
    considerNode(nodeId);
  }

  for (int64_t nodeId : mutualReach) {
    for (const AdjacencyEdge& adj : graph.neighbors(nodeId)) {
      if (!seenEdges.insert(adj.edgeId).second) {
        continue;
      }
      const Edge* edge = graph.findEdge(adj.edgeId);
      if (edge == nullptr || !edgeAllowedForVehicle(edge->type, a.info.type)) {
        continue;
      }
      if (mutualReach.count(edge->from) == 0 || mutualReach.count(edge->to) == 0) {
        continue;
      }

      for (int i = 0; i <= samples; ++i) {
        const double along = edge->length * (static_cast<double>(i) / samples);
        const GraphPosition onEdge = makeEdgePosition(*edge, along);
        const double tA =
            timeAtPosition(graph, fieldA, onEdge, a.speedMs, a.info.type, param);
        const double tB =
            timeAtPosition(graph, fieldB, onEdge, b.speedMs, b.info.type, param);
        if (tA >= kInfTime / 2.0 || tB >= kInfTime / 2.0) {
          continue;
        }
        considerCandidate(std::max(tA, tB), onEdge, graph, bestDuration, bestWhere);

        if (i > 0) {
          const double prevAlong = edge->length * (static_cast<double>(i - 1) / samples);
          const GraphPosition prevPos = makeEdgePosition(*edge, prevAlong);
          const double prevA =
              timeAtPosition(graph, fieldA, prevPos, a.speedMs, a.info.type, param);
          const double prevB =
              timeAtPosition(graph, fieldB, prevPos, b.speedMs, b.info.type, param);
          const double fPrev = prevA - prevB;
          const double fCur = tA - tB;
          if (fPrev * fCur < 0.0) {
            double lo = prevAlong;
            double hi = along;
            for (int k = 0; k < 24; ++k) {
              const double mid = 0.5 * (lo + hi);
              const GraphPosition midPos = makeEdgePosition(*edge, mid);
              const double midA =
                  timeAtPosition(graph, fieldA, midPos, a.speedMs, a.info.type, param);
              const double midB =
                  timeAtPosition(graph, fieldB, midPos, b.speedMs, b.info.type, param);
              if (midA - midB < 0.0) {
                lo = mid;
              } else {
                hi = mid;
              }
            }
            const GraphPosition meetPos = makeEdgePosition(*edge, 0.5 * (lo + hi));
            const double midA =
                timeAtPosition(graph, fieldA, meetPos, a.speedMs, a.info.type, param);
            const double midB =
                timeAtPosition(graph, fieldB, meetPos, b.speedMs, b.info.type, param);
            considerCandidate(std::max(midA, midB), meetPos, graph, bestDuration, bestWhere);
          }
        }
      }
    }
  }
}

double nodeTimeFromDist(const DistMap& dist, int64_t nodeId) {
  const auto it = dist.find(nodeId);
  if (it == dist.end()) {
    return kInfTime;
  }
  return it->second;
}

double timeOnEdgeFromDist(const MultimodalGraph& graph, const DistMap& dist, const Edge& edge,
                          double alongMeters, double speedMs, VehicleType type,
                          const PredictParam& param) {
  const double fromSide =
      nodeTimeFromDist(dist, edge.from) +
      travelTimeSeconds(alongMeters, speedMs, type, param);
  const double toSide =
      nodeTimeFromDist(dist, edge.to) +
      travelTimeSeconds(std::max(0.0, edge.length - alongMeters), speedMs, type, param);
  return std::min(fromSide, toSide);
}

void scanEdgeMeetings(const MultimodalGraph& graph, const DistMap& distA, const DistMap& distB,
                      const PreparedVehicle& a, const PreparedVehicle& b,
                      const PredictParam& param, double& bestDuration, GraphPosition& bestWhere) {
  std::unordered_set<int64_t> seen;
  seen.reserve(distA.size() * 2);
  for (const auto& kv : distA) {
    for (const AdjacencyEdge& adj : graph.neighbors(kv.first)) {
      if (!seen.insert(adj.edgeId).second) {
        continue;
      }
      const Edge* edge = graph.findEdge(adj.edgeId);
      if (edge == nullptr || !edgeAllowedForVehicle(edge->type, a.info.type)) {
        continue;
      }
      if (nodeTimeFromDist(distA, edge->from) >= kInfTime / 2.0 ||
          nodeTimeFromDist(distA, edge->to) >= kInfTime / 2.0 ||
          nodeTimeFromDist(distB, edge->from) >= kInfTime / 2.0 ||
          nodeTimeFromDist(distB, edge->to) >= kInfTime / 2.0) {
        continue;
      }

      const int samples = edge->length > 5000.0 ? 4 : 8;
      double prevAlong = 0.0;
      double prevDiff = 0.0;
      for (int i = 0; i <= samples; ++i) {
        const double along = edge->length * (static_cast<double>(i) / samples);
        const GraphPosition onEdge = makeEdgePosition(*edge, along);
        const double tA =
            timeOnEdgeFromDist(graph, distA, *edge, along, a.speedMs, a.info.type, param);
        const double tB =
            timeOnEdgeFromDist(graph, distB, *edge, along, b.speedMs, b.info.type, param);
        if (tA >= kInfTime / 2.0 || tB >= kInfTime / 2.0) {
          continue;
        }
        considerCandidate(std::max(tA, tB), onEdge, graph, bestDuration, bestWhere);
        if (i > 0) {
          const double fCur = tA - tB;
          if (prevDiff * fCur < 0.0) {
            double lo = prevAlong;
            double hi = along;
            for (int k = 0; k < 16; ++k) {
              const double mid = 0.5 * (lo + hi);
              const double midA =
                  timeOnEdgeFromDist(graph, distA, *edge, mid, a.speedMs, a.info.type, param);
              const double midB =
                  timeOnEdgeFromDist(graph, distB, *edge, mid, b.speedMs, b.info.type, param);
              if (midA - midB < 0.0) {
                lo = mid;
              } else {
                hi = mid;
              }
            }
            const GraphPosition meetPos = makeEdgePosition(*edge, 0.5 * (lo + hi));
            const double midA = timeOnEdgeFromDist(graph, distA, *edge, 0.5 * (lo + hi),
                                                   a.speedMs, a.info.type, param);
            const double midB = timeOnEdgeFromDist(graph, distB, *edge, 0.5 * (lo + hi),
                                                   b.speedMs, b.info.type, param);
            considerCandidate(std::max(midA, midB), meetPos, graph, bestDuration, bestWhere);
          }
          prevDiff = fCur;
        }
        prevAlong = along;
      }
    }
  }
}

bool bidirectionalMeeting(const MultimodalGraph& graph, const PreparedVehicle& a,
                          const PreparedVehicle& b, const PredictParam& param,
                          double routingHorizon, double& bestDuration, GraphPosition& bestWhere) {
  DistMap distA;
  DistMap distB;
  TimePQ pqA;
  TimePQ pqB;
  const double distKm =
      haversineMeters({a.info.lat, a.info.lon}, {b.info.lat, b.info.lon}) / 1000.0;
  const std::size_t visitCap =
      std::min<std::size_t>(200000, 25000 + static_cast<std::size_t>(distKm * 400.0));

  const LatLon goalB{b.info.lat, b.info.lon};
  const LatLon goalA{a.info.lat, a.info.lon};
  seedTimeSources(graph, a.position, a.speedMs, a.info.type, param, routingHorizon, distA, pqA,
                  &goalB);
  seedTimeSources(graph, b.position, b.speedMs, b.info.type, param, routingHorizon, distB, pqB,
                  &goalA);

  while (!pqA.empty() || !pqB.empty()) {
    if (bestDuration < kInfTime / 2.0) {
      const double boundA = pqA.empty() ? kInfTime : std::get<1>(pqA.top());
      const double boundB = pqB.empty() ? kInfTime : std::get<1>(pqB.top());
      if (boundA > bestDuration + 1e-6 && boundB > bestDuration + 1e-6) {
        break;
      }
    }

    const bool expandA =
        !pqA.empty() && (pqB.empty() || std::get<1>(pqA.top()) <= std::get<1>(pqB.top()));
    if (expandA) {
      if (!expandSide(graph, pqA, distA, distB, a.speedMs, a.info.type, param, routingHorizon,
                      visitCap, &goalB, bestDuration, bestWhere)) {
        break;
      }
    } else {
      if (!expandSide(graph, pqB, distB, distA, b.speedMs, b.info.type, param, routingHorizon,
                      visitCap, &goalA, bestDuration, bestWhere)) {
        break;
      }
    }
  }

  scanEdgeMeetings(graph, distA, distB, a, b, param, bestDuration, bestWhere);
  return bestWhere.valid && bestDuration <= param.maxTime;
}

bool meetingFromFields(const MultimodalGraph& graph, const PreparedVehicle& a,
                       const PreparedVehicle& b, const TimeField& fieldA,
                       const TimeField& fieldB, int64_t alignTime, const PredictParam& param,
                       MeetingResult& out) {
  double bestDuration = kInfTime;
  GraphPosition bestWhere;
  bestWhere.valid = false;

  std::unordered_set<int64_t> mutualReach;
  mutualReach.reserve(std::min(fieldA.atNode.size(), fieldB.atNode.size()));
  for (const auto& kv : fieldA.atNode) {
    if (fieldB.atNode.count(kv.first) > 0) {
      mutualReach.insert(kv.first);
    }
  }

  if (mutualReach.empty()) {
    return false;
  }

  scanMutualReachEdges(graph, fieldA, fieldB, a, b, alignTime, param, mutualReach, bestDuration,
                       bestWhere);

  if (!bestWhere.valid || bestDuration > param.maxTime) {
    return false;
  }

  const LatLon meetLoc = positionLatLon(graph, bestWhere);
  out.vehicleA = a.info.id;
  out.vehicleB = b.info.id;
  out.meetTime = static_cast<double>(alignTime) + bestDuration;
  out.lat = meetLoc.lat;
  out.lon = meetLoc.lon;
  out.distance = 0.0;

  if (bestWhere.edgeId == 0) {
    std::ostringstream oss;
    oss << bestWhere.nodeId;
    out.locationId = oss.str();
  } else {
    std::ostringstream oss;
    oss << "edge:" << bestWhere.edgeId;
    out.locationId = oss.str();
  }

  return true;
}

}  // namespace

PreparedVehicle prepareVehicle(const MultimodalGraph& graph, const VehicleInfo& vehicle,
                               const VehicleHistory* history, int64_t alignTime,
                               const PredictParam& param, const SpatialIndex* index) {
  PreparedVehicle prepared;
  prepared.info = vehicle;
  prepared.position = matchVehicleToGraph(graph, vehicle, index);
  prepared.valid = prepared.position.valid;
  if (!prepared.valid) {
    return prepared;
  }

  const Edge* hint =
      prepared.position.edgeId != 0 ? graph.findEdge(prepared.position.edgeId) : nullptr;
  const double speedKmh = resolveSpeedKmh(vehicle, history, hint, vehicle.type);
  prepared.speedMs = speedMsFromKmh(speedKmh);

  if (vehicle.timestamp < alignTime) {
    prepared.position =
        advancePosition(graph, prepared.position, prepared.speedMs, vehicle.type, param,
                        static_cast<double>(alignTime - vehicle.timestamp));
    prepared.valid = prepared.position.valid;
  }

  return prepared;
}

void fillMeetingResult(const MultimodalGraph& graph, const PreparedVehicle& a,
                     const PreparedVehicle& b, int64_t alignTime, double meetDuration,
                     const GraphPosition& where, MeetingResult& out) {
  const LatLon meetLoc = positionLatLon(graph, where);
  out.vehicleA = a.info.id;
  out.vehicleB = b.info.id;
  out.meetTime = static_cast<double>(alignTime) + meetDuration;
  out.lat = meetLoc.lat;
  out.lon = meetLoc.lon;
  out.distance = 0.0;
  if (where.edgeId == 0) {
    std::ostringstream oss;
    oss << where.nodeId;
    out.locationId = oss.str();
  } else {
    std::ostringstream oss;
    oss << "edge:" << where.edgeId;
    out.locationId = oss.str();
  }
}

bool computePairwiseMeetingFast(const MultimodalGraph& graph, const PreparedVehicle& a,
                                const PreparedVehicle& b, int64_t alignTime,
                                const PredictParam& param, MeetingResult& out) {
  if (!a.valid || !b.valid || a.info.type != b.info.type) {
    return false;
  }

  const double routingHorizon = pairRoutingHorizonSec(a, b, param);
  double bestDuration = kInfTime;
  GraphPosition bestWhere;
  bestWhere.valid = false;

  if (!bidirectionalMeeting(graph, a, b, param, routingHorizon, bestDuration, bestWhere)) {
    return false;
  }

  fillMeetingResult(graph, a, b, alignTime, bestDuration, bestWhere, out);
  return true;
}

bool computePairwiseMeeting(const MultimodalGraph& graph, const PreparedVehicle& a,
                            const PreparedVehicle& b, int64_t alignTime,
                            const PredictParam& param, MeetingResult& out) {
  if (!a.valid || !b.valid || a.info.type != b.info.type) {
    return false;
  }
  return computePairwiseMeetingFast(graph, a, b, alignTime, param, out);
}

bool computePairwiseMeeting(const MultimodalGraph& graph, const PreparedVehicle& a,
                            const PreparedVehicle& b, const TimeField& fieldA,
                            const TimeField& fieldB, int64_t alignTime,
                            const PredictParam& param, MeetingResult& out) {
  if (!a.valid || !b.valid || a.info.type != b.info.type) {
    return false;
  }
  return meetingFromFields(graph, a, b, fieldA, fieldB, alignTime, param, out);
}

}  // namespace mmlp
