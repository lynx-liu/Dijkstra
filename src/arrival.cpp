#include "mmlp/arrival.hpp"

#include "mmlp/ch_graph.hpp"
#include "mmlp/geo.hpp"
#include "mmlp/matching.hpp"
#include "mmlp/meeting.hpp"
#include "mmlp/motion.hpp"
#include "mmlp/region_loader.hpp"
#include "mmlp/routing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <map>
#include <queue>
#include <sstream>
#include <thread>
#include <unordered_set>
#include <vector>

namespace mmlp {

namespace {

const VehicleHistory* findHistory(const std::vector<VehicleHistory>& histories,
                                 const std::string& id) {
  for (const auto& h : histories) {
    if (h.id == id) {
      return &h;
    }
  }
  return nullptr;
}

double polylineLengthMeters(const RoutePolyline& route) {
  double sum = 0.0;
  for (std::size_t i = 1; i < route.points.size(); ++i) {
    sum += haversineMeters(route.points[i - 1], route.points[i]);
  }
  return sum;
}

bool vehicleMayReachDestination(const VehicleInfo& vehicle, const DestinationQuery& dest,
                                double speedMs) {
  if (vehicle.type != dest.type) {
    return false;
  }
  const double horizon = static_cast<double>(dest.arriveByUnix - vehicle.timestamp);
  if (horizon < 1.0) {
    return false;
  }
  const double dist =
      haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
  // Straight line at ~65% of road speed is a conservative travel-time lower bound.
  const double minTravelSec = dist / (std::max(speedMs, 5.0) * 0.65);
  return minTravelSec <= horizon + 120.0;
}

void addRoutingTargetNodes(const MultimodalGraph& graph, const GraphPosition& pos,
                           std::unordered_set<int64_t>& targets) {
  if (!pos.valid) {
    return;
  }
  if (pos.edgeId == 0) {
    targets.insert(pos.nodeId);
    return;
  }
  const Edge* edge = graph.findEdge(pos.edgeId);
  if (edge == nullptr) {
    return;
  }
  targets.insert(edge->from);
  targets.insert(edge->to);
}

std::optional<int64_t> chNodeFromSnap(const GraphFileStore& store, const ChGraph& ch,
                                      const GraphPosition& pos) {
  if (!pos.valid) {
    return std::nullopt;
  }
  if (pos.edgeId == 0) {
    if (ch.nodeIndex(pos.nodeId) >= 0) {
      return pos.nodeId;
    }
    return std::nullopt;
  }
  int64_t from = 0;
  int64_t to = 0;
  if (!store.edgeEndpoints(pos.edgeId, from, to)) {
    return std::nullopt;
  }
  if (ch.nodeIndex(from) >= 0) {
    return from;
  }
  if (ch.nodeIndex(to) >= 0) {
    return to;
  }
  return std::nullopt;
}

std::optional<int64_t> nearestChNodeViaHighwayCsr(const GraphFileStore& store, const ChGraph& ch,
                                                  const CsrGraph& hwyCsr, const GraphPosition& pos,
                                                  double maxStubMeters) {
  if (!pos.valid || maxStubMeters <= 0.0) {
    return std::nullopt;
  }

  std::vector<int64_t> seeds;
  if (pos.edgeId == 0) {
    seeds.push_back(pos.nodeId);
  } else {
    int64_t from = 0;
    int64_t to = 0;
    if (store.edgeEndpoints(pos.edgeId, from, to)) {
      seeds.push_back(from);
      seeds.push_back(to);
    }
  }
  if (seeds.empty()) {
    return std::nullopt;
  }

  for (int64_t nodeId : seeds) {
    if (ch.nodeIndex(nodeId) >= 0) {
      return nodeId;
    }
  }

  using QItem = std::pair<double, int64_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
  std::unordered_map<int64_t, double> best;
  best.reserve(4096);
  for (int64_t nodeId : seeds) {
    best[nodeId] = 0.0;
    pq.push({0.0, nodeId});
  }

  while (!pq.empty()) {
    const auto [distM, nodeId] = pq.top();
    pq.pop();
    const auto bestIt = best.find(nodeId);
    if (bestIt == best.end() || distM > bestIt->second + 1e-6) {
      continue;
    }
    if (distM > maxStubMeters || best.size() > 4000) {
      break;
    }
    if (ch.nodeIndex(nodeId) >= 0) {
      return nodeId;
    }

    hwyCsr.forEachNeighbor(store, nodeId, nullptr, [&](const CsrArc& arc) {
      const double nextM = distM + static_cast<double>(arc.length);
      if (nextM > maxStubMeters) {
        return;
      }
      const auto it = best.find(arc.toNodeId);
      if (it == best.end() || nextM < it->second - 1e-6) {
        best[arc.toNodeId] = nextM;
        pq.push({nextM, arc.toNodeId});
      }
    });
  }
  return std::nullopt;
}

std::optional<int64_t> nearestChNodeViaLocalBBox(
    const GraphFileStore& store, const SpatialIndex& index, const ChGraph& ch,
    const VehicleInfo& anchor, double radiusM, const GraphPosition& startPos, double speedMs,
    VehicleType type, const PredictParam& param, double maxHorizon) {
  if (radiusM <= 0.0 || !startPos.valid) {
    return std::nullopt;
  }

  std::unordered_set<int64_t> edgeIds;
  const LatLon center{anchor.lat, anchor.lon};
  collectEdgesInBboxIndexed(store, index, bboxAroundSegment(center, center, radiusM), edgeIds);
  if (edgeIds.empty()) {
    return std::nullopt;
  }

  GraphContext localCtx;
  std::string err;
  if (!store.loadGraphSubset(edgeIds, localCtx.graph, &err)) {
    return std::nullopt;
  }

  std::unordered_set<int64_t> chTargets;
  for (const Node& node : localCtx.graph.nodes()) {
    if (ch.nodeIndex(node.id) >= 0) {
      chTargets.insert(node.id);
    }
  }
  if (chTargets.empty()) {
    return std::nullopt;
  }

  GraphPosition localStart = matchVehicleToGraph(localCtx.graph, anchor, &index);
  if (!localStart.valid) {
    return std::nullopt;
  }

  PredictParam localParam = param;
  localParam.maxVisitedNodes = 40000;
  const TimeField field = computeTimeField(localCtx.graph, localStart, speedMs, type, localParam,
                                          maxHorizon);

  int64_t bestNode = 0;
  double bestT = kInfTime;
  for (int64_t nodeId : chTargets) {
    const auto it = field.atNode.find(nodeId);
    if (it != field.atNode.end() && it->second < bestT) {
      bestT = it->second;
      bestNode = nodeId;
    }
  }
  if (bestNode == 0 || bestT >= kInfTime / 2.0) {
    return std::nullopt;
  }
  return bestNode;
}

std::optional<int64_t> nearestChNodeViaLocalGraph(
    const GraphFileStore& store, const SpatialIndex& index, const ChGraph& ch,
    const VehicleInfo& anchor, double partnerLat, double partnerLon, double corridorWidthM,
    const GraphPosition& startPos, double speedMs, VehicleType type, const PredictParam& param,
    double maxHorizon) {
  GraphContext corridorCtx;
  std::string err;
  if (!extractGraphContextForPairIndexed(store, index, anchor, partnerLat, partnerLon,
                                         corridorWidthM, corridorCtx, &err)) {
    return std::nullopt;
  }

  std::unordered_set<int64_t> chTargets;
  for (const Node& node : corridorCtx.graph.nodes()) {
    if (ch.nodeIndex(node.id) >= 0) {
      chTargets.insert(node.id);
    }
  }
  if (chTargets.empty()) {
    return std::nullopt;
  }

  GraphPosition localStart = startPos;
  if (!localStart.valid || localStart.edgeId == 0) {
    localStart = matchVehicleToGraph(corridorCtx.graph, anchor, &index);
  } else {
    localStart = matchVehicleToGraph(corridorCtx.graph, anchor, &index);
  }
  if (!localStart.valid) {
    return std::nullopt;
  }

  PredictParam localParam = param;
  localParam.maxVisitedNodes = 120000;
  const TimeField field = computeTimeField(corridorCtx.graph, localStart, speedMs, type, localParam,
                                          maxHorizon);

  int64_t bestNode = 0;
  double bestT = kInfTime;
  for (int64_t nodeId : chTargets) {
    const auto it = field.atNode.find(nodeId);
    if (it != field.atNode.end() && it->second < bestT) {
      bestT = it->second;
      bestNode = nodeId;
    }
  }
  if (bestNode == 0 || bestT >= kInfTime / 2.0) {
    return std::nullopt;
  }
  return bestNode;
}

RoutePolyline appendPolylines(RoutePolyline a, const RoutePolyline& b) {
  if (a.points.empty()) {
    return b;
  }
  for (const LatLon& p : b.points) {
    if (a.points.empty() || haversineMeters(a.points.back(), p) > 1.0) {
      a.points.push_back(p);
    }
  }
  return a;
}

std::optional<VehicleArrivalResult> predictOneVehicleHighwayCh(
    const VehicleInfo& vehicle, const VehicleHistory* history, const GraphFileStore& store,
    const SpatialIndex& matchIndex, const DestinationQuery& dest, const VehicleInfo& destProbe,
    const GraphPosition& goalSnap, const PredictParam& param) {
  const ChGraph& ch = store.highwayCh();
  const CsrGraph& hwyCsr = store.highwayCsr();
  if (!ch.isOpen() || !hwyCsr.isOpen() || !goalSnap.valid) {
    return std::nullopt;
  }

  const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, history, nullptr, vehicle.type));
  const double maxHorizon =
      std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
  if (maxHorizon < 1.0) {
    return std::nullopt;
  }

  const GraphPosition startSnap = matchVehicleToGraphIndexed(store, matchIndex, vehicle);
  if (!startSnap.valid) {
    return std::nullopt;
  }

  const double distM = haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
  const double stubM = std::min(25000.0, distM * 0.2 + 8000.0);

  std::optional<int64_t> startCh = chNodeFromSnap(store, ch, startSnap);
  if (!startCh) {
    startCh = nearestChNodeViaHighwayCsr(store, ch, hwyCsr, startSnap, stubM);
  }
  if (!startCh) {
    startCh = nearestChNodeViaLocalBBox(store, matchIndex, ch, vehicle, std::min(8000.0, stubM),
                                      startSnap, speedMs, vehicle.type, param, maxHorizon);
  }
  std::optional<int64_t> goalCh = chNodeFromSnap(store, ch, goalSnap);
  if (!goalCh) {
    goalCh = nearestChNodeViaHighwayCsr(store, ch, hwyCsr, goalSnap, stubM);
  }
  if (!goalCh) {
    goalCh = nearestChNodeViaLocalBBox(store, matchIndex, ch, destProbe, std::min(8000.0, stubM),
                                       goalSnap, speedMs, dest.type, param, maxHorizon);
  }
  if (!startCh || !goalCh) {
    return std::nullopt;
  }

  RouteToGoal total;
  if (*startCh != *goalCh) {
    total = ch.query(store, hwyCsr, *startCh, *goalCh, vehicle.type, param, maxHorizon);
    if (total.travelTimeSec >= kInfTime / 2.0) {
      return std::nullopt;
    }
  } else {
    total.travelTimeSec = 0.0;
  }

  double travel = total.travelTimeSec;
  RoutePolyline route = total.polyline;

  const bool startOnCh = chNodeFromSnap(store, ch, startSnap).has_value();
  if (!startOnCh) {
    GraphContext localCtx;
    std::string err;
    const double localW = std::min(8000.0, stubM);
    if (extractGraphContextForPairIndexed(store, matchIndex, vehicle, dest.lat, dest.lon, localW,
                                          localCtx, &err)) {
      GraphPosition lg;
      lg.valid = true;
      lg.nodeId = *startCh;
      lg.edgeId = 0;
      const RouteToGoal local = computeRouteToGoal(
          localCtx.graph, matchVehicleToGraph(localCtx.graph, vehicle, &matchIndex), lg, speedMs,
          vehicle.type, param, maxHorizon);
      if (local.travelTimeSec < kInfTime / 2.0) {
        travel += local.travelTimeSec;
        route = appendPolylines(local.polyline, route);
      }
    }
  }

  const bool goalOnCh = chNodeFromSnap(store, ch, goalSnap).has_value();
  if (!goalOnCh) {
    GraphContext localCtx;
    std::string err;
    const double localW = std::min(8000.0, stubM);
    if (extractGraphContextForPairIndexed(store, matchIndex, destProbe, vehicle.lat, vehicle.lon,
                                          localW, localCtx, &err)) {
      GraphPosition lg;
      lg.valid = true;
      lg.nodeId = *goalCh;
      lg.edgeId = 0;
      const RouteToGoal local = computeRouteToGoal(
          localCtx.graph, lg, matchVehicleToGraph(localCtx.graph, destProbe, &matchIndex), speedMs,
          dest.type, param, maxHorizon);
      if (local.travelTimeSec < kInfTime / 2.0) {
        travel += local.travelTimeSec;
        route = appendPolylines(route, local.polyline);
      }
    }
  }

  if (travel > maxHorizon + 1e-6) {
    return std::nullopt;
  }
  const double eta = static_cast<double>(vehicle.timestamp) + travel;
  if (eta > static_cast<double>(dest.arriveByUnix) + 1e-6) {
    return std::nullopt;
  }

  VehicleArrivalResult row;
  row.vehicleId = vehicle.id;
  row.reachable = true;
  row.travelDurationSec = travel;
  row.etaUnix = eta;
  row.route = route;
  row.routeDistanceM = polylineLengthMeters(row.route);
  simplifyRoutePolyline(row.route, 120);
  return row;
}

}  // namespace

std::string graphLocationId(const GraphPosition& pos) {
  if (!pos.valid) {
    return {};
  }
  if (pos.edgeId == 0) {
    std::ostringstream oss;
    oss << pos.nodeId;
    return oss.str();
  }
  std::ostringstream oss;
  oss << "edge:" << pos.edgeId;
  return oss.str();
}

void sortDestinationArrivals(std::vector<VehicleArrivalResult>& rows, ArrivalSortBy sortBy) {
  switch (sortBy) {
    case ArrivalSortBy::ETA:
      std::sort(rows.begin(), rows.end(),
                [](const VehicleArrivalResult& a, const VehicleArrivalResult& b) {
                  return a.etaUnix < b.etaUnix;
                });
      break;
    case ArrivalSortBy::DISTANCE:
      std::sort(rows.begin(), rows.end(),
                [](const VehicleArrivalResult& a, const VehicleArrivalResult& b) {
                  return a.routeDistanceM < b.routeDistanceM;
                });
      break;
    default:
      std::sort(rows.begin(), rows.end(),
                [](const VehicleArrivalResult& a, const VehicleArrivalResult& b) {
                  return a.travelDurationSec < b.travelDurationSec;
                });
      break;
  }
}

std::optional<VehicleArrivalResult> predictVehicleToDestination(
    const VehicleInfo& vehicle, const VehicleHistory* history, const GraphContext& routeCtx,
    const SpatialIndex& matchIndex, const DestinationQuery& dest, const GraphPosition& goalPos,
    const PredictParam& param) {
  if (vehicle.type != dest.type || !goalPos.valid) {
    return std::nullopt;
  }

  const double maxHorizon =
      std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
  if (maxHorizon < 1.0) {
    return std::nullopt;
  }

  const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, history, nullptr, vehicle.type));
  if (!vehicleMayReachDestination(vehicle, dest, speedMs)) {
    return std::nullopt;
  }

  const MultimodalGraph& graph = routeCtx.graph;
  const PreparedVehicle prepared =
      prepareVehicle(graph, vehicle, history, vehicle.timestamp, param, &matchIndex);
  if (!prepared.valid) {
    return std::nullopt;
  }

  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;  // destination A* must not truncate long hauls

  const RouteToGoal path = computeRouteToGoal(graph, prepared.position, goalPos, prepared.speedMs,
                                              vehicle.type, routeParam, maxHorizon);
  const double travel = path.travelTimeSec;
  if (travel >= kInfTime / 2.0 || travel > maxHorizon + 1e-6) {
    return std::nullopt;
  }

  const double eta = static_cast<double>(vehicle.timestamp) + travel;
  if (eta > static_cast<double>(dest.arriveByUnix) + 1e-6) {
    return std::nullopt;
  }

  VehicleArrivalResult row;
  row.vehicleId = vehicle.id;
  row.reachable = true;
  row.travelDurationSec = travel;
  row.etaUnix = eta;
  row.routeDistanceM = polylineLengthMeters(path.polyline);
  row.route = path.polyline;
  simplifyRoutePolyline(row.route, 120);
  return row;
}

DestinationArrivalSummary predictVehiclesToDestination(
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const GraphContext& routeCtx, const SpatialIndex& matchIndex, const DestinationQuery& dest,
    const PredictParam& param) {
  DestinationArrivalSummary summary;
  summary.lat = dest.lat;
  summary.lon = dest.lon;
  summary.arriveByUnix = dest.arriveByUnix;
  summary.sortBy = dest.sortBy;

  const MultimodalGraph& graph = routeCtx.graph;
  VehicleInfo destProbe;
  destProbe.id = "destination";
  destProbe.lat = dest.lat;
  destProbe.lon = dest.lon;
  destProbe.type = dest.type;
  destProbe.speed = 60.0;
  destProbe.timestamp = dest.arriveByUnix;

  const GraphPosition goalPos = matchVehicleToGraph(graph, destProbe, &matchIndex);
  summary.locationId = graphLocationId(goalPos);
  if (!goalPos.valid) {
    return summary;
  }

  struct VehicleJob {
    const VehicleInfo* vehicle = nullptr;
    const VehicleHistory* history = nullptr;
    double speedMs = 0.0;
    double maxHorizon = 0.0;
  };
  std::vector<VehicleJob> jobs;
  jobs.reserve(vehicles.size());

  for (const auto& vehicle : vehicles) {
    if (vehicle.type != dest.type) {
      continue;
    }
    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, hist, nullptr, vehicle.type));
    if (!vehicleMayReachDestination(vehicle, dest, speedMs)) {
      continue;
    }
    const double maxHorizon =
        std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
    if (maxHorizon < 1.0) {
      continue;
    }
    jobs.push_back({&vehicle, hist, speedMs, maxHorizon});
  }

  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;

  std::map<int, std::vector<VehicleJob>> bySpeedKey;
  for (const auto& job : jobs) {
    const int key = static_cast<int>(std::lround(job.speedMs * 100.0));
    bySpeedKey[key].push_back(job);
  }

  std::vector<std::pair<int, std::vector<VehicleJob>>> speedGroups;
  speedGroups.reserve(bySpeedKey.size());
  for (auto& kv : bySpeedKey) {
    speedGroups.emplace_back(kv.first, std::move(kv.second));
  }

  summary.vehicles.reserve(jobs.size());
  std::vector<std::future<std::vector<VehicleArrivalResult>>> futures;
  futures.reserve(speedGroups.size());
  for (auto& [speedKey, groupJobs] : speedGroups) {
    futures.push_back(std::async(
        std::launch::async,
        [&graph, &matchIndex, &goalPos, &dest, &param, routeParam, speedKey,
         groupJobs = std::move(groupJobs)]() {
          std::vector<VehicleArrivalResult> rows;
          const double speedMs = speedKey / 100.0;
          double groupMaxHorizon = 0.0;
          std::vector<std::pair<const VehicleJob*, PreparedVehicle>> preparedJobs;
          preparedJobs.reserve(groupJobs.size());
          std::unordered_set<int64_t> targetNodes;
          targetNodes.reserve(groupJobs.size() * 2);

          for (const auto& job : groupJobs) {
            groupMaxHorizon = std::max(groupMaxHorizon, job.maxHorizon);
            const PreparedVehicle prepared =
                prepareVehicle(graph, *job.vehicle, job.history, job.vehicle->timestamp, param,
                               &matchIndex);
            if (!prepared.valid) {
              continue;
            }
            addRoutingTargetNodes(graph, prepared.position, targetNodes);
            preparedJobs.push_back({&job, prepared});
          }
          if (preparedJobs.empty()) {
            return rows;
          }

          const RoutedTimeField field = computeRoutedTimeFieldFromGoal(
              graph, goalPos, speedMs, dest.type, routeParam, groupMaxHorizon, &targetNodes);

          for (const auto& [job, prepared] : preparedJobs) {
            const RouteToGoal path =
                routeFromRoutedField(graph, field, prepared.position, goalPos, prepared.speedMs,
                                     job->vehicle->type, routeParam);
            const double travel = path.travelTimeSec;
            if (travel >= kInfTime / 2.0 || travel > job->maxHorizon + 1e-6) {
              continue;
            }

            const double eta = static_cast<double>(job->vehicle->timestamp) + travel;
            if (eta > static_cast<double>(dest.arriveByUnix) + 1e-6) {
              continue;
            }

            VehicleArrivalResult row;
            row.vehicleId = job->vehicle->id;
            row.reachable = true;
            row.travelDurationSec = travel;
            row.etaUnix = eta;
            row.routeDistanceM = polylineLengthMeters(path.polyline);
            row.route = path.polyline;
            simplifyRoutePolyline(row.route, 120);
            rows.push_back(std::move(row));
          }
          return rows;
        }));
  }

  for (auto& future : futures) {
    std::vector<VehicleArrivalResult> rows = future.get();
    for (auto& row : rows) {
      summary.vehicles.push_back(std::move(row));
    }
  }

  sortDestinationArrivals(summary.vehicles, dest.sortBy);
  return summary;
}

DestinationArrivalSummary predictVehiclesToDestinationIndexed(
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const GraphFileStore& store, const SpatialIndex& matchIndex, const DestinationQuery& dest,
    double maxCorridorWidthM, const PredictParam& param) {
  DestinationArrivalSummary summary;
  summary.lat = dest.lat;
  summary.lon = dest.lon;
  summary.arriveByUnix = dest.arriveByUnix;
  summary.sortBy = dest.sortBy;

  VehicleInfo destProbe;
  destProbe.id = "__destination__";
  destProbe.lat = dest.lat;
  destProbe.lon = dest.lon;
  destProbe.type = dest.type;
  destProbe.speed = 60.0;
  destProbe.timestamp = dest.arriveByUnix;

  std::vector<VehicleInfo> routable;
  routable.reserve(vehicles.size());
  std::size_t pruned = 0;
  double maxSpanM = 0.0;
  for (const auto& vehicle : vehicles) {
    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, hist, nullptr, vehicle.type));
    if (!vehicleMayReachDestination(vehicle, dest, speedMs)) {
      ++pruned;
      continue;
    }
    maxSpanM = std::max(
        maxSpanM, haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon}));
    routable.push_back(vehicle);
  }

  if (routable.empty()) {
    std::cerr << "[mmlp] destination indexed vehicles=" << vehicles.size() << " pruned=" << pruned
              << " reachable=0\n"
              << std::flush;
    return summary;
  }

  const auto t0 = std::chrono::steady_clock::now();
  std::int64_t collectMs = 0;
  std::int64_t extractMs = 0;

  constexpr double kPerVehicleThresholdM = 600000.0;
  constexpr double kCompactSpanM = 60000.0;
  constexpr std::size_t kCompactMaxVehicles = 4;

  std::vector<double> distsToDest;
  distsToDest.reserve(routable.size());
  for (const auto& vehicle : routable) {
    distsToDest.push_back(
        haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon}));
  }
  const auto medianIt = distsToDest.begin() + distsToDest.size() / 2;
  std::nth_element(distsToDest.begin(), medianIt, distsToDest.end());
  const double medianDistM = *medianIt;

  std::vector<VehicleInfo> collective;
  std::vector<VehicleInfo> remote;
  collective.reserve(routable.size());
  remote.reserve(routable.size());
  for (std::size_t i = 0; i < routable.size(); ++i) {
    const double distM = distsToDest[i];
    const bool outlier =
        distM > 3.0 * medianDistM && (distM - medianDistM) > 500000.0;
    if (outlier) {
      remote.push_back(routable[i]);
    } else {
      collective.push_back(routable[i]);
    }
  }

  double collectiveSpanM = 0.0;
  for (const auto& vehicle : collective) {
    collectiveSpanM = std::max(
        collectiveSpanM,
        haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon}));
  }

  if (!collective.empty() && collectiveSpanM <= kPerVehicleThresholdM) {
    const bool smallFleet = collective.size() <= kCompactMaxVehicles;

    const GraphPosition goalPos =
        matchVehicleToGraphIndexed(store, matchIndex, destProbe);
    summary.locationId = graphLocationId(goalPos);
    if (!goalPos.valid) {
      return summary;
    }

    if (smallFleet) {
      const auto tSmall0 = std::chrono::steady_clock::now();
      struct SmallJob {
        const VehicleInfo* vehicle = nullptr;
        const VehicleHistory* history = nullptr;
      };
      std::vector<SmallJob> jobs;
      jobs.reserve(collective.size());
      for (const auto& vehicle : collective) {
        jobs.push_back({&vehicle, findHistory(histories, vehicle.id)});
      }

      std::vector<std::optional<VehicleArrivalResult>> rows(jobs.size());
      std::vector<std::future<void>> futures;
      futures.reserve(jobs.size());
      for (std::size_t i = 0; i < jobs.size(); ++i) {
        futures.push_back(std::async(std::launch::async, [&, i]() {
          const SmallJob& job = jobs[i];
          const double dist = haversineMeters({job.vehicle->lat, job.vehicle->lon},
                                              {dest.lat, dest.lon});
          GraphContext corridorCtx;
          std::string corridorErr;
          bool loaded = false;

          constexpr double kNearDestM = 45000.0;
          constexpr double kMediumDestM = 120000.0;
          if (dist <= kNearDestM) {
            std::unordered_set<int64_t> edgeIds;
            if (collectDestinationCorridorEdgeIdsIndexed(store, matchIndex, {*job.vehicle},
                                                         dest.lat, dest.lon, maxCorridorWidthM,
                                                         edgeIds, &corridorErr)) {
              loaded = store.loadGraphSubset(edgeIds, corridorCtx.graph, &corridorErr);
            }
          } else if (dist <= kMediumDestM) {
            const double corridorW =
                std::min(maxCorridorWidthM, std::max(8000.0, dist * 0.08 + 6000.0));
            loaded = extractGraphContextForPairIndexed(store, matchIndex, *job.vehicle, dest.lat,
                                                       dest.lon, corridorW, corridorCtx,
                                                       &corridorErr);
          } else {
            const double corridorW =
                dist > 40000.0 ? std::min(maxCorridorWidthM, 22000.0)
                               : std::min(maxCorridorWidthM, dist * 0.35 + 12000.0);
            loaded = extractGraphContextForPairIndexed(store, matchIndex, *job.vehicle, dest.lat,
                                                       dest.lon, corridorW, corridorCtx,
                                                       &corridorErr);
          }
          if (!loaded) {
            return;
          }
          const GraphPosition goalOnCorridor =
              matchVehicleToGraph(corridorCtx.graph, destProbe, &matchIndex);
          if (!goalOnCorridor.valid) {
            return;
          }
          rows[i] = predictVehicleToDestination(*job.vehicle, job.history, corridorCtx,
                                                matchIndex, dest, goalOnCorridor, param);
        }));
      }
      for (auto& future : futures) {
        future.get();
      }
      for (auto& row : rows) {
        if (row) {
          summary.vehicles.push_back(std::move(*row));
        }
      }

      std::int64_t predictMs = 0;
      if (!remote.empty()) {
        const auto tRemote0 = std::chrono::steady_clock::now();
        struct RemoteJob {
          const VehicleInfo* vehicle = nullptr;
          const VehicleHistory* history = nullptr;
        };
        std::vector<RemoteJob> remoteJobs;
        remoteJobs.reserve(remote.size());
        for (const auto& vehicle : remote) {
          remoteJobs.push_back({&vehicle, findHistory(histories, vehicle.id)});
        }

        std::vector<std::optional<VehicleArrivalResult>> remoteRows(remoteJobs.size());
        std::vector<std::future<void>> remoteFutures;
        remoteFutures.reserve(remoteJobs.size());
        for (std::size_t i = 0; i < remoteJobs.size(); ++i) {
          remoteFutures.push_back(std::async(std::launch::async, [&, i]() {
            const RemoteJob& job = remoteJobs[i];
            const double dist = haversineMeters({job.vehicle->lat, job.vehicle->lon},
                                                {dest.lat, dest.lon});
            const double corridorW =
                dist > 40000.0 ? std::min(maxCorridorWidthM, 22000.0)
                               : std::min(maxCorridorWidthM, dist * 0.35 + 12000.0);
            GraphContext corridorCtx;
            std::string corridorErr;
            if (!extractGraphContextForPairIndexed(store, matchIndex, *job.vehicle, dest.lat,
                                                   dest.lon, corridorW, corridorCtx,
                                                   &corridorErr)) {
              return;
            }
            const GraphPosition goalOnCorridor =
                matchVehicleToGraph(corridorCtx.graph, destProbe, &matchIndex);
            if (!goalOnCorridor.valid) {
              return;
            }
            remoteRows[i] = predictVehicleToDestination(*job.vehicle, job.history, corridorCtx,
                                                          matchIndex, dest, goalOnCorridor, param);
          }));
        }
        for (auto& future : remoteFutures) {
          future.get();
        }
        for (auto& row : remoteRows) {
          if (row) {
            summary.vehicles.push_back(std::move(*row));
          }
        }
        predictMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - tRemote0)
                        .count();
      }

      sortDestinationArrivals(summary.vehicles, dest.sortBy);
      const auto tSmall1 = std::chrono::steady_clock::now();
      predictMs += std::chrono::duration_cast<std::chrono::milliseconds>(tSmall1 - tSmall0).count();
      std::cerr << "[mmlp] destination indexed vehicles=" << vehicles.size() << " pruned=" << pruned
                << " routable=" << routable.size() << " collective=" << collective.size()
                << " remote=" << remote.size() << " per_vehicle=1"
                << " predict_ms=" << predictMs << " total_ms="
                << std::chrono::duration_cast<std::chrono::milliseconds>(tSmall1 - t0).count()
                << " reachable=" << summary.vehicles.size() << "\n"
                << std::flush;
      return summary;
    }

    const auto tc0 = std::chrono::steady_clock::now();
    std::unordered_set<int64_t> allowedEdges;
    std::string collectErr;
    bool collected = false;

    const bool compactCluster = collectiveSpanM <= kCompactSpanM;
    const bool dispersedFleet =
        collective.size() >= 4 && collectiveSpanM > kCompactSpanM;

    GraphContext regionCtx;
    std::string extractErr;

    if (compactCluster) {
      collected = collectDestinationCorridorEdgeIdsIndexed(
          store, matchIndex, collective, dest.lat, dest.lon, maxCorridorWidthM, allowedEdges,
          &collectErr);
    } else if (dispersedFleet) {
      std::vector<VehicleInfo> forExtract = collective;
      forExtract.push_back(destProbe);
      const double padM =
          std::min(maxCorridorWidthM, std::max(20000.0, collectiveSpanM * 0.15 + 12000.0));
      collected = collectDestinationBBoxEdgeIdsIndexed(store, matchIndex, forExtract, padM,
                                                       allowedEdges, &collectErr);
      if (collected) {
        pruneDestinationEdgeIdsToCorridors(store, collective, dest.lat, dest.lon,
                                           maxCorridorWidthM, allowedEdges);
      }
    } else {
      collected = collectDestinationCorridorEdgeIdsIndexed(
          store, matchIndex, collective, dest.lat, dest.lon, maxCorridorWidthM, allowedEdges,
          &collectErr);
    }
    const auto tc1 = std::chrono::steady_clock::now();
    collectMs = std::chrono::duration_cast<std::chrono::milliseconds>(tc1 - tc0).count();
    if (!collected) {
      std::cerr << "[mmlp] destination edge collect failed: " << collectErr << "\n" << std::flush;
      return summary;
    }

    if (false && store.hasCsr()) {
      // National mmap CSR + per-edge hash filter scans all node arcs nationwide
      // (much slower than compact subgraph). Reserved for future CH routing.
      const auto t1 = std::chrono::steady_clock::now();

      struct VehicleJob {
        const VehicleInfo* vehicle = nullptr;
        const VehicleHistory* history = nullptr;
        double speedMs = 0.0;
        double maxHorizon = 0.0;
      };
      std::vector<VehicleJob> jobs;
      jobs.reserve(routable.size());
      for (const auto& vehicle : collective) {
        const VehicleHistory* hist = findHistory(histories, vehicle.id);
        const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, hist, nullptr, vehicle.type));
        const double maxHorizon =
            std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
        if (maxHorizon < 1.0) {
          continue;
        }
        jobs.push_back({&vehicle, hist, speedMs, maxHorizon});
      }

      PredictParam routeParam = param;
      routeParam.maxVisitedNodes = 0;

      std::map<int, std::vector<VehicleJob>> bySpeedKey;
      for (const auto& job : jobs) {
        bySpeedKey[static_cast<int>(std::lround(job.speedMs * 100.0))].push_back(job);
      }

      summary.vehicles.reserve(jobs.size());
      const CsrGraph& csr = store.csr();
      for (auto& kv : bySpeedKey) {
        const double speedMs = kv.first / 100.0;
        double groupMaxHorizon = 0.0;
        for (const auto& job : kv.second) {
          groupMaxHorizon = std::max(groupMaxHorizon, job.maxHorizon);
        }

        const RoutedTimeField field = computeRoutedTimeFieldFromGoalCsr(
            store, csr, goalPos, speedMs, dest.type, routeParam, groupMaxHorizon, &allowedEdges);

        for (const auto& job : kv.second) {
          const GraphPosition startPos =
              matchVehicleToGraphIndexed(store, matchIndex, *job.vehicle);
          if (!startPos.valid) {
            continue;
          }

          const RouteToGoal path = routeFromRoutedFieldCsr(store, csr, field, startPos, goalPos,
                                                         job.speedMs, job.vehicle->type, routeParam);
          const double travel = path.travelTimeSec;
          if (travel >= kInfTime / 2.0 || travel > job.maxHorizon + 1e-6) {
            continue;
          }

          const double eta = static_cast<double>(job.vehicle->timestamp) + travel;
          if (eta > static_cast<double>(dest.arriveByUnix) + 1e-6) {
            continue;
          }

          VehicleArrivalResult row;
          row.vehicleId = job.vehicle->id;
          row.reachable = true;
          row.travelDurationSec = travel;
          row.etaUnix = eta;
          row.routeDistanceM = polylineLengthMeters(path.polyline);
          row.route = path.polyline;
          simplifyRoutePolyline(row.route, 120);
          summary.vehicles.push_back(std::move(row));
        }
      }

      sortDestinationArrivals(summary.vehicles, dest.sortBy);

      const auto t2 = std::chrono::steady_clock::now();
      std::cerr << "[mmlp] destination indexed(csr) vehicles=" << vehicles.size()
                << " pruned=" << pruned << " routable=" << routable.size()
                << " filter_edges=" << allowedEdges.size() << " egeo=" << (store.hasEdgeGeo() ? 1 : 0)
                << " csr=1 collect_ms=" << collectMs << " total_ms="
                << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0).count()
                << " predict_ms="
                << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                << " reachable=" << summary.vehicles.size() << "\n"
                << std::flush;
      return summary;
    }

    const auto tLoad0 = std::chrono::steady_clock::now();
    if (!store.loadGraphSubset(allowedEdges, regionCtx.graph, &extractErr)) {
      std::cerr << "[mmlp] destination subset load failed: " << extractErr << "\n" << std::flush;
      return summary;
    }
    const auto tLoad1 = std::chrono::steady_clock::now();
    extractMs = std::chrono::duration_cast<std::chrono::milliseconds>(tLoad1 - tLoad0).count();

    DestinationArrivalSummary localSummary = predictVehiclesToDestination(
        collective, histories, regionCtx, matchIndex, dest, param);
    summary.vehicles.insert(summary.vehicles.end(),
                            std::make_move_iterator(localSummary.vehicles.begin()),
                            std::make_move_iterator(localSummary.vehicles.end()));
    summary.sortBy = dest.sortBy;

    const auto tPredict1 = std::chrono::steady_clock::now();
    std::int64_t predictMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(tPredict1 - tLoad1).count();

    if (!remote.empty()) {
      const auto tRemote0 = std::chrono::steady_clock::now();
      struct RemoteJob {
        const VehicleInfo* vehicle = nullptr;
        const VehicleHistory* history = nullptr;
      };
      std::vector<RemoteJob> remoteJobs;
      remoteJobs.reserve(remote.size());
      for (const auto& vehicle : remote) {
        remoteJobs.push_back({&vehicle, findHistory(histories, vehicle.id)});
      }

      std::vector<std::optional<VehicleArrivalResult>> remoteRows(remoteJobs.size());
      std::vector<std::future<void>> remoteFutures;
      remoteFutures.reserve(remoteJobs.size());
      for (std::size_t i = 0; i < remoteJobs.size(); ++i) {
        remoteFutures.push_back(std::async(std::launch::async, [&, i]() {
          const RemoteJob& job = remoteJobs[i];
          const double dist = haversineMeters({job.vehicle->lat, job.vehicle->lon},
                                              {dest.lat, dest.lon});
          const double corridorW =
              dist > 40000.0 ? std::min(maxCorridorWidthM, 22000.0)
                             : std::min(maxCorridorWidthM, dist * 0.35 + 12000.0);
          GraphContext corridorCtx;
          std::string corridorErr;
          if (!extractGraphContextForPairIndexed(store, matchIndex, *job.vehicle, dest.lat,
                                                 dest.lon, corridorW, corridorCtx,
                                                 &corridorErr)) {
            return;
          }
          const GraphPosition goalOnCorridor =
              matchVehicleToGraph(corridorCtx.graph, destProbe, &matchIndex);
          if (!goalOnCorridor.valid) {
            return;
          }
          remoteRows[i] = predictVehicleToDestination(*job.vehicle, job.history, corridorCtx,
                                                      matchIndex, dest, goalOnCorridor, param);
        }));
      }
      for (auto& future : remoteFutures) {
        future.get();
      }
      for (auto& row : remoteRows) {
        if (row) {
          summary.vehicles.push_back(std::move(*row));
        }
      }
      predictMs += std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - tRemote0)
                       .count();
    }

    sortDestinationArrivals(summary.vehicles, dest.sortBy);

    const auto t2 = std::chrono::steady_clock::now();
    std::cerr << "[mmlp] destination indexed vehicles=" << vehicles.size() << " pruned=" << pruned
              << " routable=" << routable.size() << " collective=" << collective.size()
              << " remote=" << remote.size() << " sub_edges=" << regionCtx.graph.edges().size()
              << " candidates=" << allowedEdges.size() << " egeo=" << (store.hasEdgeGeo() ? 1 : 0)
              << " csr=0 compact=" << (compactCluster ? 1 : 0)
              << " dispersed=" << (dispersedFleet ? 1 : 0) << " collect_ms=" << collectMs
              << " extract_ms=" << extractMs << " predict_ms=" << predictMs
              << " total_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0).count()
              << " reachable=" << summary.vehicles.size() << "\n"
              << std::flush;
    return summary;
  }

  if (!remote.empty()) {
    collective.insert(collective.end(), remote.begin(), remote.end());
    remote.clear();
  }
  const std::vector<VehicleInfo>& perVehicle = collective.empty() ? routable : collective;

  GraphContext destCtx;
  std::string destErr;
  if (!extractGraphContextForDestinationIndexed(store, matchIndex, {destProbe}, 8000.0, destCtx,
                                              &destErr)) {
    return summary;
  }
  summary.locationId =
      graphLocationId(matchVehicleToGraph(destCtx.graph, destProbe, &matchIndex));

  for (const auto& vehicle : perVehicle) {
    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    const double dist =
        haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
    const double corridorW =
        dist > 40000.0 ? std::min(maxCorridorWidthM, 22000.0)
                       : std::min(maxCorridorWidthM, dist * 0.35 + 12000.0);

    GraphContext corridorCtx;
    std::string corridorErr;
    if (!extractGraphContextForPairIndexed(store, matchIndex, vehicle, dest.lat, dest.lon,
                                           corridorW, corridorCtx, &corridorErr)) {
      std::cerr << "[mmlp] corridor extract " << vehicle.id << ": " << corridorErr << "\n"
                << std::flush;
      continue;
    }

    const GraphPosition goalOnCorridor =
        matchVehicleToGraph(corridorCtx.graph, destProbe, &matchIndex);
    if (!goalOnCorridor.valid) {
      continue;
    }

    if (auto row = predictVehicleToDestination(vehicle, hist, corridorCtx, matchIndex, dest,
                                               goalOnCorridor, param)) {
      summary.vehicles.push_back(std::move(*row));
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  std::cerr << "[mmlp] destination indexed (multi-region) vehicles=" << vehicles.size()
            << " pruned=" << pruned << " reachable=" << summary.vehicles.size() << " total_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n"
            << std::flush;

  sortDestinationArrivals(summary.vehicles, dest.sortBy);
  return summary;
}

}  // namespace mmlp
