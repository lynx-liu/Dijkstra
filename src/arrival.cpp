#include "mmlp/arrival.hpp"

#include "mmlp/ch_graph.hpp"
#include "mmlp/geo.hpp"
#include "mmlp/region_registry.hpp"
#include "mmlp/matching.hpp"
#include "mmlp/meeting.hpp"
#include "mmlp/motion.hpp"
#include "mmlp/routing.hpp"
#include "mmlp/thread_pool.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mmlp {

namespace {

template <typename F>
auto runTopLevelAsync(F&& fn) -> std::future<decltype(fn())> {
  // Top-level batch work must not use ThreadPool::submit — nested parallelFor inside
  // would exhaust workers and deadlock (e.g. Guangzhou 98-vehicle batches).
  return std::async(std::launch::async, std::forward<F>(fn));
}

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

bool routePolylineQualityOk(const RoutePolyline& route, double maxSegM = 12000.0);
bool routePolylineDisplayOk(const RoutePolyline& route, double haulDistM);
std::optional<RoutePolyline> stitchChOverlayRoutePolyline(
    const GraphFileStore& snapStore, const SpatialIndex& matchIndex, const VehicleInfo& vehicle,
    const DestinationQuery& dest, double speedMs, double maxHorizon, const PredictParam& param,
    const RoutePolyline& chPath);

bool mayReachWithSpeed(const VehicleInfo& vehicle, const DestinationQuery& dest, double speedMs) {
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

std::vector<std::vector<VehicleInfo>> clusterVehiclesByGrid(
    const std::vector<VehicleInfo>& vehicles, double cellDeg, std::size_t maxPerCluster) {
  std::unordered_map<int64_t, std::vector<VehicleInfo>> buckets;
  buckets.reserve(vehicles.size());
  for (const auto& vehicle : vehicles) {
    const int gx = static_cast<int>(std::floor(vehicle.lon / cellDeg));
    const int gy = static_cast<int>(std::floor(vehicle.lat / cellDeg));
    const int64_t key = (static_cast<int64_t>(gx) << 32) |
                        (static_cast<uint32_t>(gy) & 0xffffffffu);
    buckets[key].push_back(vehicle);
  }

  std::vector<std::vector<VehicleInfo>> clusters;
  clusters.reserve(buckets.size() + vehicles.size() / std::max<std::size_t>(1, maxPerCluster));
  for (auto& kv : buckets) {
    auto& group = kv.second;
    while (group.size() > maxPerCluster) {
      clusters.emplace_back(group.begin(),
                            group.begin() + static_cast<std::ptrdiff_t>(maxPerCluster));
      group.erase(group.begin(), group.begin() + static_cast<std::ptrdiff_t>(maxPerCluster));
    }
    if (!group.empty()) {
      clusters.push_back(std::move(group));
    }
  }
  return clusters;
}

std::vector<std::vector<VehicleInfo>> clusterVehiclesByBearingBins(
    const std::vector<VehicleInfo>& vehicles, double destLat, double destLon,
    std::size_t bins) {
  struct Row {
    double bearing = 0.0;
    VehicleInfo vehicle;
  };
  std::vector<Row> rows;
  rows.reserve(vehicles.size());
  for (const auto& vehicle : vehicles) {
    const double dy = (vehicle.lat - destLat) * M_PI / 180.0;
    const double dx =
        (vehicle.lon - destLon) * M_PI / 180.0 * std::cos(destLat * M_PI / 180.0);
    rows.push_back({std::atan2(dy, dx), vehicle});
  }
  std::sort(rows.begin(), rows.end(),
            [](const Row& a, const Row& b) { return a.bearing < b.bearing; });
  bins = std::max<std::size_t>(1, std::min(bins, rows.size()));
  std::vector<std::vector<VehicleInfo>> out;
  out.reserve(bins);
  const std::size_t per = (rows.size() + bins - 1) / bins;
  for (std::size_t i = 0; i < rows.size(); i += per) {
    std::vector<VehicleInfo> group;
    const std::size_t end = std::min(rows.size(), i + per);
    group.reserve(end - i);
    for (std::size_t j = i; j < end; ++j) {
      group.push_back(rows[j].vehicle);
    }
    out.push_back(std::move(group));
  }
  return out;
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

void addRoutingTargetNodesIndexed(const GraphFileStore& store, const GraphPosition& pos,
                                  std::unordered_set<int64_t>& targets) {
  if (!pos.valid) {
    return;
  }
  if (pos.edgeId == 0) {
    targets.insert(pos.nodeId);
    return;
  }
  int64_t from = 0;
  int64_t to = 0;
  if (store.edgeEndpoints(pos.edgeId, from, to)) {
    targets.insert(from);
    targets.insert(to);
  }
}

std::vector<VehicleArrivalResult> predictVehiclesToDestinationCsrBatch(
    const GraphFileStore& store, const SpatialIndex& matchIndex,
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const DestinationQuery& dest, double corridorWidthM, const PredictParam& param) {
  std::vector<VehicleArrivalResult> results;
  if (vehicles.empty() || !store.hasCsr()) {
    return results;
  }

  VehicleInfo destProbe;
  destProbe.id = "destination";
  destProbe.lat = dest.lat;
  destProbe.lon = dest.lon;
  destProbe.type = dest.type;
  destProbe.speed = 60.0;
  destProbe.timestamp = dest.arriveByUnix;

  const GraphPosition goalPos = matchVehicleToGraphIndexed(store, matchIndex, destProbe);
  if (!goalPos.valid) {
    return results;
  }

  std::unordered_set<int64_t> allowedEdges;
  std::string collectErr;
  double spanM = 0.0;
  for (const auto& vehicle : vehicles) {
    spanM = std::max(spanM, haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon}));
  }
  // Metro fleets on a regional CSR: radius-limited reverse field beats mmap corridor unions.
  const bool radiusOnly = spanM < 200000.0;
  const std::unordered_set<int64_t>* allowedPtr = nullptr;
  if (!radiusOnly) {
    const bool useTightCorridors = spanM < 220000.0;
    const bool collected =
        useTightCorridors
            ? collectDestinationCorridorEdgeIdsIndexed(store, matchIndex, vehicles, dest.lat,
                                                       dest.lon, corridorWidthM, allowedEdges,
                                                       &collectErr)
            : collectDestinationCorridorBboxEdgeIdsIndexed(store, matchIndex, vehicles, dest.lat,
                                                           dest.lon, corridorWidthM, allowedEdges,
                                                           &collectErr);
    if (!collected || allowedEdges.empty()) {
      return results;
    }
    if (useTightCorridors) {
      pruneDestinationEdgeIdsToCorridors(store, vehicles, dest.lat, dest.lon, corridorWidthM,
                                         allowedEdges);
    }
    if (allowedEdges.empty()) {
      return results;
    }
    allowedPtr = &allowedEdges;
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
    if (!mayReachWithSpeed(vehicle, dest, speedMs)) {
      continue;
    }
    const double maxHorizon =
        std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
    if (maxHorizon < 1.0) {
      continue;
    }
    jobs.push_back({&vehicle, hist, speedMs, maxHorizon});
  }
  if (jobs.empty()) {
    return results;
  }

  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;

  double groupSpeedMs = jobs.front().speedMs;
  double groupMaxHorizon = 0.0;
  double maxRadiusM = radiusOnly ? spanM + 15000.0 : 25000.0;
  const LatLon goalLl{dest.lat, dest.lon};
  std::unordered_set<int64_t> targetNodes;
  targetNodes.reserve(jobs.size() * 2);
  for (const auto& job : jobs) {
    groupSpeedMs = std::min(groupSpeedMs, job.speedMs);
    groupMaxHorizon = std::max(groupMaxHorizon, job.maxHorizon);
    if (!radiusOnly) {
      maxRadiusM = std::max(
          maxRadiusM, haversineMeters({job.vehicle->lat, job.vehicle->lon}, goalLl) + 12000.0);
    }
    const GraphPosition startPos = matchVehicleToGraphIndexed(store, matchIndex, *job.vehicle);
    addRoutingTargetNodesIndexed(store, startPos, targetNodes);
  }

  const CsrGraph& csr = store.csr();
  const RoutedTimeField field = computeRoutedTimeFieldFromGoalCsr(
      store, csr, goalPos, groupSpeedMs, dest.type, routeParam, groupMaxHorizon, allowedPtr,
      &targetNodes, &goalLl, maxRadiusM);

  std::vector<std::optional<VehicleArrivalResult>> rows(jobs.size());
  parallelFor(jobs.size(), [&](std::size_t i) {
    const VehicleJob& job = jobs[i];
    const GraphPosition startPos = matchVehicleToGraphIndexed(store, matchIndex, *job.vehicle);
    if (!startPos.valid) {
      return;
    }
    const RouteToGoal path = routeFromRoutedFieldCsr(store, csr, field, startPos, goalPos,
                                                     job.speedMs, job.vehicle->type, routeParam);
    const double travel = path.travelTimeSec;
    if (travel >= kInfTime / 2.0 || travel > job.maxHorizon + 1e-6) {
      return;
    }
    const double eta = static_cast<double>(job.vehicle->timestamp) + travel;
    if (eta > static_cast<double>(dest.arriveByUnix) + 1e-6) {
      return;
    }
    VehicleArrivalResult row;
    row.vehicleId = job.vehicle->id;
    row.reachable = true;
    row.travelDurationSec = travel;
    row.etaUnix = eta;
    row.routeDistanceM = polylineLengthMeters(path.polyline);
    row.route = path.polyline;
    simplifyRoutePolyline(row.route, 120);
    rows[i] = std::move(row);
  });

  results.reserve(jobs.size());
  for (auto& row : rows) {
    if (row) {
      results.push_back(std::move(*row));
    }
  }
  return results;
}

void appendVehicleResults(std::vector<VehicleArrivalResult>& out,
                          std::vector<VehicleArrivalResult>&& rows) {
  out.insert(out.end(), std::make_move_iterator(rows.begin()), std::make_move_iterator(rows.end()));
}

}  // namespace

bool vehicleMayReachDestination(const VehicleInfo& vehicle, const DestinationQuery& dest,
                                const VehicleHistory* history, const PredictParam& param) {
  if (vehicle.type != dest.type) {
    return false;
  }
  const double horizon =
      std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
  if (horizon < 1.0) {
    return false;
  }
  const double speedMs =
      speedMsFromKmh(resolveSpeedKmh(vehicle, history, nullptr, vehicle.type));
  const double dist =
      haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
  const double minTravelSec = dist / (std::max(speedMs, 5.0) * 0.65);
  return minTravelSec <= horizon + 120.0;
}

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
  // On-time first, then late (still have routes); within each group apply sortBy.
  std::sort(rows.begin(), rows.end(),
            [sortBy](const VehicleArrivalResult& a, const VehicleArrivalResult& b) {
              if (a.reachable != b.reachable) {
                return a.reachable && !b.reachable;
              }
              switch (sortBy) {
                case ArrivalSortBy::ETA:
                  return a.etaUnix < b.etaUnix;
                case ArrivalSortBy::DISTANCE:
                  return a.routeDistanceM < b.routeDistanceM;
                default:
                  return a.travelDurationSec < b.travelDurationSec;
              }
            });
}

constexpr double kChMinDistM = 50000.0;

int64_t nodeFromPosition(const GraphFileStore& store, const GraphPosition& pos, double refLat,
                         double refLon) {
  if (!pos.valid) {
    return 0;
  }
  if (pos.edgeId == 0) {
    return pos.nodeId;
  }
  int64_t from = 0;
  int64_t to = 0;
  if (!store.edgeEndpoints(pos.edgeId, from, to)) {
    return 0;
  }
  double flat = 0.0;
  double flon = 0.0;
  double tlat = 0.0;
  double tlon = 0.0;
  if (!store.edgeEndpointLatLon(pos.edgeId, flat, flon, tlat, tlon)) {
    return from;
  }
  return haversineMeters({refLat, refLon}, {flat, flon}) <=
                 haversineMeters({refLat, refLon}, {tlat, tlon})
             ? from
             : to;
}

// Fast hwy portal seeds from spatial snap only (no CSR walks).
std::vector<int64_t> snapHwyPortalSeeds(const GraphFileStore& snapStore,
                                        const GraphFileStore& chStore, const CsrGraph& hwyCsr,
                                        const SpatialIndex& index, double lat, double lon,
                                        VehicleType type, std::size_t maxNodes = 4) {
  std::vector<int64_t> out;
  VehicleInfo probe;
  probe.id = "probe";
  probe.lat = lat;
  probe.lon = lon;
  probe.type = type;
  probe.speed = 60.0;
  probe.timestamp = 0;

  const ChGraph& ch = chStore.ch();
  auto tryNode = [&](int64_t nodeId) {
    if (nodeId == 0 || ch.nodeIndex(nodeId) < 0 || hwyCsr.nodeRow(chStore, nodeId) < 0) {
      return;
    }
    if (std::find(out.begin(), out.end(), nodeId) == out.end()) {
      out.push_back(nodeId);
    }
  };

  const GraphPosition pos = matchVehicleToGraphIndexed(snapStore, index, probe);
  tryNode(nodeFromPosition(snapStore, pos, lat, lon));
  if (pos.valid && pos.edgeId != 0) {
    int64_t from = 0;
    int64_t to = 0;
    if (snapStore.edgeEndpoints(pos.edgeId, from, to)) {
      tryNode(from);
      tryNode(to);
    }
  }
  if (out.size() > maxNodes) {
    out.resize(maxNodes);
  }
  return out;
}

// Nearest highway-overlay nodes near a point via spatial index (no regional CSR walk).
std::vector<int64_t> findNearestHwyOverlayNodes(const GraphFileStore& snapStore,
                                                const GraphFileStore& chStore,
                                                const CsrGraph& hwyCsr, const ChGraph& ch,
                                                const SpatialIndex& index, double lat, double lon,
                                                double radiusM, std::size_t maxNodes) {
  std::vector<int64_t> out;
  struct Candidate {
    int64_t node = 0;
    double distM = 0.0;
  };
  std::vector<Candidate> candidates;

  const double dLat = radiusM / 111320.0;
  const double cosLat = std::cos(lat * 3.141592653589793 / 180.0);
  const double dLon = cosLat > 1e-6 ? radiusM / (111320.0 * cosLat) : radiusM / 111320.0;
  GeoBBox box{lon - dLon, lat - dLat, lon + dLon, lat + dLat};
  std::unordered_set<int64_t> edgeIds;
  index.collectEdgesInBBox(box, edgeIds);

  for (int64_t edgeId : edgeIds) {
    int64_t from = 0;
    int64_t to = 0;
    if (!snapStore.edgeEndpoints(edgeId, from, to)) {
      continue;
    }
    for (int64_t nodeId : {from, to}) {
      if (nodeId == 0 || ch.nodeIndex(nodeId) < 0 || hwyCsr.nodeRow(chStore, nodeId) < 0) {
        continue;
      }
      double nlat = 0.0;
      double nlon = 0.0;
      if (!chStore.nodeLatLon(nodeId, nlat, nlon)) {
        continue;
      }
      const double d = haversineMeters({lat, lon}, {nlat, nlon});
      if (d <= radiusM) {
        candidates.push_back({nodeId, d});
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.distM < b.distM; });
  for (const auto& c : candidates) {
    if (out.size() >= maxNodes) {
      break;
    }
    if (std::find(out.begin(), out.end(), c.node) == out.end()) {
      out.push_back(c.node);
    }
  }
  return out;
}

// Nearest Full-CH road nodes via spatial index (any road edge, not only hwy overlay).
// Used when edge snap fails (click >kMaxSnapDistanceMeters from a road) so we still
// route to the nearest on-graph point instead of falling back to empty-route geodesic.
std::vector<int64_t> findNearestFullChRoadNodes(const GraphFileStore& store, const FullChGraph& ch,
                                                const SpatialIndex& index, double lat, double lon,
                                                VehicleType type, double radiusM,
                                                std::size_t maxNodes) {
  std::vector<int64_t> out;
  struct Candidate {
    int64_t node = 0;
    double distM = 0.0;
  };
  std::vector<Candidate> candidates;

  const double dLat = radiusM / 111320.0;
  const double cosLat = std::cos(lat * 3.141592653589793 / 180.0);
  const double dLon = cosLat > 1e-6 ? radiusM / (111320.0 * cosLat) : radiusM / 111320.0;
  GeoBBox box{lon - dLon, lat - dLat, lon + dLon, lat + dLat};
  std::unordered_set<int64_t> edgeIds;
  index.collectEdgesInBBox(box, edgeIds);

  for (int64_t edgeId : edgeIds) {
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
    int64_t from = 0;
    int64_t to = 0;
    if (!store.edgeEndpoints(edgeId, from, to)) {
      continue;
    }
    for (int64_t nodeId : {from, to}) {
      if (nodeId == 0 || ch.nodeIndex(nodeId) < 0) {
        continue;
      }
      double nlat = 0.0;
      double nlon = 0.0;
      if (!store.nodeLatLon(nodeId, nlat, nlon)) {
        continue;
      }
      const double d = haversineMeters({lat, lon}, {nlat, nlon});
      if (d <= radiusM) {
        candidates.push_back({nodeId, d});
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.distM < b.distM; });
  for (const auto& c : candidates) {
    if (out.size() >= maxNodes) {
      break;
    }
    if (std::find(out.begin(), out.end(), c.node) == out.end()) {
      out.push_back(c.node);
    }
  }
  return out;
}

std::vector<int64_t> collectOverlayChNodes(const GraphFileStore& snapStore,
                                           const GraphFileStore& chStore, const ChGraph& ch,
                                           const CsrGraph& walkCsr, const SpatialIndex& index,
                                           double lat, double lon, VehicleType type,
                                           double maxWalkM = 20000.0, std::size_t maxNodes = 8) {
  std::vector<int64_t> out;
  VehicleInfo probe;
  probe.id = "probe";
  probe.lat = lat;
  probe.lon = lon;
  probe.type = type;
  probe.speed = 60.0;
  probe.timestamp = 0;

  const GraphPosition pos = matchVehicleToGraphIndexed(snapStore, index, probe);
  const int64_t seed = nodeFromPosition(snapStore, pos, lat, lon);

  std::vector<int64_t> seeds;
  if (seed != 0) {
    seeds.push_back(seed);
  }
  if (pos.valid && pos.edgeId != 0) {
    int64_t from = 0;
    int64_t to = 0;
    if (snapStore.edgeEndpoints(pos.edgeId, from, to)) {
      if (from != 0) {
        seeds.push_back(from);
      }
      if (to != 0) {
        seeds.push_back(to);
      }
    }
  }

  struct Candidate {
    int64_t node = 0;
    double distM = 0.0;
    int degree = 0;
  };
  std::vector<Candidate> candidates;

  const CsrGraph& hwyCsr = chStore.hwyCsr();
  std::vector<int64_t> hwySeeds;
  hwySeeds.reserve(seeds.size());

  struct QItem {
    int64_t node = 0;
    double distM = 0.0;
  };

  // Phase 1: walk regional/full road CSR to reach nodes on the highway overlay.
  if (&walkCsr != &hwyCsr) {
    std::deque<QItem> roadQueue;
    std::unordered_set<int64_t> roadSeen;
    for (int64_t candidate : seeds) {
      if (candidate == 0 || roadSeen.count(candidate) != 0) {
        continue;
      }
      if (walkCsr.nodeRow(snapStore, candidate) < 0) {
        continue;
      }
      roadSeen.insert(candidate);
      roadQueue.push_back({candidate, 0.0});
    }
    while (!roadQueue.empty()) {
      const QItem item = roadQueue.front();
      roadQueue.pop_front();
      if (hwyCsr.nodeRow(chStore, item.node) >= 0) {
        hwySeeds.push_back(item.node);
      }
      if (item.distM > maxWalkM) {
        continue;
      }
      walkCsr.forEachNeighbor(snapStore, item.node, nullptr, [&](const CsrArc& arc) {
        if (type == VehicleType::TRUCK && arc.type != EdgeType::ROAD) {
          return;
        }
        if (roadSeen.count(arc.toNodeId) != 0) {
          return;
        }
        roadSeen.insert(arc.toNodeId);
        roadQueue.push_back({arc.toNodeId, item.distM + static_cast<double>(arc.length)});
      });
    }
  } else {
    for (int64_t candidate : seeds) {
      if (candidate != 0 && hwyCsr.nodeRow(chStore, candidate) >= 0) {
        hwySeeds.push_back(candidate);
      }
    }
  }

  std::sort(hwySeeds.begin(), hwySeeds.end());
  hwySeeds.erase(std::unique(hwySeeds.begin(), hwySeeds.end()), hwySeeds.end());
  if (hwySeeds.empty()) {
    return out;
  }

  // Phase 2: walk highway CSR only — candidates are connected on the CH overlay graph.
  std::deque<QItem> queue;
  std::unordered_set<int64_t> seen;
  for (int64_t candidate : hwySeeds) {
    if (seen.count(candidate) != 0) {
      continue;
    }
    seen.insert(candidate);
    queue.push_back({candidate, 0.0});
  }

  while (!queue.empty()) {
    const QItem item = queue.front();
    queue.pop_front();
    if (ch.nodeIndex(item.node) >= 0) {
      int degree = 0;
      hwyCsr.forEachNeighbor(chStore, item.node, nullptr,
                             [&](const CsrArc& arc) { (void)arc; ++degree; });
      double nlat = 0.0;
      double nlon = 0.0;
      if (chStore.nodeLatLon(item.node, nlat, nlon)) {
        candidates.push_back(
            {item.node, haversineMeters({lat, lon}, {nlat, nlon}), degree});
      }
    }
    if (item.distM > maxWalkM) {
      continue;
    }
    hwyCsr.forEachNeighbor(chStore, item.node, nullptr, [&](const CsrArc& arc) {
      if (seen.count(arc.toNodeId) != 0) {
        return;
      }
      seen.insert(arc.toNodeId);
      queue.push_back({arc.toNodeId, item.distM + static_cast<double>(arc.length)});
    });
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
    if (a.degree >= 2 && b.degree < 2) {
      return true;
    }
    if (b.degree >= 2 && a.degree < 2) {
      return false;
    }
    return a.distM < b.distM;
  });
  out.reserve(std::min(maxNodes, candidates.size()));
  for (const auto& c : candidates) {
    if (out.size() >= maxNodes) {
      break;
    }
    out.push_back(c.node);
  }
  return out;
}

// BFS backward on the highway overlay from dest portals until CH nodes near the vehicle are found.
std::vector<int64_t> collectOverlayChNodesToward(const GraphFileStore& chStore, const ChGraph& ch,
                                                 const CsrGraph& hwyCsr, double lat, double lon,
                                                 const std::vector<int64_t>& destHwySeeds,
                                                 double geoMaxM = 80000.0,
                                                 std::size_t maxNodes = 16) {
  std::vector<int64_t> out;
  if (destHwySeeds.empty()) {
    return out;
  }

  struct Candidate {
    int64_t node = 0;
    double distM = 0.0;
    int degree = 0;
  };
  std::vector<Candidate> candidates;

  struct QItem {
    int64_t node = 0;
    double hwyDistM = 0.0;
  };
  std::deque<QItem> queue;
  std::unordered_set<int64_t> seen;
  for (int64_t seed : destHwySeeds) {
    if (seed == 0 || hwyCsr.nodeRow(chStore, seed) < 0 || seen.count(seed) != 0) {
      continue;
    }
    seen.insert(seed);
    queue.push_back({seed, 0.0});
  }

  const double stopHwyM = geoMaxM * 3.0;
  while (!queue.empty()) {
    const QItem item = queue.front();
    queue.pop_front();
    if (ch.nodeIndex(item.node) >= 0) {
      double nlat = 0.0;
      double nlon = 0.0;
      if (chStore.nodeLatLon(item.node, nlat, nlon)) {
        const double geoM = haversineMeters({lat, lon}, {nlat, nlon});
        if (geoM <= geoMaxM) {
          int degree = 0;
          hwyCsr.forEachNeighbor(chStore, item.node, nullptr,
                                 [&](const CsrArc& arc) { (void)arc; ++degree; });
          candidates.push_back({item.node, geoM, degree});
        }
      }
    }
    if (item.hwyDistM > stopHwyM) {
      continue;
    }
    hwyCsr.forEachNeighbor(chStore, item.node, nullptr, [&](const CsrArc& arc) {
      if (seen.count(arc.toNodeId) != 0) {
        return;
      }
      seen.insert(arc.toNodeId);
      queue.push_back({arc.toNodeId, item.hwyDistM + static_cast<double>(arc.length)});
    });
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
    if (a.degree >= 2 && b.degree < 2) {
      return true;
    }
    if (b.degree >= 2 && a.degree < 2) {
      return false;
    }
    return a.distM < b.distM;
  });
  out.reserve(std::min(maxNodes, candidates.size()));
  for (const auto& c : candidates) {
    if (out.size() >= maxNodes) {
      break;
    }
    out.push_back(c.node);
  }
  return out;
}

// One goal-ward BFS shared across all vehicles (replaces N parallel toward walks).
std::vector<std::vector<int64_t>> collectOverlayChNodesTowardBatch(
    const GraphFileStore& chStore, const ChGraph& ch, const CsrGraph& hwyCsr,
    const std::vector<LatLon>& vehicleLl, const std::vector<int64_t>& destHwySeeds,
    const std::vector<double>& geoMaxPerVehicle, std::size_t maxNodesPerVehicle = 8) {
  const std::size_t n = vehicleLl.size();
  std::vector<std::vector<int64_t>> out(n);
  if (n == 0 || destHwySeeds.empty() || geoMaxPerVehicle.size() != n) {
    return out;
  }

  struct Candidate {
    int64_t node = 0;
    double distM = 0.0;
    int degree = 0;
  };
  std::vector<std::vector<Candidate>> candidates(n);

  struct QItem {
    int64_t node = 0;
    double hwyDistM = 0.0;
  };
  std::deque<QItem> queue;
  std::unordered_set<int64_t> seen;
  for (int64_t seed : destHwySeeds) {
    if (seed == 0 || hwyCsr.nodeRow(chStore, seed) < 0 || seen.count(seed) != 0) {
      continue;
    }
    seen.insert(seed);
    queue.push_back({seed, 0.0});
  }

  double maxGeoM = 0.0;
  for (double g : geoMaxPerVehicle) {
    maxGeoM = std::max(maxGeoM, g);
  }
  const double stopHwyM = std::min(maxGeoM * 3.0, 400000.0);
  std::unordered_map<int64_t, int> nodeDegree;
  nodeDegree.reserve(4096);
  auto vehicleSatisfied = [&]() {
    for (std::size_t i = 0; i < n; ++i) {
      if (candidates[i].empty()) {
        return false;
      }
    }
    return true;
  };

  while (!queue.empty()) {
    if (vehicleSatisfied()) {
      break;
    }
    const QItem item = queue.front();
    queue.pop_front();
    if (ch.nodeIndex(item.node) >= 0) {
      double nlat = 0.0;
      double nlon = 0.0;
      if (chStore.nodeLatLon(item.node, nlat, nlon)) {
        int degree = 0;
        const auto degIt = nodeDegree.find(item.node);
        if (degIt != nodeDegree.end()) {
          degree = degIt->second;
        } else {
          hwyCsr.forEachNeighbor(chStore, item.node, nullptr,
                                 [&](const CsrArc& arc) { (void)arc; ++degree; });
          nodeDegree[item.node] = degree;
        }
        for (std::size_t vi = 0; vi < n; ++vi) {
          if (candidates[vi].size() >= maxNodesPerVehicle) {
            continue;
          }
          const double geoM = haversineMeters(vehicleLl[vi], {nlat, nlon});
          if (geoM <= geoMaxPerVehicle[vi]) {
            candidates[vi].push_back({item.node, geoM, degree});
          }
        }
      }
    }
    if (item.hwyDistM > stopHwyM) {
      continue;
    }
    hwyCsr.forEachNeighbor(chStore, item.node, nullptr, [&](const CsrArc& arc) {
      if (seen.count(arc.toNodeId) != 0) {
        return;
      }
      seen.insert(arc.toNodeId);
      queue.push_back({arc.toNodeId, item.hwyDistM + static_cast<double>(arc.length)});
    });
  }

  for (std::size_t vi = 0; vi < n; ++vi) {
    auto& list = candidates[vi];
    std::sort(list.begin(), list.end(), [](const Candidate& a, const Candidate& b) {
      if (a.degree >= 2 && b.degree < 2) {
        return true;
      }
      if (b.degree >= 2 && a.degree < 2) {
        return false;
      }
      return a.distM < b.distM;
    });
    out[vi].reserve(std::min(maxNodesPerVehicle, list.size()));
    for (const auto& c : list) {
      if (out[vi].size() >= maxNodesPerVehicle) {
        break;
      }
      out[vi].push_back(c.node);
    }
  }
  return out;
}

std::unordered_set<int64_t> reachableOnCsr(const GraphFileStore& store, const CsrGraph& csr,
                                           const std::vector<int64_t>& seeds) {
  std::unordered_set<int64_t> seen;
  std::deque<int64_t> queue;
  for (int64_t seed : seeds) {
    if (seed == 0 || csr.nodeRow(store, seed) < 0 || seen.count(seed) != 0) {
      continue;
    }
    seen.insert(seed);
    queue.push_back(seed);
  }
  while (!queue.empty()) {
    const int64_t u = queue.front();
    queue.pop_front();
    csr.forEachNeighbor(store, u, nullptr, [&](const CsrArc& arc) {
      if (seen.count(arc.toNodeId) == 0) {
        seen.insert(arc.toNodeId);
        queue.push_back(arc.toNodeId);
      }
    });
  }
  return seen;
}

bool csrSeedsConnected(const GraphFileStore& store, const CsrGraph& csr,
                       const std::vector<int64_t>& seedsA,
                       const std::vector<int64_t>& seedsB,
                       std::size_t maxExpansions = 250000) {
  std::unordered_set<int64_t> sideA;
  std::unordered_set<int64_t> sideB;
  std::deque<int64_t> qa;
  std::deque<int64_t> qb;
  for (int64_t seed : seedsA) {
    if (seed == 0 || csr.nodeRow(store, seed) < 0 || sideA.count(seed) != 0) {
      continue;
    }
    sideA.insert(seed);
    qa.push_back(seed);
  }
  for (int64_t seed : seedsB) {
    if (seed == 0 || csr.nodeRow(store, seed) < 0 || sideB.count(seed) != 0) {
      continue;
    }
    sideB.insert(seed);
    qb.push_back(seed);
  }
  if (sideA.empty() || sideB.empty()) {
    return false;
  }
  for (int64_t seed : seedsA) {
    if (sideB.count(seed) != 0) {
      return true;
    }
  }

  std::size_t expansions = 0;
  while ((!qa.empty() || !qb.empty()) && expansions < maxExpansions) {
    if (!qa.empty() && (qb.empty() || qa.size() <= qb.size())) {
      const int64_t u = qa.front();
      qa.pop_front();
      bool found = false;
      csr.forEachNeighbor(store, u, nullptr, [&](const CsrArc& arc) {
        if (found) {
          return;
        }
        if (sideB.count(arc.toNodeId) != 0) {
          found = true;
          return;
        }
        if (sideA.insert(arc.toNodeId).second) {
          qa.push_back(arc.toNodeId);
        }
      });
      if (found) {
        return true;
      }
    } else if (!qb.empty()) {
      const int64_t u = qb.front();
      qb.pop_front();
      bool found = false;
      csr.forEachNeighbor(store, u, nullptr, [&](const CsrArc& arc) {
        if (found) {
          return;
        }
        if (sideA.count(arc.toNodeId) != 0) {
          found = true;
          return;
        }
        if (sideB.insert(arc.toNodeId).second) {
          qb.push_back(arc.toNodeId);
        }
      });
      if (found) {
        return true;
      }
    }
    ++expansions;
  }
  return false;
}

int64_t nearestOverlayChNode(const GraphFileStore& snapStore, const GraphFileStore& chStore,
                             const ChGraph& ch, const CsrGraph& walkCsr, const SpatialIndex& index,
                             double lat, double lon, VehicleType type, double maxWalkM = 15000.0) {
  VehicleInfo probe;
  probe.id = "probe";
  probe.lat = lat;
  probe.lon = lon;
  probe.type = type;
  probe.speed = 60.0;
  probe.timestamp = 0;

  const GraphPosition pos = matchVehicleToGraphIndexed(snapStore, index, probe);
  const int64_t seed = nodeFromPosition(snapStore, pos, lat, lon);

  std::vector<int64_t> seeds;
  if (seed != 0) {
    seeds.push_back(seed);
  }
  if (pos.valid && pos.edgeId != 0) {
    int64_t from = 0;
    int64_t to = 0;
    if (snapStore.edgeEndpoints(pos.edgeId, from, to)) {
      if (from != 0) {
        seeds.push_back(from);
      }
      if (to != 0) {
        seeds.push_back(to);
      }
    }
  }

  for (int64_t candidate : seeds) {
    if (candidate != 0 && ch.nodeIndex(candidate) >= 0) {
      return candidate;
    }
  }

  if (seeds.empty()) {
    return 0;
  }

  struct QItem {
    int64_t node = 0;
    double distM = 0.0;
  };
  std::deque<QItem> queue;
  std::unordered_set<int64_t> seen;
  for (int64_t candidate : seeds) {
    if (candidate == 0 || seen.count(candidate) != 0) {
      continue;
    }
    if (walkCsr.nodeRow(snapStore, candidate) < 0) {
      continue;
    }
    seen.insert(candidate);
    queue.push_back({candidate, 0.0});
  }
  if (queue.empty()) {
    return 0;
  }

  int64_t best = 0;
  double bestDist = maxWalkM + 1.0;

  while (!queue.empty()) {
    const QItem item = queue.front();
    queue.pop_front();
    if (ch.nodeIndex(item.node) >= 0) {
      double nlat = 0.0;
      double nlon = 0.0;
      if (chStore.nodeLatLon(item.node, nlat, nlon)) {
        const double d = haversineMeters({lat, lon}, {nlat, nlon});
        if (d < bestDist) {
          bestDist = d;
          best = item.node;
        }
      }
    }
    if (item.distM > maxWalkM) {
      continue;
    }

    walkCsr.forEachNeighbor(snapStore, item.node, nullptr, [&](const CsrArc& arc) {
      if (type == VehicleType::TRUCK && arc.type != EdgeType::ROAD) {
        return;
      }
      if (seen.count(arc.toNodeId) != 0) {
        return;
      }
      seen.insert(arc.toNodeId);
      queue.push_back({arc.toNodeId, item.distM + static_cast<double>(arc.length)});
    });
  }
  return best;
}

int64_t chNodeForPosition(const GraphFileStore& store, const ChGraph& ch,
                          const GraphPosition& pos, double refLat, double refLon) {
  if (!pos.valid) {
    return 0;
  }
  if (pos.edgeId == 0) {
    return ch.nodeIndex(pos.nodeId) >= 0 ? pos.nodeId : 0;
  }
  int64_t from = 0;
  int64_t to = 0;
  if (!store.edgeEndpoints(pos.edgeId, from, to)) {
    return 0;
  }
  const int iFrom = ch.nodeIndex(from);
  const int iTo = ch.nodeIndex(to);
  if (iFrom >= 0 && iTo >= 0) {
    double flat = 0.0;
    double flon = 0.0;
    double tlat = 0.0;
    double tlon = 0.0;
    if (store.edgeEndpointLatLon(pos.edgeId, flat, flon, tlat, tlon)) {
      const double dFrom = haversineMeters({refLat, refLon}, {flat, flon});
      const double dTo = haversineMeters({refLat, refLon}, {tlat, tlon});
      return dFrom <= dTo ? from : to;
    }
    return from;
  }
  if (iFrom >= 0) {
    return from;
  }
  if (iTo >= 0) {
    return to;
  }
  return 0;
}

std::optional<VehicleArrivalResult> predictVehicleToDestinationCh(
    const GraphFileStore& snapStore, const SpatialIndex& snapIndex,
    const GraphFileStore& chStore, const VehicleInfo& vehicle, const VehicleHistory* history,
    const DestinationQuery& dest, const PredictParam& param) {
  if (!chStore.hasCh() || !chStore.hasHwyCsr() || vehicle.type != dest.type) {
    return std::nullopt;
  }

  const double distM = haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
  if (distM < kChMinDistM) {
    return std::nullopt;
  }

  const double maxHorizon =
      std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
  if (maxHorizon < 1.0) {
    return std::nullopt;
  }

  const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, history, nullptr, vehicle.type));
  if (!mayReachWithSpeed(vehicle, dest, speedMs)) {
    return std::nullopt;
  }

  VehicleInfo destProbe;
  destProbe.id = "destination";
  destProbe.lat = dest.lat;
  destProbe.lon = dest.lon;
  destProbe.type = dest.type;
  destProbe.speed = 60.0;
  destProbe.timestamp = dest.arriveByUnix;

  const GraphPosition startPos = matchVehicleToGraphIndexed(snapStore, snapIndex, vehicle);
  const GraphPosition goalPos = matchVehicleToGraphIndexed(snapStore, snapIndex, destProbe);
  if (!startPos.valid || !goalPos.valid) {
    return std::nullopt;
  }

  const CsrGraph& walkCsr = snapStore.hasCsr() ? snapStore.csr() : chStore.hwyCsr();
  const CsrGraph& hwyCsr = chStore.hwyCsr();
  std::vector<int64_t> toNodes =
      collectOverlayChNodes(snapStore, chStore, chStore.ch(), walkCsr, snapIndex, dest.lat,
                            dest.lon, dest.type, 50000.0, 16);
  if (toNodes.empty()) {
    if (std::getenv("MMLP_DEBUG_CH")) {
      std::cerr << "[mmlp] ch snap empty to=0 veh=" << vehicle.id << "\n" << std::flush;
    }
    return std::nullopt;
  }
  const double geoMaxM =
      std::max(80000.0, haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon}) + 20000.0);
  std::vector<int64_t> fromNodes = collectOverlayChNodes(
      snapStore, chStore, chStore.ch(), walkCsr, snapIndex, vehicle.lat, vehicle.lon, vehicle.type,
      50000.0, 16);
  if (fromNodes.empty()) {
    fromNodes = collectOverlayChNodesToward(chStore, chStore.ch(), hwyCsr, vehicle.lat, vehicle.lon,
                                          toNodes, geoMaxM, 16);
  }
  if (fromNodes.empty()) {
    fromNodes = collectOverlayChNodes(snapStore, chStore, chStore.ch(), walkCsr, snapIndex,
                                      vehicle.lat, vehicle.lon, vehicle.type, 50000.0, 32);
    if (fromNodes.empty()) {
      fromNodes = collectOverlayChNodesToward(chStore, chStore.ch(), hwyCsr, vehicle.lat,
                                            vehicle.lon, toNodes, geoMaxM * 1.5, 32);
    }
  }
  if (fromNodes.empty()) {
    if (std::getenv("MMLP_DEBUG_CH")) {
      std::cerr << "[mmlp] ch snap empty from=0 to=" << toNodes.size() << " veh=" << vehicle.id
                << "\n"
                << std::flush;
    }
    return std::nullopt;
  }

  if (!csrSeedsConnected(chStore, hwyCsr, fromNodes, toNodes)) {
    if (std::getenv("MMLP_DEBUG_CH")) {
      std::cerr << "[mmlp] ch hwy disconnected from=" << fromNodes.size()
                << " to=" << toNodes.size() << " veh=" << vehicle.id << "\n"
                << std::flush;
    }
  }

  if (fromNodes.size() > 4) {
    fromNodes.resize(4);
  }
  if (toNodes.size() > 4) {
    toNodes.resize(4);
  }

  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;
  for (int64_t fromNode : fromNodes) {
    for (int64_t toNode : toNodes) {
      const RouteToGoal path = chStore.ch().query(chStore, chStore.hwyCsr(), fromNode, toNode,
                                                  vehicle.type, routeParam, maxHorizon);
      const double travel = path.travelTimeSec;
      if (std::getenv("MMLP_DEBUG_CH")) {
        std::cerr << "[mmlp] ch try from=" << fromNode << " to=" << toNode
                  << " travel=" << travel << "\n"
                  << std::flush;
      }
      if (travel >= kInfTime / 2.0 || travel > maxHorizon + 1e-6) {
        continue;
      }
      const double eta = static_cast<double>(vehicle.timestamp) + travel;
      if (eta > static_cast<double>(dest.arriveByUnix) + 1e-6) {
        continue;
      }
      VehicleArrivalResult row;
      row.vehicleId = vehicle.id;
      row.reachable = true;
      row.travelDurationSec = travel;
      row.etaUnix = eta;
      const double haulM = haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
      if (auto stitched = stitchChOverlayRoutePolyline(snapStore, snapIndex, vehicle, dest, speedMs,
                                                     maxHorizon, param, path.polyline)) {
        if (routePolylineQualityOk(*stitched) || routePolylineDisplayOk(*stitched, haulM)) {
          row.route = std::move(*stitched);
          row.routeDistanceM = polylineLengthMeters(row.route);
          simplifyRoutePolyline(row.route, 120);
          return row;
        }
      }
      row.routeDistanceM = polylineLengthMeters(path.polyline);
      row.route = path.polyline;
      simplifyRoutePolyline(row.route, 120);
      if (routePolylineQualityOk(row.route) || routePolylineDisplayOk(row.route, haulM)) {
        return row;
      }
      // Last resort for corridor bridges: only accept drawable road shape.
      if (routePolylineDisplayOk(row.route, haulM) && haulM >= 80000.0) {
        return row;
      }
      return std::nullopt;
    }
  }
  if (std::getenv("MMLP_DEBUG_CH")) {
    std::cerr << "[mmlp] ch query miss from_cands=" << fromNodes.size()
              << " to_cands=" << toNodes.size() << " veh=" << vehicle.id << "\n"
              << std::flush;
  }
  return std::nullopt;
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
  if (!mayReachWithSpeed(vehicle, dest, speedMs)) {
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

std::optional<VehicleArrivalResult> predictVehicleToDestinationIndexedOrCh(
    const GraphFileStore& store, const SpatialIndex& matchIndex, const GraphFileStore& chStore,
    const VehicleInfo& vehicle, const VehicleHistory* history, const DestinationQuery& dest,
    double maxCorridorWidthM, const PredictParam& param) {
  if (auto chRow = predictVehicleToDestinationCh(store, matchIndex, chStore, vehicle, history,
                                                 dest, param)) {
    return chRow;
  }

  const double dist = haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
  const double corridorW = dist > 40000.0 ? std::min(maxCorridorWidthM, 22000.0)
                                          : std::min(maxCorridorWidthM, dist * 0.35 + 12000.0);
  GraphContext corridorCtx;
  std::string corridorErr;
  if (!extractGraphContextForPairIndexed(store, matchIndex, vehicle, dest.lat, dest.lon, corridorW,
                                         corridorCtx, &corridorErr)) {
    return std::nullopt;
  }
  VehicleInfo destProbe;
  destProbe.id = "destination";
  destProbe.lat = dest.lat;
  destProbe.lon = dest.lon;
  destProbe.type = dest.type;
  destProbe.speed = 60.0;
  destProbe.timestamp = dest.arriveByUnix;
  const GraphPosition goalOnCorridor =
      matchVehicleToGraph(corridorCtx.graph, destProbe, &matchIndex);
  if (!goalOnCorridor.valid) {
    return std::nullopt;
  }
  return predictVehicleToDestination(vehicle, history, corridorCtx, matchIndex, dest,
                                     goalOnCorridor, param);
}

namespace {

constexpr double kPortalReuseDistM = 1000.0;
constexpr double kMaxPlausibleSpeedMs = 50.0;

struct VehiclePortalCacheEntry {
  double lat = 0.0;
  double lon = 0.0;
  int64_t timestamp = 0;
  std::vector<int64_t> portals;
};

std::mutex vehiclePortalCacheMu;
std::unordered_map<std::string, VehiclePortalCacheEntry> vehiclePortalCache;

bool tryReuseVehiclePortals(const VehicleInfo& vehicle, std::vector<int64_t>& portals) {
  std::lock_guard<std::mutex> lock(vehiclePortalCacheMu);
  const auto it = vehiclePortalCache.find(vehicle.id);
  if (it == vehiclePortalCache.end()) {
    return false;
  }
  const VehiclePortalCacheEntry& entry = it->second;
  const double distM = haversineMeters({vehicle.lat, vehicle.lon}, {entry.lat, entry.lon});
  if (vehicle.timestamp > entry.timestamp) {
    const double dt = static_cast<double>(vehicle.timestamp - entry.timestamp);
    if (distM > kMaxPlausibleSpeedMs * dt * 1.5 + 500.0) {
      return false;
    }
  }
  if (distM > kPortalReuseDistM || entry.portals.empty()) {
    return false;
  }
  portals = entry.portals;
  return true;
}

void storeVehiclePortals(const VehicleInfo& vehicle, const std::vector<int64_t>& portals) {
  if (portals.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(vehiclePortalCacheMu);
  vehiclePortalCache[vehicle.id] = {vehicle.lat, vehicle.lon, vehicle.timestamp, portals};
}

uint64_t destRouteCacheKey(const DestinationQuery& dest, double corridorWidthM) {
  const int64_t latQ = static_cast<int64_t>(std::llround(dest.lat * 1000.0));
  const int64_t lonQ = static_cast<int64_t>(std::llround(dest.lon * 1000.0));
  uint64_t key = static_cast<uint64_t>(latQ) * 2654435761u;
  key ^= static_cast<uint64_t>(lonQ) * 1597334677u;
  key ^= static_cast<uint64_t>(static_cast<int>(dest.type)) << 56;
  key ^= static_cast<uint64_t>(static_cast<int>(corridorWidthM / 1000.0)) << 48;
  return key;
}

struct CachedDestRouteField {
  uint64_t key = 0;
  GraphPosition goalPos;
  RoutedTimeField field;
  std::shared_ptr<std::unordered_set<int64_t>> allowedEdges;
  double maxRadiusM = 0.0;
};

std::shared_mutex destRouteFieldCacheMu;
std::shared_ptr<CachedDestRouteField> destRouteFieldCache;

struct CachedHwyDistField {
  uint64_t key = 0;
  int64_t goalNodeId = 0;
  std::vector<double> distByRow;
  std::vector<int64_t> parentNodeByRow;
  std::vector<int64_t> parentEdgeByRow;
  RoutePolyline sharedEgress;
};

std::shared_mutex hwyDistFieldCacheMu;
std::shared_ptr<CachedHwyDistField> hwyDistFieldCache;

struct CachedLocalDestField {
  uint64_t key = 0;
  GraphPosition goalPos;
  RoutedTimeField field;
};

std::shared_mutex localDestFieldCacheMu;
std::shared_ptr<CachedLocalDestField> localDestFieldCache;

struct CachedMetroRegionalField {
  uint64_t key = 0;
  GraphPosition goalPos;
  int64_t goalNodeId = 0;
  std::vector<double> distByRow;
  std::vector<int64_t> parentNodeByRow;
  std::vector<int64_t> parentEdgeByRow;
  double maxRadiusM = 0.0;
};

std::shared_mutex metroRegionalFieldCacheMu;
std::shared_ptr<CachedMetroRegionalField> metroRegionalFieldCache;

uint64_t metroRegionalFieldKey(const DestinationQuery& dest, double speedMs, double maxRadiusM) {
  const int speedBucket = static_cast<int>(std::floor(speedMs * 3.6 / 10.0));
  return destRouteCacheKey(dest, maxRadiusM) ^
         (static_cast<uint64_t>(speedBucket) << 32);
}

double bucketMetroRadiusM(double spanM) {
  double r = std::min(150000.0, std::max(80000.0, spanM + 25000.0));
  return std::ceil(r / 10000.0) * 10000.0;
}

int64_t resolveDenseStartNode(const GraphFileStore& store, const CsrGraph& csr,
                              const std::vector<double>& distByRow, const GraphPosition& start,
                              double speedMs, VehicleType type, const PredictParam& param) {
  if (!start.valid || distByRow.empty()) {
    return 0;
  }
  auto nodeTime = [&](int64_t nodeId) -> double {
    const int row = csr.nodeRow(store, nodeId);
    if (row < 0 || static_cast<std::size_t>(row) >= distByRow.size()) {
      return kInfTime;
    }
    return distByRow[static_cast<std::size_t>(row)];
  };
  if (start.edgeId == 0) {
    return start.nodeId;
  }
  int64_t from = 0;
  int64_t to = 0;
  EdgeType edgeType = EdgeType::ROAD;
  double length = 0.0;
  double speedLimit = 0.0;
  if (!store.edgeEndpoints(start.edgeId, from, to) ||
      !store.readEdge(start.edgeId, edgeType, length, speedLimit)) {
    return 0;
  }
  const double toFrom =
      nodeTime(from) + travelTimeSeconds(start.alongMeters, speedMs, type, param);
  const double toTo =
      nodeTime(to) +
      travelTimeSeconds(std::max(0.0, length - start.alongMeters), speedMs, type, param);
  return (toFrom <= toTo) ? from : to;
}

std::shared_ptr<CachedMetroRegionalField> getOrBuildMetroRegionalField(
    const GraphFileStore& store, const SpatialIndex& matchIndex, const DestinationQuery& dest,
    double groupSpeedMs, double fieldHorizon, double maxRadiusM, const PredictParam& param,
    const std::unordered_set<int64_t>* targetNodes, bool* cacheHitOut = nullptr) {
  const uint64_t key = metroRegionalFieldKey(dest, groupSpeedMs, maxRadiusM);
  {
    std::shared_lock<std::shared_mutex> read(metroRegionalFieldCacheMu);
    if (metroRegionalFieldCache && metroRegionalFieldCache->key == key &&
        !metroRegionalFieldCache->distByRow.empty()) {
      if (cacheHitOut != nullptr) {
        *cacheHitOut = true;
      }
      return metroRegionalFieldCache;
    }
  }
  if (cacheHitOut != nullptr) {
    *cacheHitOut = false;
  }

  const auto tBuild0 = std::chrono::steady_clock::now();
  VehicleInfo destProbe;
  destProbe.id = "destination";
  destProbe.lat = dest.lat;
  destProbe.lon = dest.lon;
  destProbe.type = dest.type;
  destProbe.speed = 60.0;
  destProbe.timestamp = dest.arriveByUnix;
  const GraphPosition goalPos = matchVehicleToGraphIndexed(store, matchIndex, destProbe);
  if (!goalPos.valid) {
    return nullptr;
  }

  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;
  const CsrGraph& csr = store.csr();
  const LatLon goalLl{dest.lat, dest.lon};

  auto cached = std::make_shared<CachedMetroRegionalField>();
  cached->key = key;
  cached->goalPos = goalPos;
  cached->maxRadiusM = maxRadiusM;
  int64_t goalNodeId = 0;
  cached->distByRow = computeRoutedDistFromGoalCsrDense(
      store, csr, goalPos, groupSpeedMs, dest.type, routeParam, fieldHorizon, targetNodes, &goalLl,
      maxRadiusM, &cached->parentNodeByRow, &cached->parentEdgeByRow, &goalNodeId);
  cached->goalNodeId = goalNodeId;

  std::unique_lock<std::shared_mutex> write(metroRegionalFieldCacheMu);
  if (!metroRegionalFieldCache || metroRegionalFieldCache->key != key) {
    metroRegionalFieldCache = cached;
  }
  const auto buildMs =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                          tBuild0)
          .count();
  if (buildMs >= 200) {
    std::cerr << "[mmlp] metro_field build_ms=" << buildMs
              << " targets=" << (targetNodes != nullptr ? targetNodes->size() : 0)
              << " horizon_s=" << static_cast<int>(fieldHorizon) << "\n"
              << std::flush;
  }
  return metroRegionalFieldCache;
}

void appendRoutePt(RoutePolyline& route, const LatLon& p) {
  if (route.points.empty() || haversineMeters(route.points.back(), p) > 1.0) {
    route.points.push_back(p);
  }
}

void appendPolylinePoints(RoutePolyline& route, const RoutePolyline& src) {
  for (const LatLon& p : src.points) {
    appendRoutePt(route, p);
  }
}

double maxPolylineSegmentMeters(const RoutePolyline& route) {
  double maxD = 0.0;
  for (std::size_t i = 1; i < route.points.size(); ++i) {
    maxD = std::max(maxD, haversineMeters(route.points[i - 1], route.points[i]));
  }
  return maxD;
}

bool routePolylineQualityOk(const RoutePolyline& route, double maxSegM) {
  if (route.points.size() < 2) {
    return false;
  }
  const double maxSeg = maxPolylineSegmentMeters(route);
  if (maxSeg <= maxSegM) {
    return true;
  }
  // Reject regional shortcut artifacts (e.g. vehicle -> node -> dest with a 173km jump).
  if (maxSeg > 60000.0) {
    return false;
  }
  if (route.points.size() <= 3 && maxSeg > 25000.0) {
    return false;
  }
  // Sparse CH highway backbone may have moderate gaps when the polyline has shape.
  return route.points.size() >= 4 && maxSeg <= 50000.0;
}

bool routePolylineDisplayOk(const RoutePolyline& route, double haulDistM) {
  if (route.points.size() < 2) {
    return false;
  }
  if (routePolylineQualityOk(route)) {
    return true;
  }
  if (haulDistM < 150000.0) {
    return false;
  }
  const double maxSeg = maxPolylineSegmentMeters(route);
  // National highway CH is a sparse backbone: allow moderate steps, but reject
  // multi-hundred-km chords that look like geodesic lines on the map.
  return route.points.size() >= 8 && maxSeg <= 50000.0;
}

GraphPosition matchLatLonIndexed(const GraphFileStore& store, const SpatialIndex& matchIndex,
                                 double lat, double lon, VehicleType type, int64_t timestamp) {
  VehicleInfo probe;
  probe.id = "__snap__";
  probe.lat = lat;
  probe.lon = lon;
  probe.type = type;
  probe.timestamp = timestamp;
  return matchVehicleToGraphIndexed(store, matchIndex, probe);
}

std::optional<RoutePolyline> routeLocalSegmentIndexed(
    const GraphFileStore& store, const SpatialIndex& /*matchIndex*/,
    const GraphPosition& start, const GraphPosition& goal, double speedMs, VehicleType type,
    const PredictParam& param, double maxHorizon, double maxRadiusM,
    const RoutedTimeField* prebuiltField = nullptr,
    const GraphPosition* prebuiltGoal = nullptr) {
  if (!start.valid || !goal.valid || !store.hasCsr()) {
    return std::nullopt;
  }
  const CsrGraph& csr = store.csr();
  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;

  const GraphPosition& fieldGoal = prebuiltGoal != nullptr ? *prebuiltGoal : goal;
  const RoutedTimeField* fieldPtr = prebuiltField;
  RoutedTimeField owned;
  if (fieldPtr == nullptr) {
    std::unordered_set<int64_t> targetNodes;
    addRoutingTargetNodesIndexed(store, start, targetNodes);
    double glat = 0.0;
    double glon = 0.0;
    if (fieldGoal.edgeId == 0) {
      if (!store.nodeLatLon(fieldGoal.nodeId, glat, glon)) {
        return std::nullopt;
      }
    } else {
      int64_t from = 0;
      int64_t to = 0;
      if (!store.edgeEndpoints(fieldGoal.edgeId, from, to)) {
        return std::nullopt;
      }
      if (!store.nodeLatLon(from, glat, glon)) {
        return std::nullopt;
      }
    }
    const LatLon goalLl{glat, glon};
    owned = computeRoutedTimeFieldFromGoalCsr(store, csr, fieldGoal, speedMs, type, routeParam,
                                              maxHorizon, nullptr, &targetNodes, &goalLl,
                                              maxRadiusM);
    fieldPtr = &owned;
  }

  const RouteToGoal path =
      routeFromRoutedFieldCsr(store, csr, *fieldPtr, start, goal, speedMs, type, routeParam);
  if (path.polyline.points.size() >= 2) {
    return path.polyline;
  }
  return std::nullopt;
}

void densifyRouteGaps(const GraphFileStore& store, const SpatialIndex& matchIndex,
                      const VehicleInfo& vehicle, double speedMs, double maxHorizon,
                      const PredictParam& param, RoutePolyline& route) {
  if (!store.hasCsr() || route.points.size() < 2) {
    return;
  }
  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;
  RoutePolyline dense;
  dense.points.reserve(route.points.size() * 2);
  appendRoutePt(dense, route.points.front());
  for (std::size_t i = 1; i < route.points.size(); ++i) {
    const LatLon& a = dense.points.back();
    const LatLon& b = route.points[i];
    const double d = haversineMeters(a, b);
    if (d > 800.0) {
      const GraphPosition from =
          matchLatLonIndexed(store, matchIndex, a.lat, a.lon, vehicle.type, vehicle.timestamp);
      const GraphPosition to =
          matchLatLonIndexed(store, matchIndex, b.lat, b.lon, vehicle.type, vehicle.timestamp);
      if (from.valid && to.valid) {
        if (auto bridge = routeLocalSegmentIndexed(store, matchIndex, from, to, speedMs,
                                                   vehicle.type, routeParam, maxHorizon,
                                                   std::min(40000.0, d + 8000.0))) {
          appendPolylinePoints(dense, *bridge);
          continue;
        }
      }
    }
    appendRoutePt(dense, b);
  }
  route = std::move(dense);
}

std::shared_ptr<CachedLocalDestField> getOrBuildLocalDestField(
    const GraphFileStore& store, const SpatialIndex& matchIndex, const DestinationQuery& dest,
    double speedMs, double maxHorizon, const PredictParam& param, double maxRadiusM = 25000.0) {
  const uint64_t key = destRouteCacheKey(dest, maxRadiusM);
  {
    std::shared_lock<std::shared_mutex> read(localDestFieldCacheMu);
    if (localDestFieldCache && localDestFieldCache->key == key) {
      return localDestFieldCache;
    }
  }

  VehicleInfo destProbe;
  destProbe.id = "destination";
  destProbe.lat = dest.lat;
  destProbe.lon = dest.lon;
  destProbe.type = dest.type;
  destProbe.speed = 60.0;
  destProbe.timestamp = dest.arriveByUnix;
  const GraphPosition goalPos = matchVehicleToGraphIndexed(store, matchIndex, destProbe);
  if (!goalPos.valid) {
    return nullptr;
  }

  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;
  const CsrGraph& csr = store.csr();
  const LatLon goalLl{dest.lat, dest.lon};
  RoutedTimeField field = computeRoutedTimeFieldFromGoalCsr(
      store, csr, goalPos, speedMs, dest.type, routeParam, maxHorizon, nullptr, nullptr, &goalLl,
      maxRadiusM);

  auto cached = std::make_shared<CachedLocalDestField>();
  cached->key = key;
  cached->goalPos = goalPos;
  cached->field = std::move(field);

  std::unique_lock<std::shared_mutex> write(localDestFieldCacheMu);
  localDestFieldCache = cached;
  return cached;
}

std::optional<RoutePolyline> stitchChOverlayRoutePolyline(
    const GraphFileStore& snapStore, const SpatialIndex& matchIndex, const VehicleInfo& vehicle,
    const DestinationQuery& dest, double speedMs, double maxHorizon, const PredictParam& param,
    const RoutePolyline& chPath) {
  if (chPath.points.size() < 2) {
    return std::nullopt;
  }
  const LatLon vehLl{vehicle.lat, vehicle.lon};
  const LatLon destLl{dest.lat, dest.lon};
  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;

  RoutePolyline route;
  const LatLon& firstChPt = chPath.points.front();
  const LatLon& lastChPt = chPath.points.back();

  bool hasAccess = false;
  if (snapStore.hasCsr()) {
    const GraphPosition startPos = matchVehicleToGraphIndexed(snapStore, matchIndex, vehicle);
    const GraphPosition portalPos =
        matchLatLonIndexed(snapStore, matchIndex, firstChPt.lat, firstChPt.lon, vehicle.type,
                           vehicle.timestamp);
    const double accessGap = haversineMeters(vehLl, firstChPt);
    if (accessGap <= 1500.0) {
      appendRoutePt(route, vehLl);
      hasAccess = true;
    } else if (startPos.valid && portalPos.valid) {
      const double accessRadius = std::min(35000.0, accessGap + 10000.0);
      if (auto access = routeLocalSegmentIndexed(snapStore, matchIndex, startPos, portalPos, speedMs,
                                                 vehicle.type, routeParam, maxHorizon,
                                                 accessRadius)) {
        appendPolylinePoints(route, *access);
        hasAccess = true;
      }
    }
  }
  if (!hasAccess) {
    if (haversineMeters(vehLl, firstChPt) > 60000.0) {
      return std::nullopt;
    }
    appendRoutePt(route, vehLl);
  }
  appendPolylinePoints(route, chPath);

  bool hasTail = false;
  const double tailGap = haversineMeters(route.points.back(), destLl);
  if (tailGap <= 1500.0) {
    appendRoutePt(route, destLl);
    hasTail = true;
  } else if (snapStore.hasCsr()) {
    const auto destLocal =
        getOrBuildLocalDestField(snapStore, matchIndex, dest, speedMs, maxHorizon, param);
    if (destLocal && destLocal->goalPos.valid) {
      const GraphPosition goalPortalPos =
          matchLatLonIndexed(snapStore, matchIndex, lastChPt.lat, lastChPt.lon, dest.type,
                             dest.arriveByUnix);
      if (goalPortalPos.valid) {
        if (auto egress = routeLocalSegmentIndexed(snapStore, matchIndex, goalPortalPos,
                                                   destLocal->goalPos, speedMs, dest.type,
                                                   routeParam, maxHorizon, 35000.0,
                                                   &destLocal->field, &destLocal->goalPos)) {
          appendPolylinePoints(route, *egress);
          hasTail = true;
        }
      }
    }
  }
  if (!hasTail) {
    if (tailGap > 12000.0) {
      return std::nullopt;
    }
    appendRoutePt(route, destLl);
  }

  if (route.points.size() < 2) {
    return std::nullopt;
  }
  simplifyRoutePolyline(route, 120);
  return route;
}

void stitchHybridMetroRoute(const GraphFileStore& snapStore, const SpatialIndex& matchIndex,
                            const GraphFileStore& chStore, const ChGraph& ch,
                            const CsrGraph& hwyCsr, const VehicleInfo& vehicle, double speedMs,
                            double maxHorizon, int64_t bestPortal, int64_t goalNodeId,
                            const std::vector<int64_t>& parentNodeByRow,
                            const std::vector<int64_t>& parentEdgeByRow,
                            const RoutePolyline* sharedEgress,
                            const CachedLocalDestField* destLocal, const DestinationQuery& dest,
                            const PredictParam& param, const SpatialIndex* nationalSnapIndex,
                            VehicleArrivalResult& row) {
  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;

  RoutePolyline hwyPath = polylineFromGoalCsrParents(chStore, hwyCsr, parentNodeByRow,
                                                    parentEdgeByRow, bestPortal, goalNodeId);
  if (hwyPath.points.size() < 2) {
    return;
  }

  RoutePolyline route;
  double plat = 0.0;
  double plon = 0.0;
  if (!chStore.nodeLatLon(bestPortal, plat, plon)) {
    return;
  }

  const double portalDistM = haversineMeters({vehicle.lat, vehicle.lon}, {plat, plon});

  bool hasAccess = false;
  if (snapStore.hasCsr()) {
    const GraphPosition startPos = matchVehicleToGraphIndexed(snapStore, matchIndex, vehicle);
    const GraphPosition portalPos =
        matchLatLonIndexed(snapStore, matchIndex, plat, plon, vehicle.type, vehicle.timestamp);
    if (startPos.valid && portalPos.valid) {
      if (portalDistM <= 1500.0) {
        appendRoutePt(route, {vehicle.lat, vehicle.lon});
        hasAccess = true;
      } else {
        const double accessRadius = std::min(35000.0, portalDistM + 10000.0);
        if (auto access = routeLocalSegmentIndexed(snapStore, matchIndex, startPos, portalPos,
                                                   speedMs, vehicle.type, routeParam, maxHorizon,
                                                   accessRadius)) {
          appendPolylinePoints(route, *access);
          hasAccess = true;
        }
      }
    }
  }
  if (!hasAccess && chStore.hasCh() && chStore.hasHwyCsr()) {
    const SpatialIndex& chIndex =
        nationalSnapIndex != nullptr ? *nationalSnapIndex : matchIndex;
    std::vector<int64_t> fromNodes =
        snapHwyPortalSeeds(chStore, chStore, hwyCsr, chIndex, vehicle.lat, vehicle.lon,
                           vehicle.type, 4);
    if (fromNodes.empty()) {
      const std::vector<int64_t> towardGoal{bestPortal};
      fromNodes = collectOverlayChNodesToward(chStore, ch, hwyCsr, vehicle.lat, vehicle.lon,
                                              towardGoal, portalDistM * 0.25 + 25000.0, 8);
    }
    RoutePolyline bestAccess;
    for (int64_t fromNode : fromNodes) {
      const RouteToGoal path = ch.query(chStore, hwyCsr, fromNode, bestPortal, vehicle.type,
                                        routeParam, maxHorizon);
      if (path.polyline.points.size() >= 2) {
        bestAccess = path.polyline;
        break;
      }
    }
    if (bestAccess.points.size() >= 2) {
      if (haversineMeters({vehicle.lat, vehicle.lon}, bestAccess.points.front()) > 1500.0) {
        appendRoutePt(route, {vehicle.lat, vehicle.lon});
      }
      appendPolylinePoints(route, bestAccess);
      hasAccess = true;
    }
  }
  if (!hasAccess) {
    return;
  }

  appendPolylinePoints(route, hwyPath);

  bool hasEgress = false;
  if (sharedEgress != nullptr && sharedEgress->points.size() >= 2) {
    appendPolylinePoints(route, *sharedEgress);
    hasEgress = true;
  } else if (snapStore.hasCsr() && destLocal != nullptr && destLocal->goalPos.valid) {
    double glat = 0.0;
    double glon = 0.0;
    if (chStore.nodeLatLon(goalNodeId, glat, glon)) {
      const GraphPosition goalPortalPos = matchLatLonIndexed(snapStore, matchIndex, glat, glon,
                                                             dest.type, dest.arriveByUnix);
      if (goalPortalPos.valid) {
        if (auto egress = routeLocalSegmentIndexed(snapStore, matchIndex, goalPortalPos,
                                                   destLocal->goalPos, speedMs, dest.type,
                                                   routeParam, maxHorizon, 35000.0,
                                                   &destLocal->field, &destLocal->goalPos)) {
          appendPolylinePoints(route, *egress);
          hasEgress = true;
        }
      }
    }
  }
  if (!hasEgress) {
    return;
  }

  const LatLon destLl{dest.lat, dest.lon};
  if (route.points.empty() || haversineMeters(route.points.back(), destLl) > 200.0) {
    if (snapStore.hasCsr() && destLocal != nullptr && destLocal->goalPos.valid) {
      const GraphPosition endPos = matchLatLonIndexed(snapStore, matchIndex, route.points.back().lat,
                                                    route.points.back().lon, dest.type,
                                                    dest.arriveByUnix);
      if (endPos.valid) {
        if (auto tail = routeLocalSegmentIndexed(snapStore, matchIndex, endPos, destLocal->goalPos,
                                                 speedMs, dest.type, routeParam, maxHorizon,
                                                 5000.0, &destLocal->field, &destLocal->goalPos)) {
          appendPolylinePoints(route, *tail);
        }
      }
    }
    if (route.points.empty() || haversineMeters(route.points.back(), destLl) > 200.0) {
      appendRoutePt(route, destLl);
    }
  }

  if (route.points.size() >= 2) {
    densifyRouteGaps(snapStore, matchIndex, vehicle, speedMs, maxHorizon, param, route);
    simplifyRoutePolyline(route, 120);
    row.route = std::move(route);
    row.routeDistanceM = polylineLengthMeters(row.route);
  }
}

void enrichChRouteWithRegionalConnectors(const GraphFileStore& snapStore,
                                         const SpatialIndex& matchIndex,
                                         const VehicleInfo& vehicle,
                                         const DestinationQuery& dest, const PredictParam& param,
                                         VehicleArrivalResult& row) {
  if (!snapStore.hasCsr() || row.route.points.size() < 2) {
    return;
  }

  const double speedMs = speedMsFromKmh(vehicle.speed > 0.0 ? vehicle.speed : 60.0);
  const double maxHorizon = row.travelDurationSec + 120.0;
  const auto destLocal =
      getOrBuildLocalDestField(snapStore, matchIndex, dest, speedMs, maxHorizon, param);
  if (!destLocal || !destLocal->goalPos.valid) {
    return;
  }

  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;

  RoutePolyline route;
  const LatLon firstChPt = row.route.points.front();
  const LatLon lastChPt = row.route.points.back();
  appendRoutePt(route, {vehicle.lat, vehicle.lon});

  const GraphPosition startPos = matchVehicleToGraphIndexed(snapStore, matchIndex, vehicle);
  const GraphPosition portalPos =
      matchLatLonIndexed(snapStore, matchIndex, firstChPt.lat, firstChPt.lon, vehicle.type,
                         vehicle.timestamp);
  if (startPos.valid && portalPos.valid) {
    const double accessRadius =
        std::min(35000.0, haversineMeters({vehicle.lat, vehicle.lon}, firstChPt) + 10000.0);
    if (auto access = routeLocalSegmentIndexed(snapStore, matchIndex, startPos, portalPos, speedMs,
                                               vehicle.type, routeParam, maxHorizon,
                                               accessRadius)) {
      appendPolylinePoints(route, *access);
    }
  }
  appendPolylinePoints(route, row.route);

  const GraphPosition goalPortalPos =
      matchLatLonIndexed(snapStore, matchIndex, lastChPt.lat, lastChPt.lon, dest.type,
                         dest.arriveByUnix);
  if (goalPortalPos.valid) {
    if (auto egress = routeLocalSegmentIndexed(snapStore, matchIndex, goalPortalPos,
                                               destLocal->goalPos, speedMs, dest.type, routeParam,
                                               maxHorizon, 35000.0, &destLocal->field,
                                               &destLocal->goalPos)) {
      appendPolylinePoints(route, *egress);
    }
  }
  appendRoutePt(route, {dest.lat, dest.lon});

  if (route.points.size() >= 2) {
    simplifyRoutePolyline(route, 120);
    row.route = std::move(route);
    row.routeDistanceM = polylineLengthMeters(row.route);
  }
}

std::shared_ptr<CachedDestRouteField> getOrBuildDestRouteField(
    const GraphFileStore& store, const SpatialIndex& matchIndex,
    const std::vector<VehicleInfo>& corridorVehicles, const DestinationQuery& dest,
    double corridorWidthM, double groupMaxHorizon, double groupSpeedMs,
    const PredictParam& param) {
  const uint64_t key = destRouteCacheKey(dest, corridorWidthM);
  {
    std::shared_lock<std::shared_mutex> read(destRouteFieldCacheMu);
    if (destRouteFieldCache && destRouteFieldCache->key == key) {
      return destRouteFieldCache;
    }
  }
  VehicleInfo destProbe;
  destProbe.id = "destination";
  destProbe.lat = dest.lat;
  destProbe.lon = dest.lon;
  destProbe.type = dest.type;
  destProbe.speed = 60.0;
  destProbe.timestamp = dest.arriveByUnix;
  const GraphPosition goalPos = matchVehicleToGraphIndexed(store, matchIndex, destProbe);
  if (!goalPos.valid) {
    return nullptr;
  }

  double spanM = 0.0;
  const LatLon goalLl{dest.lat, dest.lon};
  for (const auto& vehicle : corridorVehicles) {
    spanM = std::max(spanM, haversineMeters({vehicle.lat, vehicle.lon}, goalLl));
  }
  const bool radiusOnly = spanM < 200000.0;

  std::shared_ptr<std::unordered_set<int64_t>> allowed;
  std::unordered_set<int64_t> targetNodes;
  targetNodes.reserve(corridorVehicles.size() * 2);
  double maxRadiusM = radiusOnly ? std::min(spanM + 15000.0, 100000.0) : 25000.0;
  const double fieldHorizon = groupMaxHorizon;

  if (!radiusOnly) {
    allowed = std::make_shared<std::unordered_set<int64_t>>();
    std::string collectErr;
    if (!collectDestinationCorridorBboxEdgeIdsIndexed(store, matchIndex, corridorVehicles, dest.lat,
                                                      dest.lon, corridorWidthM, *allowed,
                                                      &collectErr) ||
        allowed->empty()) {
      return nullptr;
    }
    for (const auto& vehicle : corridorVehicles) {
      maxRadiusM = std::max(
          maxRadiusM,
          std::min(haversineMeters({vehicle.lat, vehicle.lon}, goalLl) + 12000.0, 150000.0));
    }
  }

  for (const auto& vehicle : corridorVehicles) {
    if (vehicle.id == destProbe.id || vehicle.id == "destination") {
      continue;
    }
    const GraphPosition startPos = matchVehicleToGraphIndexed(store, matchIndex, vehicle);
    addRoutingTargetNodesIndexed(store, startPos, targetNodes);
  }

  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;
  const CsrGraph& csr = store.csr();
  const std::unordered_set<int64_t>* allowedPtr = allowed ? allowed.get() : nullptr;
  const std::unordered_set<int64_t>* targetPtr = targetNodes.empty() ? nullptr : &targetNodes;
  RoutedTimeField field = computeRoutedTimeFieldFromGoalCsr(
      store, csr, goalPos, groupSpeedMs, dest.type, routeParam, fieldHorizon, allowedPtr,
      targetPtr, &goalLl, maxRadiusM);

  auto cached = std::make_shared<CachedDestRouteField>();
  cached->key = key;
  cached->goalPos = goalPos;
  cached->field = std::move(field);
  cached->allowedEdges = std::move(allowed);
  cached->maxRadiusM = maxRadiusM;

  {
    std::unique_lock<std::shared_mutex> write(destRouteFieldCacheMu);
    if (destRouteFieldCache && destRouteFieldCache->key == key) {
      return destRouteFieldCache;
    }
    destRouteFieldCache = cached;
  }
  return cached;
}

std::vector<VehicleArrivalResult> predictVehiclesToDestinationHwyCsrBatch(
    const GraphFileStore& snapStore, const SpatialIndex& matchIndex,
    const GraphFileStore& chStore, const std::vector<VehicleInfo>& vehicles,
    const std::vector<VehicleHistory>& histories, const DestinationQuery& dest,
    const PredictParam& param, const SpatialIndex* nationalSnapIndex = nullptr,
    const GraphFileStore* nationalSnapStore = nullptr) {
  std::vector<VehicleArrivalResult> results;
  if (vehicles.empty() || !chStore.hasHwyCsr() || !chStore.hasCh()) {
    return results;
  }

  const CsrGraph& hwyCsr = chStore.hwyCsr();
  const ChGraph& ch = chStore.ch();
  const CsrGraph& goalWalkCsr = snapStore.hasCsr() ? snapStore.csr() : hwyCsr;
  const LatLon goalLl{dest.lat, dest.lon};

  struct VehicleJob {
    const VehicleInfo* vehicle = nullptr;
    const VehicleHistory* history = nullptr;
    std::vector<int64_t> portals;
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
    if (!mayReachWithSpeed(vehicle, dest, speedMs)) {
      continue;
    }
    const double maxHorizon =
        std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
    if (maxHorizon < 1.0) {
      continue;
    }
    jobs.push_back({&vehicle, hist, {}, speedMs, maxHorizon});
  }
  if (jobs.empty()) {
    return results;
  }

  std::vector<int64_t> goalNodes = findNearestHwyOverlayNodes(
      snapStore, chStore, hwyCsr, ch, matchIndex, dest.lat, dest.lon, 15000.0, 16);
  if (goalNodes.empty()) {
    goalNodes = snapHwyPortalSeeds(snapStore, chStore, hwyCsr, matchIndex, dest.lat, dest.lon,
                                   dest.type, 16);
  }
  if (goalNodes.empty()) {
    goalNodes = collectOverlayChNodes(snapStore, chStore, ch, goalWalkCsr, matchIndex, dest.lat,
                                      dest.lon, dest.type, 25000.0, 16);
  }
  if (goalNodes.empty()) {
    return results;
  }

  parallelFor(jobs.size(), [&](std::size_t i) {
    if (!tryReuseVehiclePortals(*jobs[i].vehicle, jobs[i].portals)) {
      jobs[i].portals = snapHwyPortalSeeds(snapStore, chStore, hwyCsr, matchIndex,
                                             jobs[i].vehicle->lat, jobs[i].vehicle->lon,
                                             jobs[i].vehicle->type, 4);
      storeVehiclePortals(*jobs[i].vehicle, jobs[i].portals);
    }
  });

  // Drop portals that are not near the vehicle — a far portal makes ETA look
  // reachable via a huge "walk" and reconstructs as a straight line on the map.
  constexpr double kMaxPortalFromVehicleM = 80000.0;
  auto filterNearVehiclePortals = [&](VehicleJob& job) {
    std::vector<int64_t> kept;
    kept.reserve(job.portals.size());
    for (int64_t portal : job.portals) {
      double plat = 0.0;
      double plon = 0.0;
      if (!chStore.nodeLatLon(portal, plat, plon)) {
        continue;
      }
      if (haversineMeters({job.vehicle->lat, job.vehicle->lon}, {plat, plon}) <=
          kMaxPortalFromVehicleM) {
        kept.push_back(portal);
      }
    }
    job.portals = std::move(kept);
  };
  for (auto& job : jobs) {
    filterNearVehiclePortals(job);
  }

  std::vector<std::size_t> needToward;
  needToward.reserve(jobs.size());
  for (std::size_t i = 0; i < jobs.size(); ++i) {
    if (jobs[i].portals.empty()) {
      needToward.push_back(i);
    }
  }
  if (!needToward.empty()) {
    // Prefer geometric nearest highway nodes near the vehicle (not a walk from
    // the destination, which can leave remote fleets with goal-side portals).
    parallelFor(needToward.size(), [&](std::size_t j) {
      const std::size_t i = needToward[j];
      const VehicleInfo& vehicle = *jobs[i].vehicle;
      jobs[i].portals = findNearestHwyOverlayNodes(snapStore, chStore, hwyCsr, ch, matchIndex,
                                                   vehicle.lat, vehicle.lon, 50000.0, 8);
      if (jobs[i].portals.empty()) {
        jobs[i].portals = findNearestHwyOverlayNodes(snapStore, chStore, hwyCsr, ch, matchIndex,
                                                     vehicle.lat, vehicle.lon, 120000.0, 8);
      }
      if (jobs[i].portals.empty()) {
        const double distToGoal =
            haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
        const double geoMax =
            distToGoal > 180000.0 ? std::min(120000.0, distToGoal * 0.08 + 25000.0)
                                  : std::min(80000.0, distToGoal * 0.25 + 15000.0);
        jobs[i].portals = collectOverlayChNodesToward(chStore, ch, hwyCsr, vehicle.lat, vehicle.lon,
                                                      goalNodes, geoMax, 8);
        filterNearVehiclePortals(jobs[i]);
      }
      if (jobs[i].portals.empty()) {
        jobs[i].portals = collectOverlayChNodes(snapStore, chStore, ch, hwyCsr, matchIndex,
                                                vehicle.lat, vehicle.lon, vehicle.type, 50000.0, 8);
        filterNearVehiclePortals(jobs[i]);
      }
      if (!jobs[i].portals.empty()) {
        storeVehiclePortals(vehicle, jobs[i].portals);
      }
    });
  }

  {
    std::size_t withPortal = 0;
    double maxPortalGap = 0.0;
    for (const auto& job : jobs) {
      if (job.portals.empty()) {
        continue;
      }
      ++withPortal;
      for (int64_t portal : job.portals) {
        double plat = 0.0;
        double plon = 0.0;
        if (chStore.nodeLatLon(portal, plat, plon)) {
          maxPortalGap = std::max(
              maxPortalGap,
              haversineMeters({job.vehicle->lat, job.vehicle->lon}, {plat, plon}));
        }
      }
    }
    std::cerr << "[mmlp] hwy_portals jobs=" << jobs.size() << " with_portal=" << withPortal
              << " need_seed=" << needToward.size() << " max_portal_gap_m=" << maxPortalGap
              << " goal_nodes=" << goalNodes.size() << "\n"
              << std::flush;
  }

  constexpr double kMetroDistM = 180000.0;
  std::vector<VehicleJob> metroJobs;
  std::vector<VehicleJob> remoteJobs;
  metroJobs.reserve(jobs.size());
  remoteJobs.reserve(jobs.size());
  for (auto& job : jobs) {
    if (job.portals.empty()) {
      continue;
    }
    const double distM =
        haversineMeters({job.vehicle->lat, job.vehicle->lon}, {dest.lat, dest.lon});
    if (distM <= kMetroDistM) {
      metroJobs.push_back(std::move(job));
    } else {
      remoteJobs.push_back(std::move(job));
    }
  }

  std::vector<VehicleJob> hwyJobs;
  hwyJobs.reserve(metroJobs.size() + remoteJobs.size());
  for (auto& job : metroJobs) {
    hwyJobs.push_back(std::move(job));
  }
  for (auto& job : remoteJobs) {
    hwyJobs.push_back(std::move(job));
  }

  GraphPosition goalPos;
  goalPos.valid = true;
  goalPos.nodeId = goalNodes.front();
  goalPos.edgeId = 0;
  double bestGoalDist = 1e100;
  for (int64_t gn : goalNodes) {
    double glat = 0.0;
    double glon = 0.0;
    if (!chStore.nodeLatLon(gn, glat, glon)) {
      continue;
    }
    const double d = haversineMeters({dest.lat, dest.lon}, {glat, glon});
    if (d < bestGoalDist) {
      bestGoalDist = d;
      goalPos.nodeId = gn;
    }
  }

  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;

  auto buildRowFromPortals = [&](const VehicleJob& job,
                                 const std::function<double(int64_t)>& nodeTime,
                                 int64_t* bestPortalOut = nullptr)
      -> std::optional<VehicleArrivalResult> {
    double bestTravel = kInfTime;
    int64_t bestPortal = 0;
    for (int64_t portal : job.portals) {
      const double base = nodeTime(portal);
      if (base >= kInfTime / 2.0) {
        continue;
      }
      double plat = 0.0;
      double plon = 0.0;
      if (!chStore.nodeLatLon(portal, plat, plon)) {
        continue;
      }
      const double walkM =
          haversineMeters({job.vehicle->lat, job.vehicle->lon}, {plat, plon});
      const double travel =
          base + travelTimeSeconds(walkM, job.speedMs, job.vehicle->type, routeParam);
      if (travel < bestTravel) {
        bestTravel = travel;
        bestPortal = portal;
      }
    }
    if (bestTravel >= kInfTime / 2.0 || bestTravel > job.maxHorizon + 1e-6) {
      return std::nullopt;
    }
    const double eta = static_cast<double>(job.vehicle->timestamp) + bestTravel;
    if (eta > static_cast<double>(dest.arriveByUnix) + 1e-6) {
      return std::nullopt;
    }
    if (bestPortalOut != nullptr) {
      *bestPortalOut = bestPortal;
    }
    VehicleArrivalResult row;
    row.vehicleId = job.vehicle->id;
    row.reachable = true;
    row.travelDurationSec = bestTravel;
    row.etaUnix = eta;
    row.routeDistanceM = bestTravel * job.speedMs;
    return row;
  };

  // Reconstruct highway polyline from the shared reverse Dijkstra parent tree
  // (O(path length), not a second CH query per vehicle).
  auto attachHwyParentRoute = [&](VehicleArrivalResult& row, const VehicleJob& job,
                                  int64_t portalNode, const CachedHwyDistField& field) {
    if (portalNode == 0 || field.parentNodeByRow.empty()) {
      return;
    }
    double plat = 0.0;
    double plon = 0.0;
    if (!chStore.nodeLatLon(portalNode, plat, plon)) {
      return;
    }
    const LatLon vehLl{job.vehicle->lat, job.vehicle->lon};
    // Guard: portal must sit near the vehicle or the map line becomes a fake chord.
    if (haversineMeters(vehLl, {plat, plon}) > 80000.0) {
      return;
    }
    RoutePolyline hwyPath =
        polylineFromGoalCsrParents(chStore, hwyCsr, field.parentNodeByRow, field.parentEdgeByRow,
                                   portalNode, field.goalNodeId);
    if (hwyPath.points.size() < 2) {
      // Parent tree incomplete for this portal: fall back to one CH query.
      const RouteToGoal path =
          ch.query(chStore, hwyCsr, portalNode, field.goalNodeId, job.vehicle->type, routeParam,
                   job.maxHorizon + 60.0);
      if (path.polyline.points.size() < 2 || path.travelTimeSec >= kInfTime / 2.0) {
        return;
      }
      hwyPath = path.polyline;
    }
    const LatLon destLl{dest.lat, dest.lon};
    RoutePolyline route;
    appendRoutePt(route, vehLl);
    appendPolylinePoints(route, hwyPath);
    appendRoutePt(route, destLl);
    simplifyRoutePolyline(route, 120);
    // Reject near-straight artifacts (vehicle -> dest with almost no shape).
    if (route.points.size() >= 4 && maxPolylineSegmentMeters(route) <= 250000.0) {
      row.route = std::move(route);
      row.routeDistanceM = polylineLengthMeters(row.route);
    }
  };

  if (!hwyJobs.empty()) {
    std::unordered_set<int64_t> targetNodes;
    targetNodes.reserve(hwyJobs.size() * 4);
    double spanM = 0.0;
    double groupSpeedMs = hwyJobs.front().speedMs;
    double groupMaxHorizon = 0.0;
    for (const auto& job : hwyJobs) {
      spanM = std::max(
          spanM, haversineMeters({job.vehicle->lat, job.vehicle->lon}, {dest.lat, dest.lon}));
      groupSpeedMs = std::min(groupSpeedMs, job.speedMs);
      groupMaxHorizon = std::max(groupMaxHorizon, job.maxHorizon);
      for (int64_t portal : job.portals) {
        targetNodes.insert(portal);
      }
    }

    // Cross-country: reverse Dijkstra from the destination often fails to reach
    // remote portals (sparse hwy CSR / early target pruning). Use one CH query
    // per vehicle instead — parallel and returns a real highway polyline.
    constexpr double kLongHaulSpanM = 250000.0;
    if (spanM > kLongHaulSpanM) {
      // Cap seed fan-out: 2 portals × 2 goals keeps 98-vehicle fleets in ~1–3s.
      std::vector<int64_t> longGoals = goalNodes;
      if (longGoals.size() > 2) {
        longGoals.resize(2);
      }
      std::vector<std::optional<VehicleArrivalResult>> rows(hwyJobs.size());
      parallelFor(hwyJobs.size(), [&](std::size_t i) {
        const VehicleJob& job = hwyJobs[i];
        const LatLon vehLl{job.vehicle->lat, job.vehicle->lon};
        const LatLon destLl{dest.lat, dest.lon};
        std::vector<int64_t> portals = job.portals;
        if (portals.size() > 2) {
          // Keep the two portals closest to the vehicle.
          std::sort(portals.begin(), portals.end(), [&](int64_t a, int64_t b) {
            double alat = 0.0, alon = 0.0, blat = 0.0, blon = 0.0;
            const double da =
                chStore.nodeLatLon(a, alat, alon) ? haversineMeters(vehLl, {alat, alon}) : 1e100;
            const double db =
                chStore.nodeLatLon(b, blat, blon) ? haversineMeters(vehLl, {blat, blon}) : 1e100;
            return da < db;
          });
          portals.resize(2);
        }
        double bestTravel = kInfTime;
        RoutePolyline bestRoute;
        int tried = 0;
        int foundPath = 0;
        int rejectedShape = 0;
        for (int64_t portal : portals) {
          double plat = 0.0;
          double plon = 0.0;
          if (!chStore.nodeLatLon(portal, plat, plon)) {
            continue;
          }
          const double walkM = haversineMeters(vehLl, {plat, plon});
          if (walkM > 80000.0) {
            continue;
          }
          for (int64_t goalNode : longGoals) {
            ++tried;
            const RouteToGoal path =
                ch.query(chStore, hwyCsr, portal, goalNode, job.vehicle->type, routeParam,
                         job.maxHorizon + 60.0);
            if (path.polyline.points.size() < 2 || path.travelTimeSec >= kInfTime / 2.0) {
              continue;
            }
            ++foundPath;
            const double travel =
                path.travelTimeSec +
                travelTimeSeconds(walkM, job.speedMs, job.vehicle->type, routeParam);
            if (travel >= bestTravel || travel > job.maxHorizon + 1e-6) {
              continue;
            }
            RoutePolyline route;
            appendRoutePt(route, vehLl);
            appendPolylinePoints(route, path.polyline);
            appendRoutePt(route, destLl);
            simplifyRoutePolyline(route, 120);
            if (route.points.size() < 4 || maxPolylineSegmentMeters(route) > 250000.0) {
              ++rejectedShape;
              continue;
            }
            bestTravel = travel;
            bestRoute = std::move(route);
          }
        }
        if (i < 2) {
          std::cerr << "[mmlp] long_haul_ch veh=" << job.vehicle->id << " portals=";
          for (int64_t portal : portals) {
            std::cerr << portal << ",";
          }
          std::cerr << " goals=";
          for (int64_t g : longGoals) {
            std::cerr << g << ",";
          }
          std::cerr << " tried=" << tried << " found=" << foundPath
                    << " reject_shape=" << rejectedShape << " best=" << bestTravel << "\n"
                    << std::flush;
        }
        if (bestTravel >= kInfTime / 2.0) {
          return;
        }
        const double eta = static_cast<double>(job.vehicle->timestamp) + bestTravel;
        if (eta > static_cast<double>(dest.arriveByUnix) + 1e-6) {
          return;
        }
        VehicleArrivalResult row;
        row.vehicleId = job.vehicle->id;
        row.reachable = true;
        row.travelDurationSec = bestTravel;
        row.etaUnix = eta;
        row.route = std::move(bestRoute);
        row.routeDistanceM = polylineLengthMeters(row.route);
        rows[i] = std::move(row);
      });
      results.reserve(results.size() + hwyJobs.size());
      for (auto& row : rows) {
        if (row) {
          results.push_back(std::move(*row));
        }
      }
      return results;
    }

    // Metro / regional: shared reverse Dijkstra field + parent reconstruction.
    const double maxRadiusM = spanM + 35000.0;
    const uint64_t hwyKey = destRouteCacheKey(dest, 0.0) ^ static_cast<uint64_t>(goalPos.nodeId);
    std::shared_ptr<CachedHwyDistField> hwyCached;
    {
      std::shared_lock<std::shared_mutex> read(hwyDistFieldCacheMu);
      if (hwyDistFieldCache && hwyDistFieldCache->key == hwyKey &&
          hwyDistFieldCache->goalNodeId == goalPos.nodeId &&
          hwyDistFieldCache->distByRow.size() == hwyCsr.nodeCount() &&
          !hwyDistFieldCache->parentNodeByRow.empty()) {
        hwyCached = hwyDistFieldCache;
      }
    }
    if (!hwyCached) {
      auto cached = std::make_shared<CachedHwyDistField>();
      cached->key = hwyKey;
      cached->goalNodeId = goalPos.nodeId;
      cached->distByRow = computeRoutedDistFromGoalCsrDense(
          chStore, hwyCsr, goalPos.nodeId, groupSpeedMs, dest.type, routeParam, groupMaxHorizon,
          &targetNodes, &goalLl, maxRadiusM, &cached->parentNodeByRow, &cached->parentEdgeByRow);
      std::unique_lock<std::shared_mutex> write(hwyDistFieldCacheMu);
      if (hwyDistFieldCache && hwyDistFieldCache->key == hwyKey &&
          !hwyDistFieldCache->parentNodeByRow.empty()) {
        hwyCached = hwyDistFieldCache;
      } else {
        hwyDistFieldCache = cached;
        hwyCached = std::move(cached);
      }
    }
    const std::vector<double>& distByRow = hwyCached->distByRow;
    auto nodeTime = [&](int64_t nodeId) -> double {
      const int row = hwyCsr.nodeRow(chStore, nodeId);
      if (row < 0) {
        return kInfTime;
      }
      return distByRow[static_cast<std::size_t>(row)];
    };

    {
      std::size_t finitePortal = 0;
      std::size_t totalPortal = 0;
      for (const auto& job : hwyJobs) {
        for (int64_t portal : job.portals) {
          ++totalPortal;
          if (nodeTime(portal) < kInfTime / 2.0) {
            ++finitePortal;
          }
        }
      }
      std::cerr << "[mmlp] hwy_field goal=" << goalPos.nodeId << " span_m=" << spanM
                << " radius_m=" << maxRadiusM << " targets=" << targetNodes.size()
                << " finite_portals=" << finitePortal << "/" << totalPortal << "\n"
                << std::flush;
    }

    // Parent-tree reconstruction is cheap; do all vehicles in parallel.
    std::vector<std::optional<VehicleArrivalResult>> rows(hwyJobs.size());
    parallelFor(hwyJobs.size(), [&](std::size_t i) {
      int64_t bestPortal = 0;
      if (auto row = buildRowFromPortals(hwyJobs[i], nodeTime, &bestPortal)) {
        attachHwyParentRoute(*row, hwyJobs[i], bestPortal, *hwyCached);
        rows[i] = std::move(*row);
      }
    });
    results.reserve(results.size() + hwyJobs.size());
    for (auto& row : rows) {
      if (row) {
        results.push_back(std::move(*row));
      }
    }
  }

  return results;
}

std::unordered_map<std::string, RoutePolyline> computeRegionalRoutesByVehicleId(
    const GraphFileStore& store, const SpatialIndex& matchIndex,
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const DestinationQuery& dest, const PredictParam& param) {
  std::unordered_map<std::string, RoutePolyline> routes;
  if (vehicles.empty() || !store.hasCsr()) {
    return routes;
  }

  struct EnrichJob {
    std::string vehicleId;
    const VehicleInfo* vehicle = nullptr;
    double speedMs = 0.0;
    double distM = 0.0;
    double maxHorizon = 0.0;
  };
  std::vector<EnrichJob> jobs;
  jobs.reserve(vehicles.size());
  constexpr double kMetroDistM = 150000.0;
  for (const auto& vehicle : vehicles) {
    if (vehicle.type != dest.type) {
      continue;
    }
    const double distM = haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
    if (distM > kMetroDistM) {
      continue;
    }
    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, hist, nullptr, vehicle.type));
    if (!mayReachWithSpeed(vehicle, dest, speedMs)) {
      continue;
    }
    const double maxHorizon =
        std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
    if (maxHorizon < 1.0) {
      continue;
    }
    jobs.push_back({vehicle.id, &vehicle, speedMs, distM, maxHorizon});
  }
  if (jobs.empty()) {
    return routes;
  }

  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;
  double groupSpeedMs = jobs.front().speedMs;
  double groupMaxHorizon = 0.0;
  double spanM = 0.0;
  for (const auto& job : jobs) {
    groupSpeedMs = std::min(groupSpeedMs, job.speedMs);
    groupMaxHorizon = std::max(groupMaxHorizon, job.maxHorizon);
    spanM = std::max(spanM, job.distM);
  }

  std::unordered_set<int64_t> targetNodes;
  targetNodes.reserve(jobs.size() * 2);
  std::vector<GraphPosition> startPositions(jobs.size());
  parallelFor(jobs.size(), [&](std::size_t j) {
    startPositions[j] = matchVehicleToGraphIndexed(store, matchIndex, *jobs[j].vehicle);
  });
  for (std::size_t j = 0; j < jobs.size(); ++j) {
    if (startPositions[j].valid) {
      addRoutingTargetNodesIndexed(store, startPositions[j], targetNodes);
    }
  }

  const double maxRadiusM = bucketMetroRadiusM(spanM);
  const auto tField0 = std::chrono::steady_clock::now();
  bool fieldCacheHit = false;
  const std::shared_ptr<CachedMetroRegionalField> metroCached = getOrBuildMetroRegionalField(
      store, matchIndex, dest, groupSpeedMs, groupMaxHorizon, maxRadiusM, param, &targetNodes,
      &fieldCacheHit);
  const auto tField1 = std::chrono::steady_clock::now();
  if (!metroCached || metroCached->distByRow.empty() || metroCached->goalNodeId == 0) {
    return routes;
  }
  const GraphPosition& cachedGoalPos = metroCached->goalPos;
  const CsrGraph& csr = store.csr();
  const int64_t goalNodeId = metroCached->goalNodeId;

  const auto tRoute0 = std::chrono::steady_clock::now();
  std::vector<std::optional<RoutePolyline>> rows(jobs.size());
  parallelFor(jobs.size(), [&](std::size_t j) {
    const EnrichJob& job = jobs[j];
    if (!startPositions[j].valid) {
      return;
    }
    const int64_t startNode = resolveDenseStartNode(store, csr, metroCached->distByRow,
                                                    startPositions[j], job.speedMs,
                                                    job.vehicle->type, routeParam);
    if (startNode == 0) {
      return;
    }
    RoutePolyline route = polylineFromGoalCsrParents(store, csr, metroCached->parentNodeByRow,
                                                   metroCached->parentEdgeByRow, startNode,
                                                   goalNodeId);
    if (startPositions[j].edgeId != 0) {
      const LatLon startPt = positionLatLonStore(store, startPositions[j]);
      if (route.points.empty() || haversineMeters(route.points.front(), startPt) > 1.0) {
        route.points.insert(route.points.begin(), startPt);
      }
    }
    const LatLon endLoc = positionLatLonStore(store, cachedGoalPos);
    if (route.points.empty() || haversineMeters(route.points.back(), endLoc) > 1.0) {
      route.points.push_back(endLoc);
    }
    if (route.points.size() >= 2) {
      rows[j] = std::move(route);
    }
  });
  const auto tRoute1 = std::chrono::steady_clock::now();

  routes.reserve(jobs.size());
  for (std::size_t j = 0; j < jobs.size(); ++j) {
    if (rows[j]) {
      RoutePolyline route = std::move(*rows[j]);
      simplifyRoutePolyline(route, 120);
      routes.emplace(jobs[j].vehicleId, std::move(route));
    }
  }
  std::cerr << "[mmlp] regional_routes jobs=" << jobs.size() << " field_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(tField1 - tField0).count()
            << " route_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(tRoute1 - tRoute0).count()
            << " cache_hit=" << (fieldCacheHit ? 1 : 0) << " routes=" << routes.size() << "\n"
            << std::flush;
  return routes;
}

void enrichArrivalRoutesRegional(const GraphFileStore& store, const SpatialIndex& matchIndex,
                                 const std::vector<VehicleInfo>& vehicles,
                                 const std::vector<VehicleHistory>& histories,
                                 const DestinationQuery& dest, double corridorWidthM,
                                 const PredictParam& param,
                                 std::vector<VehicleArrivalResult>& batch) {
  if (batch.empty() || !store.hasCsr()) {
    return;
  }
  const std::unordered_map<std::string, RoutePolyline> routes =
      computeRegionalRoutesByVehicleId(store, matchIndex, vehicles, histories, dest, param);
  for (auto& row : batch) {
    if (row.route.points.size() >= 2) {
      continue;
    }
    const auto it = routes.find(row.vehicleId);
    if (it == routes.end() || it->second.points.size() < 2) {
      continue;
    }
    row.route = it->second;
    row.routeDistanceM = polylineLengthMeters(row.route);
  }
}

std::unordered_map<std::string, RoutePolyline> computeChOverlayRoutesByVehicleId(
    const GraphFileStore& snapStore, const SpatialIndex& matchIndex,
    const GraphFileStore& chStore, const std::vector<VehicleInfo>& vehicles,
    const std::vector<VehicleHistory>& histories, const DestinationQuery& dest,
    const PredictParam& param, double minDistM = 180000.0) {
  std::unordered_map<std::string, RoutePolyline> routes;
  if (vehicles.empty() || !chStore.hasCh() || !chStore.hasHwyCsr()) {
    return routes;
  }

  struct OverlayJob {
    std::string vehicleId;
    const VehicleInfo* vehicle = nullptr;
    double maxHorizon = 0.0;
    double distM = 0.0;
  };
  std::vector<OverlayJob> jobs;
  jobs.reserve(vehicles.size());
  for (const auto& vehicle : vehicles) {
    if (vehicle.type != dest.type) {
      continue;
    }
    const double distM = haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
    if (distM <= minDistM) {
      continue;
    }
    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, hist, nullptr, vehicle.type));
    if (!mayReachWithSpeed(vehicle, dest, speedMs)) {
      continue;
    }
    const double maxHorizon =
        std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
    if (maxHorizon < 1.0) {
      continue;
    }
    jobs.push_back({vehicle.id, &vehicle, maxHorizon + 60.0, distM});
  }
  if (jobs.empty()) {
    return routes;
  }

  const CsrGraph& hwyCsr = chStore.hwyCsr();
  const CsrGraph& walkCsr = snapStore.hasCsr() ? snapStore.csr() : hwyCsr;
  const ChGraph& ch = chStore.ch();
  PredictParam routeParam = param;
  routeParam.maxVisitedNodes = 0;

  std::vector<int64_t> goalNodes = findNearestHwyOverlayNodes(
      snapStore, chStore, hwyCsr, ch, matchIndex, dest.lat, dest.lon, 15000.0, 8);
  if (goalNodes.empty()) {
    goalNodes = snapHwyPortalSeeds(snapStore, chStore, hwyCsr, matchIndex, dest.lat, dest.lon,
                                   dest.type, 8);
  }
  if (goalNodes.empty()) {
    return routes;
  }

  const auto tOverlay0 = std::chrono::steady_clock::now();
  std::vector<std::optional<RoutePolyline>> rows(jobs.size());
  parallelFor(jobs.size(), [&](std::size_t j) {
    const OverlayJob& job = jobs[j];
    const VehicleInfo& vehicle = *job.vehicle;
    std::vector<int64_t> fromNodes =
        snapHwyPortalSeeds(snapStore, chStore, hwyCsr, matchIndex, vehicle.lat, vehicle.lon,
                           vehicle.type, 4);
    if (fromNodes.empty()) {
      fromNodes = collectOverlayChNodesToward(chStore, ch, hwyCsr, vehicle.lat, vehicle.lon,
                                            goalNodes, job.distM * 0.25 + 25000.0, 8);
    }
    if (fromNodes.empty()) {
      fromNodes = collectOverlayChNodes(snapStore, chStore, ch, walkCsr, matchIndex, vehicle.lat,
                                        vehicle.lon, vehicle.type, 50000.0, 8);
    }
    if (fromNodes.empty()) {
      return;
    }
    RoutePolyline best;
    double bestTravel = kInfTime;
    const LatLon destLl{dest.lat, dest.lon};
    const LatLon vehLl{vehicle.lat, vehicle.lon};
    for (int64_t fromNode : fromNodes) {
      double flat = 0.0;
      double flon = 0.0;
      if (!chStore.nodeLatLon(fromNode, flat, flon)) {
        continue;
      }
      if (haversineMeters({flat, flon}, vehLl) > 60000.0) {
        continue;
      }
      for (int64_t toNode : goalNodes) {
        double tlat = 0.0;
        double tlon = 0.0;
        if (!chStore.nodeLatLon(toNode, tlat, tlon)) {
          continue;
        }
        if (haversineMeters({tlat, tlon}, destLl) > 50000.0) {
          continue;
        }
        const RouteToGoal path =
            ch.query(chStore, hwyCsr, fromNode, toNode, vehicle.type, routeParam, job.maxHorizon);
        if (path.polyline.points.size() < 2 || path.travelTimeSec >= bestTravel ||
            path.travelTimeSec >= kInfTime / 2.0 || path.travelTimeSec > job.maxHorizon + 1e-6) {
          continue;
        }
        if (haversineMeters(path.polyline.points.back(), destLl) > 80000.0) {
          continue;
        }
        bestTravel = path.travelTimeSec;
        best = path.polyline;
      }
    }
    if (best.points.size() < 2) {
      const VehicleHistory* hist = findHistory(histories, vehicle.id);
      if (auto chRow = predictVehicleToDestinationCh(snapStore, matchIndex, chStore, vehicle, hist,
                                                     dest, param)) {
        if (routePolylineQualityOk(chRow->route)) {
          rows[j] = chRow->route;
        }
      }
      return;
    }
    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, hist, nullptr, vehicle.type));
    if (auto stitched = stitchChOverlayRoutePolyline(snapStore, matchIndex, vehicle, dest, speedMs,
                                                     job.maxHorizon, param, best)) {
      if (routePolylineQualityOk(*stitched)) {
        rows[j] = std::move(*stitched);
        return;
      }
    }
    RoutePolyline route;
    if (haversineMeters(vehLl, best.points.front()) > 1500.0) {
      appendRoutePt(route, vehLl);
    }
    appendPolylinePoints(route, best);
    const double tailGap = haversineMeters(route.points.back(), destLl);
    if (tailGap <= 12000.0) {
      appendRoutePt(route, destLl);
    }
    if (routePolylineQualityOk(route)) {
      rows[j] = std::move(route);
    }
  });

  routes.reserve(jobs.size());
  for (std::size_t j = 0; j < jobs.size(); ++j) {
    if (rows[j]) {
      RoutePolyline route = std::move(*rows[j]);
      simplifyRoutePolyline(route, 120);
      routes.emplace(jobs[j].vehicleId, std::move(route));
    }
  }
  std::cerr << "[mmlp] ch_overlay jobs=" << jobs.size() << " overlay_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - tOverlay0)
                   .count()
            << " routes=" << routes.size() << "\n"
            << std::flush;
  return routes;
}

void enrichArrivalRoutesChOverlay(const GraphFileStore& snapStore, const SpatialIndex& matchIndex,
                                  const GraphFileStore& chStore,
                                  const std::vector<VehicleInfo>& vehicles,
                                  const std::vector<VehicleHistory>& histories,
                                  const DestinationQuery& dest, const PredictParam& param,
                                  std::vector<VehicleArrivalResult>& batch) {
  if (batch.empty() || !chStore.hasCh() || !chStore.hasHwyCsr()) {
    return;
  }
  std::vector<VehicleInfo> needOverlay;
  needOverlay.reserve(batch.size());
  std::unordered_set<std::string> haveRoute;
  for (const auto& row : batch) {
    if (row.route.points.size() >= 2) {
      haveRoute.insert(row.vehicleId);
    }
  }
  for (const auto& vehicle : vehicles) {
    if (haveRoute.count(vehicle.id) > 0) {
      continue;
    }
    const double distM = haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
    if (distM > 180000.0) {
      needOverlay.push_back(vehicle);
    }
  }
  if (needOverlay.empty()) {
    return;
  }
  const std::unordered_map<std::string, RoutePolyline> routes =
      computeChOverlayRoutesByVehicleId(snapStore, matchIndex, chStore, needOverlay, histories,
                                        dest, param);
  for (auto& row : batch) {
    if (row.route.points.size() >= 2) {
      continue;
    }
    const auto it = routes.find(row.vehicleId);
    if (it == routes.end() || it->second.points.size() < 2) {
      continue;
    }
    row.route = it->second;
    row.routeDistanceM = polylineLengthMeters(row.route);
  }
}

void appendDestinationCsrBatchWithFallback(
    std::vector<VehicleArrivalResult>& out, const GraphFileStore& store,
    const SpatialIndex& matchIndex, const GraphFileStore& chRef,
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const DestinationQuery& dest, double corridorWidthM, const PredictParam& param,
    const SpatialIndex* nationalSnapIndex = nullptr,
    const GraphFileStore* nationalSnapStore = nullptr) {
  if (vehicles.empty()) {
    return;
  }

  std::vector<VehicleArrivalResult> batch;
  if (chRef.hasHwyCsr() && chRef.hasCh()) {
    const auto tBatch0 = std::chrono::steady_clock::now();
    std::future<std::vector<VehicleArrivalResult>> etaFuture = runTopLevelAsync([&]() {
      return predictVehiclesToDestinationHwyCsrBatch(store, matchIndex, chRef, vehicles, histories,
                                                     dest, param, nationalSnapIndex,
                                                     nationalSnapStore);
    });
    std::future<std::unordered_map<std::string, RoutePolyline>> routeFuture;
    bool needChOverlay = false;
    for (const auto& vehicle : vehicles) {
      if (haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon}) > 180000.0) {
        needChOverlay = true;
        break;
      }
    }
    std::future<std::unordered_map<std::string, RoutePolyline>> overlayFuture;
    if (needChOverlay) {
      overlayFuture = runTopLevelAsync([&]() {
        return computeChOverlayRoutesByVehicleId(store, matchIndex, chRef, vehicles, histories, dest,
                                               param);
      });
    }
    if (store.hasCsr()) {
      routeFuture = runTopLevelAsync([&]() {
        return computeRegionalRoutesByVehicleId(store, matchIndex, vehicles, histories, dest, param);
      });
    }
    batch = etaFuture.get();
    if (routeFuture.valid()) {
      const std::unordered_map<std::string, RoutePolyline> routes = routeFuture.get();
      for (auto& row : batch) {
        const auto it = routes.find(row.vehicleId);
        if (it != routes.end() && routePolylineQualityOk(it->second)) {
          row.route = it->second;
          row.routeDistanceM = polylineLengthMeters(row.route);
        }
      }
    }
    if (overlayFuture.valid()) {
      const std::unordered_map<std::string, RoutePolyline> overlayRoutes = overlayFuture.get();
      for (auto& row : batch) {
        if (routePolylineQualityOk(row.route)) {
          continue;
        }
        const auto it = overlayRoutes.find(row.vehicleId);
        if (it != overlayRoutes.end() && routePolylineQualityOk(it->second)) {
          row.route = it->second;
          row.routeDistanceM = polylineLengthMeters(row.route);
        }
      }
    }
    constexpr double kLongHaulDistM = 150000.0;
    std::unordered_map<std::string, const VehicleInfo*> vehicleById;
    vehicleById.reserve(vehicles.size());
    for (const auto& vehicle : vehicles) {
      vehicleById.emplace(vehicle.id, &vehicle);
    }
    std::vector<VehicleInfo> needRouteFix;
    needRouteFix.reserve(batch.size());
    for (auto& row : batch) {
      if (routePolylineQualityOk(row.route)) {
        continue;
      }
      const auto vit = vehicleById.find(row.vehicleId);
      if (vit == vehicleById.end()) {
        continue;
      }
      const VehicleInfo& vehicle = *vit->second;
      const double distM = haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
      if (routePolylineDisplayOk(row.route, distM)) {
        continue;
      }
      if (distM > kLongHaulDistM) {
        row.route.points.clear();
        needRouteFix.push_back(vehicle);
      }
    }
    if (!needRouteFix.empty()) {
      const std::unordered_map<std::string, RoutePolyline> fixRoutes =
          computeChOverlayRoutesByVehicleId(store, matchIndex, chRef, needRouteFix, histories, dest,
                                            param, kLongHaulDistM);
      for (auto& row : batch) {
        if (routePolylineQualityOk(row.route)) {
          continue;
        }
        const auto it = fixRoutes.find(row.vehicleId);
        if (it != fixRoutes.end() && routePolylineQualityOk(it->second)) {
          row.route = it->second;
          row.routeDistanceM = polylineLengthMeters(row.route);
        }
      }
    }
    const auto tBatch1 = std::chrono::steady_clock::now();
    std::cerr << "[mmlp] dest_batch total_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(tBatch1 - tBatch0).count()
              << " reachable=" << batch.size() << "\n"
              << std::flush;
  }
  if (batch.size() < vehicles.size()) {
    std::unordered_set<std::string> have;
    have.reserve(batch.size());
    for (const auto& row : batch) {
      have.insert(row.vehicleId);
    }
    std::vector<VehicleInfo> needRegional;
    needRegional.reserve(vehicles.size());
    for (const auto& vehicle : vehicles) {
      if (have.count(vehicle.id) == 0) {
        needRegional.push_back(vehicle);
      }
    }
    if (!needRegional.empty() && store.hasCsr()) {
      std::vector<VehicleArrivalResult> regional =
          predictVehiclesToDestinationCsrBatch(store, matchIndex, needRegional, histories, dest,
                                               corridorWidthM, param);
      batch.insert(batch.end(), std::make_move_iterator(regional.begin()),
                   std::make_move_iterator(regional.end()));
    }
  }
  if (batch.empty() && store.hasCsr()) {
    batch = predictVehiclesToDestinationCsrBatch(store, matchIndex, vehicles, histories, dest,
                                                 corridorWidthM, param);
  }
  std::unordered_set<std::string> hitIds;
  hitIds.reserve(batch.size());
  for (auto& row : batch) {
    hitIds.insert(row.vehicleId);
    out.push_back(std::move(row));
  }

  std::vector<std::size_t> missed;
  missed.reserve(vehicles.size());
  for (std::size_t i = 0; i < vehicles.size(); ++i) {
    if (hitIds.count(vehicles[i].id) == 0) {
      missed.push_back(i);
    }
  }
  if (missed.empty()) {
    return;
  }
  if (missed.size() > 6) {
    return;
  }

  std::vector<std::optional<VehicleArrivalResult>> fallbackRows(missed.size());
  parallelFor(missed.size(), [&](std::size_t j) {
    const VehicleInfo& vehicle = vehicles[missed[j]];
    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    fallbackRows[j] = predictVehicleToDestinationIndexedOrCh(store, matchIndex, chRef, vehicle, hist,
                                                             dest, corridorWidthM, param);
  });
  for (auto& row : fallbackRows) {
    if (row) {
      out.push_back(std::move(*row));
    }
  }
}

}  // namespace

DestinationArrivalSummary predictVehiclesToDestination(
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const GraphContext& routeCtx, const SpatialIndex& matchIndex, const DestinationQuery& dest,
    const PredictParam& param) {
  const auto t0 = std::chrono::steady_clock::now();
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
    if (!mayReachWithSpeed(vehicle, dest, speedMs)) {
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

  std::vector<std::pair<int, std::vector<VehicleJob>>> speedGroups;
  if (jobs.size() >= 8) {
    // One backward search at fleet min speed (mobile-style multi-target).
    speedGroups.emplace_back(0, jobs);
  } else {
    std::map<int, std::vector<VehicleJob>> bySpeedKey;
    for (const auto& job : jobs) {
      const int key = static_cast<int>(std::floor(job.speedMs * 3.6 / 10.0));
      bySpeedKey[key].push_back(job);
    }
    speedGroups.reserve(bySpeedKey.size());
    for (auto& kv : bySpeedKey) {
      speedGroups.emplace_back(kv.first, std::move(kv.second));
    }
  }

  summary.vehicles.reserve(jobs.size());
  std::vector<std::future<std::vector<VehicleArrivalResult>>> futures;
  futures.reserve(speedGroups.size());
  for (auto& [speedKey, groupJobs] : speedGroups) {
    futures.push_back(std::async(
        std::launch::async,
        [&graph, &matchIndex, &goalPos, &dest, &param, routeParam,
         groupJobs = std::move(groupJobs)]() {
          std::vector<VehicleArrivalResult> rows;
          double speedMs = groupJobs.front().speedMs;
          for (const auto& job : groupJobs) {
            speedMs = std::min(speedMs, job.speedMs);
          }
          double groupMaxHorizon = 0.0;
          double maxRadiusM = 25000.0;
          const LatLon goalLl{dest.lat, dest.lon};
          std::vector<std::pair<const VehicleJob*, PreparedVehicle>> preparedJobs;
          preparedJobs.reserve(groupJobs.size());
          std::unordered_set<int64_t> targetNodes;
          targetNodes.reserve(groupJobs.size() * 2);

          for (const auto& job : groupJobs) {
            groupMaxHorizon = std::max(groupMaxHorizon, job.maxHorizon);
            maxRadiusM = std::max(
                maxRadiusM,
                haversineMeters({job.vehicle->lat, job.vehicle->lon}, goalLl) + 25000.0);
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
              graph, goalPos, speedMs, dest.type, routeParam, groupMaxHorizon, &targetNodes,
              &goalLl, maxRadiusM);

          std::cerr << "[mmlp] routed-field speed_kmh=" << static_cast<int>(speedMs * 3.6)
                    << " nodes=" << field.atNode.size() << " vehicles=" << groupJobs.size()
                    << " radius_m=" << static_cast<int>(maxRadiusM) << "\n"
                    << std::flush;

          for (const auto& [job, prepared] : preparedJobs) {
            const RouteToGoal path = routeFromRoutedField(graph, field, prepared.position, goalPos,
                                                          speedMs, job->vehicle->type, routeParam);
            double travel = path.travelTimeSec;
            if (travel < kInfTime / 2.0 && prepared.speedMs > speedMs + 0.01) {
              travel *= speedMs / prepared.speedMs;
            }
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
  const auto t1 = std::chrono::steady_clock::now();
  std::cerr << "[mmlp] destination full-graph vehicles=" << vehicles.size() << " edges="
            << routeCtx.graph.edges().size() << " total_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
            << " reachable=" << summary.vehicles.size() << "\n"
            << std::flush;
  return summary;
}

namespace {

struct FullChSeedInfo {
  std::vector<FullChGraph::Seed> seeds;              // costs in CH profile space
  std::unordered_map<int64_t, double> offsetMeters;  // seed node -> meters from snap point
};

// Seeds for a snapped position: node itself, or both edge endpoints with the
// partial-edge cost. Profile-space costs keep the CH search metric consistent.
FullChSeedInfo fullChSeedsForPosition(const GraphFileStore& store, const FullChGraph& ch,
                                      const GraphPosition& pos, double profileMs) {
  FullChSeedInfo info;
  if (!pos.valid) {
    return info;
  }
  if (pos.edgeId != 0) {
    EdgeType type = EdgeType::ROAD;
    double length = 0.0;
    double limitKmh = 0.0;
    int64_t from = 0;
    int64_t to = 0;
    if (!store.readEdge(pos.edgeId, type, length, limitKmh) ||
        !store.edgeEndpoints(pos.edgeId, from, to)) {
      return info;
    }
    double effMs = profileMs;
    if (limitKmh > 0.0) {
      effMs = std::min(effMs, speedMsFromKmh(limitKmh));
    }
    effMs = std::max(effMs, 0.5);
    const double along = std::min(std::max(pos.alongMeters, 0.0), std::max(length, 0.0));
    if (ch.nodeIndex(from) >= 0) {
      info.seeds.push_back({from, along / effMs});
      info.offsetMeters[from] = along;
    }
    if (ch.nodeIndex(to) >= 0) {
      info.seeds.push_back({to, std::max(0.0, length - along) / effMs});
      info.offsetMeters[to] = std::max(0.0, length - along);
    }
  } else if (pos.nodeId != 0 && ch.nodeIndex(pos.nodeId) >= 0) {
    info.seeds.push_back({pos.nodeId, 0.0});
    info.offsetMeters[pos.nodeId] = 0.0;
  }
  return info;
}

// Straight-line ETA rows (road-circuity factor + truck rests, no polyline) for
// vehicles at least minDistM from the destination; returns the rest untouched.
std::vector<VehicleInfo> appendGeodesicArrivalRows(std::vector<VehicleArrivalResult>& out,
                                                   const std::vector<VehicleInfo>& vehicles,
                                                   const std::vector<VehicleHistory>& histories,
                                                   const DestinationQuery& dest,
                                                   const PredictParam& param, double minDistM) {
  std::vector<VehicleInfo> rest;
  rest.reserve(vehicles.size());
  for (const VehicleInfo& vehicle : vehicles) {
    const double distM = haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
    if (vehicle.type != dest.type || distM < minDistM) {
      rest.push_back(vehicle);
      continue;
    }
    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, hist, nullptr, vehicle.type));
    const double maxHorizon =
        std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
    const double roadM = distM * 1.25;  // typical road-network circuity over straight line
    const double travel = travelTimeSeconds(roadM, speedMs, vehicle.type, param);
    const double eta = static_cast<double>(vehicle.timestamp) + travel;
    if (maxHorizon < 1.0 || travel > maxHorizon + 1e-6 ||
        eta > static_cast<double>(dest.arriveByUnix) + 1e-6) {
      continue;  // cannot arrive in time even on a straight line
    }
    VehicleArrivalResult row;
    row.vehicleId = vehicle.id;
    row.reachable = true;
    row.travelDurationSec = travel;
    row.etaUnix = eta;
    row.routeDistanceM = roadM;
    // No polyline: geodesic is ETA-only; callers that need a map line must
    // attach a real highway/regional route elsewhere.
    out.push_back(std::move(row));
  }
  return rest;
}

// Cross-province: one shared hwy reverse-Dijkstra field; ETA + real highway
// polylines reconstructed from parent pointers (no per-vehicle CH query).
void appendRemoteHwyArrivalBatch(
    std::vector<VehicleArrivalResult>& out, const GraphFileStore& snapStore,
    const SpatialIndex& snapIndex, const GraphFileStore& chRef,
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const DestinationQuery& dest, const PredictParam& param,
    const SpatialIndex* nationalSnapIndex, const GraphFileStore* nationalSnapStore) {
  if (vehicles.empty() || !chRef.hasCh() || !chRef.hasHwyCsr()) {
    return;
  }
  const auto t0 = std::chrono::steady_clock::now();
  std::vector<VehicleArrivalResult> batch = predictVehiclesToDestinationHwyCsrBatch(
      snapStore, snapIndex, chRef, vehicles, histories, dest, param, nationalSnapIndex,
      nationalSnapStore);
  std::unordered_set<std::string> have;
  have.reserve(batch.size());
  std::size_t withRoute = 0;
  for (auto& row : batch) {
    have.insert(row.vehicleId);
    if (row.route.points.size() >= 2) {
      ++withRoute;
    }
    out.push_back(std::move(row));
  }
  // Vehicles with no hwy portal / disconnected component: ETA-only geodesic
  // (skipped when the caller requires a drawable road polyline).
  std::vector<VehicleInfo> leftover;
  leftover.reserve(vehicles.size());
  for (const auto& vehicle : vehicles) {
    if (have.count(vehicle.id) == 0) {
      leftover.push_back(vehicle);
    }
  }
  if (!leftover.empty() && !param.requireRoutePolyline) {
    appendGeodesicArrivalRows(out, leftover, histories, dest, param, 0.0);
  }
  std::cerr << "[mmlp] remote_hwy_batch vehicles=" << vehicles.size()
            << " reachable=" << batch.size() << " with_route=" << withRoute
            << " geodesic=" << leftover.size() << " ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t0)
                   .count()
            << "\n"
            << std::flush;
}

// Full-graph CH fast path: one bidirectional query per vehicle, fully parallel,
// no on-demand Dijkstra fields or subgraph extraction. Returns the vehicles it
// could not serve (snap failure / off-graph), which the caller routes through
// the existing fallback paths. Vehicles proven unable to arrive in time are
// consumed here (no row, matching existing API semantics).
std::vector<VehicleInfo> appendDestinationFullChBatch(
    std::vector<VehicleArrivalResult>& out, std::string& locationId, const GraphFileStore& store,
    const SpatialIndex& matchIndex, const std::vector<VehicleInfo>& vehicles,
    const std::vector<VehicleHistory>& histories, const DestinationQuery& dest,
    const PredictParam& param, bool nearestSeeds = false) {
  const FullChGraph& ch = store.fullCh();
  const double profileMs = speedMsFromKmh(ch.profileKmh());

  VehicleInfo destProbe;
  destProbe.id = "destination";
  destProbe.lat = dest.lat;
  destProbe.lon = dest.lon;
  destProbe.type = dest.type;
  destProbe.speed = 60.0;
  destProbe.timestamp = dest.arriveByUnix;
  // Straight-line offset seeds from a list of CH node ids (hwy portal seeding).
  const auto seedsFromNodes = [&](const std::vector<int64_t>& nodes, double lat,
                                  double lon) -> FullChSeedInfo {
    FullChSeedInfo info;
    for (int64_t nodeId : nodes) {
      if (ch.nodeIndex(nodeId) < 0) {
        continue;
      }
      double nlat = 0.0;
      double nlon = 0.0;
      if (!store.nodeLatLon(nodeId, nlat, nlon)) {
        continue;
      }
      const double d = haversineMeters({lat, lon}, {nlat, nlon});
      info.seeds.push_back({nodeId, d / profileMs});
      info.offsetMeters[nodeId] = d;
    }
    return info;
  };
  const bool hwySeeding = nearestSeeds && store.hasCh() && store.hasHwyCsr();

  const GraphPosition goalPos = matchVehicleToGraphIndexed(store, matchIndex, destProbe);
  LatLon goalLl{dest.lat, dest.lon};
  FullChSeedInfo goalSeeds;
  if (goalPos.valid) {
    goalLl = positionLatLonStore(store, goalPos);
    const double goalGapM = haversineMeters(goalLl, {dest.lat, dest.lon});
    // Dest region already resolved by bbox, so a large snap gap only means an
    // off-road destination (sea, mountains): route to the nearest road point
    // and add the straight-line remainder, instead of exhaustive fallbacks.
    if (goalGapM <= 100000.0) {
      goalSeeds = fullChSeedsForPosition(store, ch, goalPos, profileMs);
      if (goalGapM > 30000.0) {
        for (FullChGraph::Seed& s : goalSeeds.seeds) {
          s.costSec += goalGapM / profileMs;
          goalSeeds.offsetMeters[s.nodeId] += goalGapM;
        }
      }
    }
  }
  // Always try nearest-road / hwy portal seeds when edge snap failed — a single
  // bad destination snap used to zero out every vehicle (all-or-nothing).
  // Edge snap only searches ~8km; desert/mountain clicks often need a wider
  // Full-CH nearest-node search so we still draw a real road polyline.
  if (goalSeeds.seeds.empty()) {
    goalLl = {dest.lat, dest.lon};
    std::vector<int64_t> goalNodes;
    if (store.hasCh() && store.hasHwyCsr()) {
      goalNodes = snapHwyPortalSeeds(store, store, store.hwyCsr(), matchIndex, dest.lat, dest.lon,
                                     dest.type, 8);
      if (goalNodes.empty()) {
        goalNodes = findNearestHwyOverlayNodes(store, store, store.hwyCsr(), store.ch(), matchIndex,
                                               dest.lat, dest.lon, 50000.0, 8);
      }
    }
    if (goalNodes.empty()) {
      for (const double radiusM : {25000.0, 50000.0, 100000.0}) {
        goalNodes = findNearestFullChRoadNodes(store, ch, matchIndex, dest.lat, dest.lon, dest.type,
                                               radiusM, 8);
        if (!goalNodes.empty()) {
          break;
        }
      }
    }
    if (goalNodes.empty() && goalPos.valid) {
      // Last resort: use whatever node/edge endpoints the spatial snap found,
      // even if the gap was large (better than returning zero routes).
      goalSeeds = fullChSeedsForPosition(store, ch, goalPos, profileMs);
      const double gapM = haversineMeters(positionLatLonStore(store, goalPos), {dest.lat, dest.lon});
      for (FullChGraph::Seed& s : goalSeeds.seeds) {
        s.costSec += gapM / profileMs;
        goalSeeds.offsetMeters[s.nodeId] += gapM;
      }
      if (!goalSeeds.seeds.empty()) {
        goalLl = positionLatLonStore(store, goalPos);
      }
    } else if (!goalNodes.empty()) {
      goalSeeds = seedsFromNodes(goalNodes, dest.lat, dest.lon);
      if (!goalSeeds.seeds.empty()) {
        double nlat = 0.0;
        double nlon = 0.0;
        if (store.nodeLatLon(goalSeeds.seeds.front().nodeId, nlat, nlon)) {
          goalLl = {nlat, nlon};
        }
        if (locationId.empty()) {
          locationId = "node:" + std::to_string(goalSeeds.seeds.front().nodeId);
        }
      }
    }
  }
  if (goalSeeds.seeds.empty()) {
    std::cerr << "[mmlp] full_ch goal_seeds empty dest=" << dest.lat << "," << dest.lon
              << " snap=" << (goalPos.valid ? "ok" : "miss") << "\n"
              << std::flush;
    if (!goalPos.valid || haversineMeters(goalLl, {dest.lat, dest.lon}) > 100000.0) {
      // Destination is far off-road (open sea, deep mountains): no road route
      // exists, straight-line ETA is the honest answer for everyone.
      if (param.requireRoutePolyline) {
        return vehicles;
      }
      return appendGeodesicArrivalRows(out, vehicles, histories, dest, param, 0.0);
    }
    return vehicles;
  }
  if (locationId.empty() && goalPos.valid) {
    locationId = graphLocationId(goalPos);
  }

  struct SlotResult {
    std::optional<VehicleArrivalResult> row;
    bool needFallback = false;
    char reason = 0;  // t=type s=snap g=gap e=seeds c=capped (debug log)
  };
  std::vector<SlotResult> slots(vehicles.size());

  parallelFor(vehicles.size(), [&](std::size_t i) {
    const VehicleInfo& vehicle = vehicles[i];
    if (vehicle.type != dest.type) {
      slots[i].needFallback = true;
      slots[i].reason = 't';
      return;
    }
    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    const double speedMs =
        std::max(speedMsFromKmh(resolveSpeedKmh(vehicle, hist, nullptr, vehicle.type)), 0.5);
    const double deadlineHorizon =
        std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
    // Search budget is independent of the user deadline: we still want a real
    // road route for late vehicles (reachable=false). Cap at 30 days profile.
    const double geoM = haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
    const double searchHorizon =
        std::max(deadlineHorizon > 1.0 ? deadlineHorizon : 0.0,
                 std::min(30.0 * 86400.0, geoM / std::max(speedMs * 0.5, 5.0) + 86400.0));
    if (searchHorizon < 1.0 && deadlineHorizon < 1.0) {
      return;
    }

    const GraphPosition pos = matchVehicleToGraphIndexed(store, matchIndex, vehicle);
    LatLon snapLl{vehicle.lat, vehicle.lon};
    FullChSeedInfo fromSeeds;
    if (pos.valid) {
      snapLl = positionLatLonStore(store, pos);
      if (haversineMeters(snapLl, {vehicle.lat, vehicle.lon}) <= 20000.0) {
        fromSeeds = fullChSeedsForPosition(store, ch, pos, profileMs);
      }
    }
    if (fromSeeds.seeds.empty() && hwySeeding) {
      snapLl = {vehicle.lat, vehicle.lon};
      std::vector<int64_t> fromNodes;
      if (!tryReuseVehiclePortals(vehicle, fromNodes)) {
        fromNodes = snapHwyPortalSeeds(store, store, store.hwyCsr(), matchIndex, vehicle.lat,
                                       vehicle.lon, vehicle.type, 4);
        if (fromNodes.empty()) {
          // Geometric nearest highway nodes (spatial-index bbox, ~tens of ms).
          fromNodes = findNearestHwyOverlayNodes(store, store, store.hwyCsr(), store.ch(),
                                                 matchIndex, vehicle.lat, vehicle.lon, 20000.0, 4);
        }
        storeVehiclePortals(vehicle, fromNodes);
      }
      fromSeeds = seedsFromNodes(fromNodes, vehicle.lat, vehicle.lon);
    }
    if (fromSeeds.seeds.empty()) {
      slots[i].needFallback = true;
      slots[i].reason = pos.valid ? 'e' : 's';
      return;
    }

    // Profile speed (<=80km/h capped by limits) is >= truck speed in practice, so
    // profile time <= vehicle time; pad anyway for the rare faster vehicle.
    const double maxProfileSec = searchHorizon * 1.6 + 1800.0;
    const std::size_t settleCap =
        param.maxVisitedNodes > 0 ? param.maxVisitedNodes : static_cast<std::size_t>(400000);
    const FullChGraph::PathResult path = ch.route(fromSeeds.seeds, goalSeeds.seeds, maxProfileSec,
                                                  settleCap, param.maxRouteWallMs);
    if (path.capped) {
      if (param.maxRouteWallMs > 0.0) {
        // Interactive corridor hop: hard miss, no slow fallback / geodesic chord.
        return;
      }
      slots[i].needFallback = true;  // search truncated: reachability unknown
      slots[i].reason = 'c';
      return;
    }
    if (!path.found) {
      if (nearestSeeds && param.maxRouteWallMs <= 0.0) {
        // Nearest-node seeding is approximate (seed may sit in a disconnected
        // fragment): "no path" is not authoritative, let the fallback decide.
        slots[i].needFallback = true;
        slots[i].reason = 'n';
        return;
      }
      // Full-graph CH covers every road node: no path within the padded cap
      // means the destination is not reachable inside the horizon.
      // Corridor hops (maxRouteWallMs>0) also hard-miss here.
      return;
    }

    double driveSec = 0.0;
    double distM = 0.0;
    for (const FullChGraph::PathArc& arc : path.arcs) {
      double effMs = speedMs;
      if (arc.speedLimitKmh > 0.0f) {
        effMs = std::min(effMs, speedMsFromKmh(static_cast<double>(arc.speedLimitKmh)));
      }
      driveSec += static_cast<double>(arc.lengthM) / std::max(effMs, 0.5);
      distM += static_cast<double>(arc.lengthM);
    }
    const auto offFromIt = fromSeeds.offsetMeters.find(path.startNodeId);
    const auto offGoalIt = goalSeeds.offsetMeters.find(path.endNodeId);
    const double offFromM = (offFromIt != fromSeeds.offsetMeters.end()) ? offFromIt->second : 0.0;
    const double offGoalM = (offGoalIt != goalSeeds.offsetMeters.end()) ? offGoalIt->second : 0.0;
    driveSec += (offFromM + offGoalM) / speedMs;
    distM += offFromM + offGoalM;

    double travel = driveSec;
    if (vehicle.type == VehicleType::TRUCK && param.truckCycle > 0.0) {
      travel += std::floor(driveSec / param.truckCycle) * param.truckRest;
    }
    const double eta = static_cast<double>(vehicle.timestamp) + travel;
    // Always emit a real road polyline when CH found a path. Vehicles that miss
    // the deadline are marked reachable=false so the map still shows the route
    // instead of the old all-or-nothing empty result.
    const bool onTime =
        deadlineHorizon >= 1.0 && travel <= deadlineHorizon + 1e-6 &&
        eta <= static_cast<double>(dest.arriveByUnix) + 1e-6;

    // Polyline: sample by road distance. Adaptive step keeps maxSeg drawable
    // on multi-thousand-km hauls (fixed 8km + 400-pt cap used to jump to end
    // and leave 200–700km visual chords).
    constexpr std::size_t kMaxPolyPts = 800;
    const double sampleEveryM =
        std::max(8000.0, distM / static_cast<double>(std::max<std::size_t>(kMaxPolyPts / 2, 40)));
    RoutePolyline route;
    route.points.reserve(std::min(path.arcs.size(), kMaxPolyPts) + 6);
    appendRoutePt(route, {vehicle.lat, vehicle.lon});
    appendRoutePt(route, snapLl);
    double plat = 0.0;
    double plon = 0.0;
    if (!path.arcs.empty() && store.nodeLatLon(path.arcs.front().fromNodeId, plat, plon)) {
      appendRoutePt(route, {plat, plon});
    }
    double sinceSampleM = 0.0;
    double stepM = sampleEveryM;
    for (std::size_t k = 0; k < path.arcs.size(); ++k) {
      sinceSampleM += static_cast<double>(path.arcs[k].lengthM);
      const bool last = (k + 1 == path.arcs.size());
      if (sinceSampleM >= stepM || last) {
        if (store.nodeLatLon(path.arcs[k].toNodeId, plat, plon)) {
          appendRoutePt(route, {plat, plon});
        }
        sinceSampleM = 0.0;
        // If we are filling the budget, sparsify remaining samples instead of
        // jumping to the destination (that creates geodesic-looking chords).
        if (route.points.size() >= (kMaxPolyPts * 3) / 4 && !last) {
          const double remainM = distM - polylineLengthMeters(route);
          const std::size_t room = (kMaxPolyPts > route.points.size() + 4)
                                       ? (kMaxPolyPts - route.points.size() - 4)
                                       : 1;
          stepM = std::max(stepM, remainM / static_cast<double>(room));
        }
      }
    }
    appendRoutePt(route, goalLl);
    appendRoutePt(route, {dest.lat, dest.lon});
    simplifyRoutePolyline(route, kMaxPolyPts);

    VehicleArrivalResult row;
    row.vehicleId = vehicle.id;
    row.reachable = onTime;
    row.travelDurationSec = travel;
    row.etaUnix = eta;
    row.routeDistanceM = distM;
    row.route = std::move(route);
    slots[i].row = std::move(row);
  });

  std::vector<VehicleInfo> leftover;
  std::string reasons;
  for (std::size_t i = 0; i < vehicles.size(); ++i) {
    if (slots[i].row) {
      out.push_back(std::move(*slots[i].row));
    } else if (slots[i].needFallback) {
      leftover.push_back(vehicles[i]);
      reasons.push_back(slots[i].reason != 0 ? slots[i].reason : '?');
    }
  }
  if (!leftover.empty()) {
    std::cerr << "[mmlp] full_ch fallback reasons=" << reasons
              << " (t=type s=snap g=gap e=seeds c=capped)\n"
              << std::flush;
  }
  return leftover;
}

}  // namespace

DestinationArrivalSummary predictVehiclesToDestinationIndexed(
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const GraphFileStore& store, const SpatialIndex& matchIndex, const DestinationQuery& dest,
    double maxCorridorWidthM, const PredictParam& param, const GraphFileStore* chOverlayStore,
    const GraphFileStore* nationalSnapStore, const SpatialIndex* nationalSnapIndex,
    const char* destRegionSuffix) {
  // Prefer explicit overlay (e.g. PRD hwy on national snap store) over store's own hwy CSR.
  const GraphFileStore& chRef =
      (chOverlayStore != nullptr && chOverlayStore->hasCh() && chOverlayStore->hasHwyCsr())
          ? *chOverlayStore
          : (store.hasCh() && store.hasHwyCsr())
                ? store
                : store;
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
    if (vehicle.type != dest.type) {
      ++pruned;
      continue;
    }
    // Do not prune by deadline here: late vehicles still get a Full-CH route
    // (reachable=false). Only drop wrong vehicle type.
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

  // Fast path: in-region vehicles use regional full.ch; cross-province vehicles use
  // national highway CH (with route polylines). Geodesic is last-resort ETA only.
  std::vector<VehicleInfo> localRoutable;
  std::vector<VehicleInfo> remoteRoutable;
  const bool splitByRegion = destRegionSuffix != nullptr && destRegionSuffix[0] != '\0';
  if (splitByRegion) {
    localRoutable.reserve(routable.size());
    remoteRoutable.reserve(routable.size());
    for (const auto& vehicle : routable) {
      if (pointInRegionSuffix(destRegionSuffix, vehicle.lat, vehicle.lon)) {
        localRoutable.push_back(vehicle);
      } else {
        remoteRoutable.push_back(vehicle);
      }
    }
  } else {
    localRoutable = routable;
  }

  const bool hasFastLocalCh = store.hasFullCh();
  const bool hasNationalHwy = chRef.hasCh() && chRef.hasHwyCsr();
  const bool canFastPath =
      dest.type == VehicleType::TRUCK &&
      ((hasFastLocalCh && !localRoutable.empty()) ||
       (hasNationalHwy && !remoteRoutable.empty()) ||
       (nationalSnapStore != nullptr && nationalSnapStore->hasFullCh()));
  if (canFastPath) {
    const auto tFast0 = std::chrono::steady_clock::now();
    std::vector<VehicleInfo> leftover;

    if (hasFastLocalCh && !localRoutable.empty()) {
      leftover = appendDestinationFullChBatch(summary.vehicles, summary.locationId, store,
                                            matchIndex, localRoutable, histories, dest, param);
      if (!leftover.empty() && param.maxRouteWallMs <= 0.0 && !param.requireRoutePolyline) {
        leftover =
            appendGeodesicArrivalRows(summary.vehicles, leftover, histories, dest, param, 0.0);
      }
    }

    if (!remoteRoutable.empty()) {
      // Prefer national dense Full CH (one-shot A→B) over sparse hwy CH.
      if (nationalSnapStore != nullptr && nationalSnapIndex != nullptr &&
          nationalSnapStore->hasFullCh()) {
        leftover = appendDestinationFullChBatch(summary.vehicles, summary.locationId,
                                                *nationalSnapStore, *nationalSnapIndex,
                                                remoteRoutable, histories, dest, param,
                                                /*nearestSeeds=*/false);
        if (!leftover.empty() && hasNationalHwy) {
          const GraphFileStore& remoteSnap = *nationalSnapStore;
          const SpatialIndex& remoteIdx = *nationalSnapIndex;
          appendRemoteHwyArrivalBatch(summary.vehicles, remoteSnap, remoteIdx, chRef, leftover,
                                      histories, dest, param, nationalSnapIndex,
                                      nationalSnapStore);
        } else if (!leftover.empty() && !param.requireRoutePolyline) {
          leftover = appendGeodesicArrivalRows(summary.vehicles, leftover, histories, dest, param,
                                               0.0);
        }
      } else if (hasNationalHwy) {
        const GraphFileStore& remoteSnap =
            (nationalSnapStore != nullptr) ? *nationalSnapStore : store;
        const SpatialIndex& remoteIdx =
            (nationalSnapIndex != nullptr) ? *nationalSnapIndex : matchIndex;
        appendRemoteHwyArrivalBatch(summary.vehicles, remoteSnap, remoteIdx, chRef,
                                    remoteRoutable, histories, dest, param, nationalSnapIndex,
                                    nationalSnapStore);
      } else if (!param.requireRoutePolyline) {
        appendGeodesicArrivalRows(summary.vehicles, remoteRoutable, histories, dest, param, 0.0);
      }
    }

    // Legacy path when no regional suffix: try national full.ch on leftovers.
    if (!splitByRegion && !leftover.empty() && nationalSnapStore != nullptr &&
        nationalSnapIndex != nullptr && nationalSnapStore->hasFullCh()) {
      leftover = appendDestinationFullChBatch(summary.vehicles, summary.locationId,
                                              *nationalSnapStore, *nationalSnapIndex, leftover,
                                              histories, dest, param, /*nearestSeeds=*/true);
      if (!leftover.empty() && !param.requireRoutePolyline) {
        leftover =
            appendGeodesicArrivalRows(summary.vehicles, leftover, histories, dest, param, 0.0);
      }
    }

    // Never surface ETA-only empty polylines when the map must draw roads.
    if (param.requireRoutePolyline) {
      summary.vehicles.erase(
          std::remove_if(summary.vehicles.begin(), summary.vehicles.end(),
                         [](const VehicleArrivalResult& row) { return row.route.points.size() < 2; }),
          summary.vehicles.end());
    }

    const auto tFast1 = std::chrono::steady_clock::now();
    std::cerr << "[mmlp] destination indexed(fast) vehicles=" << vehicles.size()
              << " pruned=" << pruned << " routable=" << routable.size()
              << " local=" << localRoutable.size() << " remote=" << remoteRoutable.size()
              << " leftover=" << leftover.size() << " fast_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(tFast1 - tFast0).count()
              << " reachable=" << summary.vehicles.size() << "\n"
              << std::flush;
    sortDestinationArrivals(summary.vehicles, dest.sortBy);
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
  const double minDistM =
      *std::min_element(distsToDest.begin(), distsToDest.end());
  const auto medianIt = distsToDest.begin() + distsToDest.size() / 2;
  std::nth_element(distsToDest.begin(), medianIt, distsToDest.end());
  const double medianDistM = *medianIt;
  const double localCorridorWidthM =
      medianDistM < 180000.0 ? std::min(maxCorridorWidthM, 30000.0) : maxCorridorWidthM;

  std::vector<VehicleInfo> collective;
  std::vector<VehicleInfo> remote;
  collective.reserve(routable.size());
  remote.reserve(routable.size());
  for (std::size_t i = 0; i < routable.size(); ++i) {
    const double distM = distsToDest[i];
  // Pull long-tail vehicles out of the collective bbox. A single far vehicle
  // (e.g. 380km vs 86km median) otherwise expands the extract to all of SC.
    const bool outlier = distM > 250000.0 ||
                         (distM > 2.5 * medianDistM && (distM - medianDistM) > 120000.0);
    if (outlier) {
      remote.push_back(routable[i]);
    } else {
      collective.push_back(routable[i]);
    }
  }

  // Small remainders batch faster than per-vehicle corridors when still metro-scale.
  if (!remote.empty() && collective.size() < 8 && medianDistM < 150000.0) {
    collective.insert(collective.end(), remote.begin(), remote.end());
    remote.clear();
  }

  double collectiveSpanM = 0.0;
  for (const auto& vehicle : collective) {
    collectiveSpanM = std::max(
        collectiveSpanM,
        haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon}));
  }

  const bool metroDenseFleet =
      collective.size() >= 12 && medianDistM < 200000.0 && store.hasCsr();

  // Hwy overlay batch: minDistM gate; small fleets skip maxSpanM (mixed near+far outliers).
  if (chRef.hasCh() && chRef.hasHwyCsr() && minDistM < 200000.0 &&
      (maxSpanM <= kPerVehicleThresholdM || routable.size() <= 12) && !routable.empty()) {
    const std::vector<VehicleInfo>& batch = routable;

    VehicleInfo destProbe;
    destProbe.id = "destination";
    destProbe.lat = dest.lat;
    destProbe.lon = dest.lon;
    destProbe.type = dest.type;
    destProbe.speed = 60.0;
    destProbe.timestamp = dest.arriveByUnix;
    const GraphPosition goalPos = matchVehicleToGraphIndexed(store, matchIndex, destProbe);
    summary.locationId = graphLocationId(goalPos);

    const auto tHwy0 = std::chrono::steady_clock::now();
    appendDestinationCsrBatchWithFallback(summary.vehicles, store, matchIndex, chRef, batch,
                                          histories, dest, localCorridorWidthM, param,
                                          nationalSnapIndex, nationalSnapStore);
    sortDestinationArrivals(summary.vehicles, dest.sortBy);
    const auto tHwy1 = std::chrono::steady_clock::now();
    std::cerr << "[mmlp] destination indexed(hwy_batch) vehicles=" << vehicles.size()
              << " pruned=" << pruned << " routable=" << routable.size()
              << " batch=" << batch.size() << " total_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(tHwy1 - t0).count()
              << " hwy_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(tHwy1 - tHwy0).count()
              << " reachable=" << summary.vehicles.size() << "\n"
              << std::flush;
    return summary;
  }

  // CH overlay probe for sparse fleets within metro-scale span.
  if (chRef.hasCh() && chRef.hasHwyCsr() && collectiveSpanM <= kPerVehicleThresholdM) {
    const CsrGraph& hwyCsr = chRef.hwyCsr();
    const CsrGraph& walkCsr = store.hasCsr() ? store.csr() : hwyCsr;
    const VehicleInfo& probeVehicle = !collective.empty() ? collective.front() : routable.front();
    std::vector<int64_t> probeTo =
        collectOverlayChNodes(store, chRef, chRef.ch(), walkCsr, matchIndex, dest.lat, dest.lon,
                            dest.type, 20000.0, 8);
    const double probeGeoMaxM =
        std::max(80000.0, haversineMeters({probeVehicle.lat, probeVehicle.lon}, {dest.lat, dest.lon}) +
                              20000.0);
    std::vector<int64_t> probeFrom = collectOverlayChNodes(
        store, chRef, chRef.ch(), walkCsr, matchIndex, probeVehicle.lat, probeVehicle.lon,
        probeVehicle.type, 20000.0, 8);
    if (probeFrom.empty()) {
      probeFrom = collectOverlayChNodesToward(chRef, chRef.ch(), hwyCsr, probeVehicle.lat,
                                              probeVehicle.lon, probeTo, probeGeoMaxM, 8);
    }
    const bool hwyConnected = !probeFrom.empty() && !probeTo.empty() &&
                              csrSeedsConnected(chRef, hwyCsr, probeFrom, probeTo, 80000);
    bool chLikely = false;
    if (!probeFrom.empty() && !probeTo.empty()) {
      PredictParam probeParam = param;
      probeParam.maxVisitedNodes = 0;
      const double probeHorizon = 86400.0;
      for (int64_t fromNode : probeFrom) {
        for (int64_t toNode : probeTo) {
          const RouteToGoal path = chRef.ch().query(chRef, hwyCsr, fromNode, toNode, probeVehicle.type,
                                                    probeParam, probeHorizon);
          if (path.travelTimeSec < kInfTime / 2.0) {
            chLikely = true;
            break;
          }
        }
        if (chLikely) {
          break;
        }
      }
    }

    std::cerr << "[mmlp] ch probe hwy_connected=" << hwyConnected << " ch_likely=" << chLikely
              << "\n"
              << std::flush;

    if (chLikely) {
      std::vector<VehicleInfo> batch;
      batch.reserve(collective.size() + remote.size());
      batch.insert(batch.end(), collective.begin(), collective.end());
      batch.insert(batch.end(), remote.begin(), remote.end());

      VehicleInfo destProbe;
      destProbe.id = "destination";
      destProbe.lat = dest.lat;
      destProbe.lon = dest.lon;
      destProbe.type = dest.type;
      destProbe.speed = 60.0;
      destProbe.timestamp = dest.arriveByUnix;
      const GraphPosition goalPos = matchVehicleToGraphIndexed(store, matchIndex, destProbe);
      summary.locationId = graphLocationId(goalPos);

      const auto tHwy0 = std::chrono::steady_clock::now();
      appendDestinationCsrBatchWithFallback(summary.vehicles, store, matchIndex, chRef, batch,
                                            histories, dest, localCorridorWidthM, param);
      sortDestinationArrivals(summary.vehicles, dest.sortBy);
      const auto tHwy1 = std::chrono::steady_clock::now();
      std::cerr << "[mmlp] destination indexed(hwy_sparse) vehicles=" << vehicles.size()
                << " batch=" << batch.size() << " total_ms="
                << std::chrono::duration_cast<std::chrono::milliseconds>(tHwy1 - t0).count()
                << " reachable=" << summary.vehicles.size() << "\n"
                << std::flush;
      return summary;
    }
  }

  // Sparse non-metro fleets (e.g. national outliers): one CSR batch beats per-vehicle extracts.
  if (!metroDenseFleet && routable.size() <= 12 && store.hasCsr()) {
    VehicleInfo destProbe;
    destProbe.id = "destination";
    destProbe.lat = dest.lat;
    destProbe.lon = dest.lon;
    destProbe.type = dest.type;
    destProbe.speed = 60.0;
    destProbe.timestamp = dest.arriveByUnix;
    const GraphPosition goalPos = matchVehicleToGraphIndexed(store, matchIndex, destProbe);
    summary.locationId = graphLocationId(goalPos);

    appendDestinationCsrBatchWithFallback(summary.vehicles, store, matchIndex, chRef, routable,
                                          histories, dest, maxCorridorWidthM, param);
    sortDestinationArrivals(summary.vehicles, dest.sortBy);
    const auto tSmallBatch1 = std::chrono::steady_clock::now();
    std::cerr << "[mmlp] destination indexed csr_small vehicles=" << vehicles.size()
              << " routable=" << routable.size() << " total_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(tSmallBatch1 - t0).count()
              << " reachable=" << summary.vehicles.size() << "\n"
              << std::flush;
    return summary;
  }

  // Spatial grid split only for sparse long-haul fleets. Near-destination metro fleets
  // are faster with one tight batch than dozens of per-vehicle subgraph builds.
  if (routable.size() >= 16 && medianDistM > 300000.0 && maxSpanM > 500000.0) {
    const double cellDeg = medianDistM < 150000.0 ? 0.06 : (medianDistM < 350000.0 ? 0.10 : 0.18);
    constexpr std::size_t kMaxPerCluster = 5;
    std::vector<std::vector<VehicleInfo>> clusters =
        clusterVehiclesByGrid(routable, cellDeg, kMaxPerCluster);
    if (clusters.size() > 1) {
      std::vector<std::future<DestinationArrivalSummary>> futures;
      futures.reserve(clusters.size());
      for (const auto& cluster : clusters) {
        futures.push_back(runTopLevelAsync([&, cluster]() {
          return predictVehiclesToDestinationIndexed(cluster, histories, store, matchIndex, dest,
                                                   maxCorridorWidthM, param, chOverlayStore,
                                                   nationalSnapStore, nationalSnapIndex,
                                                   destRegionSuffix);
        }));
      }
      for (auto& future : futures) {
        DestinationArrivalSummary part = future.get();
        if (summary.locationId.empty() && !part.locationId.empty()) {
          summary.locationId = part.locationId;
        }
        summary.vehicles.insert(summary.vehicles.end(),
                                std::make_move_iterator(part.vehicles.begin()),
                                std::make_move_iterator(part.vehicles.end()));
      }
      sortDestinationArrivals(summary.vehicles, dest.sortBy);
      const auto tGrid1 = std::chrono::steady_clock::now();
      std::cerr << "[mmlp] destination indexed grid vehicles=" << vehicles.size()
                << " routable=" << routable.size() << " clusters=" << clusters.size()
                << " cell_deg=" << cellDeg << " total_ms="
                << std::chrono::duration_cast<std::chrono::milliseconds>(tGrid1 - t0).count()
                << " reachable=" << summary.vehicles.size() << "\n"
                << std::flush;
      return summary;
    }
  }

  if (!collective.empty() && collectiveSpanM <= kPerVehicleThresholdM) {
    // Dense metro: parallel bearing bins with radius-limited CSR (no corridor mmap).
    if (metroDenseFleet) {
      std::vector<std::vector<VehicleInfo>> groups =
          clusterVehiclesByBearingBins(collective, dest.lat, dest.lon, 4);

      VehicleInfo destProbe;
      destProbe.id = "destination";
      destProbe.lat = dest.lat;
      destProbe.lon = dest.lon;
      destProbe.type = dest.type;
      destProbe.speed = 60.0;
      destProbe.timestamp = dest.arriveByUnix;
      const GraphPosition goalPos = matchVehicleToGraphIndexed(store, matchIndex, destProbe);
      summary.locationId = graphLocationId(goalPos);

      if (groups.size() <= 1) {
        appendDestinationCsrBatchWithFallback(summary.vehicles, store, matchIndex, chRef,
                                              collective, histories, dest, localCorridorWidthM,
                                              param);
      } else {
        std::vector<std::future<std::vector<VehicleArrivalResult>>> futures;
        futures.reserve(groups.size());
        for (const auto& group : groups) {
          futures.push_back(runTopLevelAsync([&, group]() {
            std::vector<VehicleArrivalResult> part;
            appendDestinationCsrBatchWithFallback(part, store, matchIndex, chRef, group, histories,
                                                dest, localCorridorWidthM, param);
            return part;
          }));
        }
        for (auto& future : futures) {
          std::vector<VehicleArrivalResult> part = future.get();
          summary.vehicles.insert(summary.vehicles.end(),
                                  std::make_move_iterator(part.begin()),
                                  std::make_move_iterator(part.end()));
        }
      }
      if (!remote.empty()) {
        appendDestinationCsrBatchWithFallback(summary.vehicles, store, matchIndex, chRef, remote,
                                            histories, dest, maxCorridorWidthM, param);
      }
      sortDestinationArrivals(summary.vehicles, dest.sortBy);
      const auto tMetro1 = std::chrono::steady_clock::now();
      std::cerr << "[mmlp] destination indexed csr_radius_bins vehicles=" << vehicles.size()
                << " groups=" << groups.size() << " total_ms="
                << std::chrono::duration_cast<std::chrono::milliseconds>(tMetro1 - t0).count()
                << " reachable=" << summary.vehicles.size() << "\n"
                << std::flush;
      return summary;
    }

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
      parallelFor(jobs.size(), [&](std::size_t i) {
        const SmallJob& job = jobs[i];
        rows[i] = predictVehicleToDestinationIndexedOrCh(store, matchIndex, chRef, *job.vehicle,
                                                         job.history, dest, maxCorridorWidthM,
                                                         param);
      });
      for (auto& row : rows) {
        if (row) {
          summary.vehicles.push_back(std::move(*row));
        }
      }

      std::int64_t predictMs = 0;
      if (!remote.empty()) {
        const auto tRemote0 = std::chrono::steady_clock::now();
        appendDestinationCsrBatchWithFallback(summary.vehicles, store, matchIndex, chRef, remote,
                                              histories, dest, maxCorridorWidthM, param);
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
          store, matchIndex, collective, dest.lat, dest.lon, localCorridorWidthM, allowedEdges,
          &collectErr);
    } else {
      collected = collectDestinationCorridorBboxEdgeIdsIndexed(
          store, matchIndex, collective, dest.lat, dest.lon, localCorridorWidthM, allowedEdges,
          &collectErr);
    }
    const auto tc1 = std::chrono::steady_clock::now();
    collectMs = std::chrono::duration_cast<std::chrono::milliseconds>(tc1 - tc0).count();
    if (!collected) {
      std::cerr << "[mmlp] destination edge collect failed: " << collectErr << "\n" << std::flush;
      return summary;
    }

    if (store.hasCsr() && !allowedEdges.empty()) {
      const auto tPredict0 = std::chrono::steady_clock::now();

      struct VehicleJob {
        const VehicleInfo* vehicle = nullptr;
        const VehicleHistory* history = nullptr;
        double speedMs = 0.0;
        double maxHorizon = 0.0;
      };
      std::vector<VehicleJob> jobs;
      jobs.reserve(collective.size());
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

      double groupSpeedMs = jobs.empty() ? 0.0 : jobs.front().speedMs;
      double groupMaxHorizon = 0.0;
      double maxRadiusM = 25000.0;
      const LatLon goalLl{dest.lat, dest.lon};
      std::unordered_set<int64_t> targetNodes;
      targetNodes.reserve(jobs.size() * 2);
      for (const auto& job : jobs) {
        groupSpeedMs = std::min(groupSpeedMs, job.speedMs);
        groupMaxHorizon = std::max(groupMaxHorizon, job.maxHorizon);
        maxRadiusM = std::max(
            maxRadiusM, haversineMeters({job.vehicle->lat, job.vehicle->lon}, goalLl) + 12000.0);
        const GraphPosition startPos =
            matchVehicleToGraphIndexed(store, matchIndex, *job.vehicle);
        addRoutingTargetNodesIndexed(store, startPos, targetNodes);
      }

      const CsrGraph& csr = store.csr();
      const RoutedTimeField field = computeRoutedTimeFieldFromGoalCsr(
          store, csr, goalPos, groupSpeedMs, dest.type, routeParam, groupMaxHorizon, &allowedEdges,
          &targetNodes, &goalLl, maxRadiusM);

      std::vector<std::optional<VehicleArrivalResult>> collectiveRows(jobs.size());
      parallelFor(jobs.size(), [&](std::size_t i) {
        const VehicleJob& job = jobs[i];
        const GraphPosition startPos =
            matchVehicleToGraphIndexed(store, matchIndex, *job.vehicle);
        if (!startPos.valid) {
          return;
        }
        const RouteToGoal path = routeFromRoutedFieldCsr(store, csr, field, startPos, goalPos,
                                                         job.speedMs, job.vehicle->type, routeParam);
        const double travel = path.travelTimeSec;
        if (travel >= kInfTime / 2.0 || travel > job.maxHorizon + 1e-6) {
          return;
        }
        const double eta = static_cast<double>(job.vehicle->timestamp) + travel;
        if (eta > static_cast<double>(dest.arriveByUnix) + 1e-6) {
          return;
        }
        VehicleArrivalResult row;
        row.vehicleId = job.vehicle->id;
        row.reachable = true;
        row.travelDurationSec = travel;
        row.etaUnix = eta;
        row.routeDistanceM = polylineLengthMeters(path.polyline);
        row.route = path.polyline;
        simplifyRoutePolyline(row.route, 120);
        collectiveRows[i] = std::move(row);
      });
      for (auto& row : collectiveRows) {
        if (row) {
          summary.vehicles.push_back(std::move(*row));
        }
      }

      if (!remote.empty()) {
        appendDestinationCsrBatchWithFallback(summary.vehicles, store, matchIndex, chRef, remote,
                                              histories, dest, maxCorridorWidthM, param);
      }

      sortDestinationArrivals(summary.vehicles, dest.sortBy);
      const auto tPredict1 = std::chrono::steady_clock::now();
      std::cerr << "[mmlp] destination indexed(csr) vehicles=" << vehicles.size()
                << " pruned=" << pruned << " routable=" << routable.size()
                << " collective=" << collective.size() << " remote=" << remote.size()
                << " filter_edges=" << allowedEdges.size() << " field_nodes=" << field.atNode.size()
                << " collect_ms=" << collectMs << " predict_ms="
                << std::chrono::duration_cast<std::chrono::milliseconds>(tPredict1 - tPredict0)
                       .count()
                << " total_ms="
                << std::chrono::duration_cast<std::chrono::milliseconds>(tPredict1 - t0).count()
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
      appendDestinationCsrBatchWithFallback(summary.vehicles, store, matchIndex, chRef, remote,
                                            histories, dest, maxCorridorWidthM, param);
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

  if (store.hasCsr()) {
    appendDestinationCsrBatchWithFallback(summary.vehicles, store, matchIndex, chRef, perVehicle,
                                          histories, dest, maxCorridorWidthM, param);
  } else {
    std::vector<std::optional<VehicleArrivalResult>> rows(perVehicle.size());
    parallelFor(perVehicle.size(), [&](std::size_t i) {
      const VehicleHistory* hist = findHistory(histories, perVehicle[i].id);
      rows[i] = predictVehicleToDestinationIndexedOrCh(store, matchIndex, chRef, perVehicle[i],
                                                       hist, dest, maxCorridorWidthM, param);
    });
    for (auto& row : rows) {
      if (row) {
        summary.vehicles.push_back(std::move(*row));
      }
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
