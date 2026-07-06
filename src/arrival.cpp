#include "mmlp/arrival.hpp"

#include "mmlp/ch_graph.hpp"
#include "mmlp/geo.hpp"
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
      row.routeDistanceM = polylineLengthMeters(path.polyline);
      row.route = path.polyline;
      simplifyRoutePolyline(row.route, 120);
      return row;
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

  std::vector<std::size_t> needToward;
  needToward.reserve(jobs.size());
  for (std::size_t i = 0; i < jobs.size(); ++i) {
    if (jobs[i].portals.empty()) {
      needToward.push_back(i);
    }
  }
  if (!needToward.empty()) {
    std::vector<LatLon> vehicleLl;
    std::vector<double> towardGeo;
    vehicleLl.reserve(needToward.size());
    towardGeo.reserve(needToward.size());
    for (std::size_t i : needToward) {
      vehicleLl.push_back({jobs[i].vehicle->lat, jobs[i].vehicle->lon});
      const double distToGoal =
          haversineMeters({jobs[i].vehicle->lat, jobs[i].vehicle->lon}, {dest.lat, dest.lon});
      towardGeo.push_back(distToGoal > 180000.0 ? distToGoal * 0.25 + 15000.0
                                                  : std::min(80000.0, distToGoal * 0.25 + 15000.0));
    }
    const std::vector<std::vector<int64_t>> towardPortals =
        collectOverlayChNodesTowardBatch(chStore, ch, hwyCsr, vehicleLl, goalNodes, towardGeo, 8);
    for (std::size_t j = 0; j < needToward.size(); ++j) {
      const std::size_t i = needToward[j];
      for (int64_t node : towardPortals[j]) {
        jobs[i].portals.push_back(node);
      }
      if (jobs[i].portals.empty()) {
        const double distToGoal =
            haversineMeters({jobs[i].vehicle->lat, jobs[i].vehicle->lon}, {dest.lat, dest.lon});
        jobs[i].portals = collectOverlayChNodesToward(
            chStore, ch, hwyCsr, jobs[i].vehicle->lat, jobs[i].vehicle->lon, goalNodes,
            distToGoal + 35000.0, 8);
      }
      if (jobs[i].portals.empty()) {
        jobs[i].portals = collectOverlayChNodes(snapStore, chStore, ch, hwyCsr, matchIndex,
                                                jobs[i].vehicle->lat, jobs[i].vehicle->lon,
                                                jobs[i].vehicle->type, 50000.0, 8);
      }
    }
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

  if (!metroJobs.empty()) {
    std::unordered_set<int64_t> targetNodes;
    targetNodes.reserve(metroJobs.size() * 4);
    double metroSpanM = 0.0;
    double groupSpeedMs = metroJobs.front().speedMs;
    double groupMaxHorizon = 0.0;
    for (const auto& job : metroJobs) {
      metroSpanM = std::max(
          metroSpanM, haversineMeters({job.vehicle->lat, job.vehicle->lon}, {dest.lat, dest.lon}));
      groupSpeedMs = std::min(groupSpeedMs, job.speedMs);
      groupMaxHorizon = std::max(groupMaxHorizon, job.maxHorizon);
      for (int64_t portal : job.portals) {
        targetNodes.insert(portal);
      }
    }
    const double maxRadiusM = metroSpanM + 35000.0;
    const uint64_t hwyKey = destRouteCacheKey(dest, 0.0) ^ static_cast<uint64_t>(goalPos.nodeId);
    std::shared_ptr<CachedHwyDistField> hwyCached;
    {
      std::shared_lock<std::shared_mutex> read(hwyDistFieldCacheMu);
      if (hwyDistFieldCache && hwyDistFieldCache->key == hwyKey &&
          hwyDistFieldCache->goalNodeId == goalPos.nodeId &&
          hwyDistFieldCache->distByRow.size() == hwyCsr.nodeCount()) {
        hwyCached = hwyDistFieldCache;
      }
    }
    if (!hwyCached) {
      std::vector<double> distByRow = computeRoutedDistFromGoalCsrDense(
          chStore, hwyCsr, goalPos.nodeId, groupSpeedMs, dest.type, routeParam, groupMaxHorizon,
          &targetNodes, &goalLl, maxRadiusM);
      auto cached = std::make_shared<CachedHwyDistField>();
      cached->key = hwyKey;
      cached->goalNodeId = goalPos.nodeId;
      cached->distByRow = std::move(distByRow);
      std::unique_lock<std::shared_mutex> write(hwyDistFieldCacheMu);
      if (hwyDistFieldCache && hwyDistFieldCache->key == hwyKey) {
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

    for (const auto& job : metroJobs) {
      if (auto row = buildRowFromPortals(job, nodeTime)) {
        results.push_back(std::move(*row));
      }
    }

    if (!remoteJobs.empty()) {
      std::vector<std::optional<VehicleArrivalResult>> remoteRows(remoteJobs.size());
      parallelFor(remoteJobs.size(), [&](std::size_t i) {
        const VehicleJob& job = remoteJobs[i];
        const double distM =
            haversineMeters({job.vehicle->lat, job.vehicle->lon}, {dest.lat, dest.lon});
        std::unordered_set<int64_t> targets(job.portals.begin(), job.portals.end());
        const std::vector<double> remoteDistByRow = computeRoutedDistFromGoalCsrDense(
            chStore, hwyCsr, goalPos.nodeId, job.speedMs, dest.type, routeParam, job.maxHorizon,
            &targets, &goalLl, distM + 35000.0);
        auto remoteNodeTime = [&](int64_t nodeId) -> double {
          const int r = hwyCsr.nodeRow(chStore, nodeId);
          if (r < 0) {
            return kInfTime;
          }
          return remoteDistByRow[static_cast<std::size_t>(r)];
        };
        remoteRows[i] = buildRowFromPortals(job, remoteNodeTime);
      });
      for (auto& row : remoteRows) {
        if (row) {
          results.push_back(std::move(*row));
        }
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
  constexpr double kMetroDistM = 180000.0;
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
    for (int64_t fromNode : fromNodes) {
      for (int64_t toNode : goalNodes) {
        const RouteToGoal path =
            ch.query(chStore, hwyCsr, fromNode, toNode, vehicle.type, routeParam, job.maxHorizon);
        if (path.polyline.points.size() >= 2 && path.travelTimeSec < bestTravel) {
          bestTravel = path.travelTimeSec;
          best = path.polyline;
        }
      }
    }
    if (best.points.size() < 2) {
      const VehicleHistory* hist = findHistory(histories, vehicle.id);
      if (auto chRow = predictVehicleToDestinationCh(snapStore, matchIndex, chStore, vehicle, hist,
                                                     dest, param)) {
        if (chRow->route.points.size() >= 2) {
          rows[j] = chRow->route;
        }
      }
      return;
    }
    RoutePolyline route;
    appendRoutePt(route, {vehicle.lat, vehicle.lon});
    appendPolylinePoints(route, best);
    appendRoutePt(route, {dest.lat, dest.lon});
    if (route.points.size() >= 2) {
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
    ThreadPool& pool = ThreadPool::instance();
    std::future<std::vector<VehicleArrivalResult>> etaFuture = pool.submit([&]() {
      return predictVehiclesToDestinationHwyCsrBatch(store, matchIndex, chRef, vehicles, histories,
                                                     dest, param, nationalSnapIndex,
                                                     nationalSnapStore);
    });
    std::future<std::unordered_map<std::string, RoutePolyline>> routeFuture;
    std::future<std::unordered_map<std::string, RoutePolyline>> overlayFuture = pool.submit([&]() {
      return computeChOverlayRoutesByVehicleId(store, matchIndex, chRef, vehicles, histories, dest,
                                               param);
    });
    if (store.hasCsr()) {
      routeFuture = pool.submit([&]() {
        return computeRegionalRoutesByVehicleId(store, matchIndex, vehicles, histories, dest,
                                                param);
      });
    }
    batch = etaFuture.get();
    if (routeFuture.valid()) {
      const std::unordered_map<std::string, RoutePolyline> routes = routeFuture.get();
      for (auto& row : batch) {
        const auto it = routes.find(row.vehicleId);
        if (it != routes.end() && it->second.points.size() >= 2) {
          row.route = it->second;
          row.routeDistanceM = polylineLengthMeters(row.route);
        }
      }
    }
    if (overlayFuture.valid()) {
      const std::unordered_map<std::string, RoutePolyline> overlayRoutes = overlayFuture.get();
      for (auto& row : batch) {
        if (row.route.points.size() >= 2) {
          continue;
        }
        const auto it = overlayRoutes.find(row.vehicleId);
        if (it != overlayRoutes.end() && it->second.points.size() >= 2) {
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

DestinationArrivalSummary predictVehiclesToDestinationIndexed(
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const GraphFileStore& store, const SpatialIndex& matchIndex, const DestinationQuery& dest,
    double maxCorridorWidthM, const PredictParam& param, const GraphFileStore* chOverlayStore,
    const GraphFileStore* nationalSnapStore, const SpatialIndex* nationalSnapIndex) {
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
    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, hist, nullptr, vehicle.type));
    if (!mayReachWithSpeed(vehicle, dest, speedMs)) {
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
      ThreadPool& pool = ThreadPool::instance();
      for (const auto& cluster : clusters) {
        futures.push_back(pool.submit([&, cluster]() {
          return predictVehiclesToDestinationIndexed(cluster, histories, store, matchIndex, dest,
                                                   maxCorridorWidthM, param, chOverlayStore,
                                                   nationalSnapStore, nationalSnapIndex);
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
        ThreadPool& pool = ThreadPool::instance();
        for (const auto& group : groups) {
          futures.push_back(pool.submit([&, group]() {
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
