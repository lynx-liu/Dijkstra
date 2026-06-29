#include "mmlp/arrival.hpp"

#include "mmlp/geo.hpp"
#include "mmlp/matching.hpp"
#include "mmlp/meeting.hpp"
#include "mmlp/motion.hpp"
#include "mmlp/routing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <sstream>
#include <thread>
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
  if (routeParam.maxVisitedNodes > 0 && routeParam.maxVisitedNodes < 250000) {
    routeParam.maxVisitedNodes = 250000;
  }

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

  summary.vehicles.reserve(vehicles.size());
  const std::size_t maxWorkers =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::thread::hardware_concurrency()));
  const bool parallel = vehicles.size() > 2 && maxWorkers > 1;
  if (parallel) {
    std::vector<std::future<std::optional<VehicleArrivalResult>>> futures;
    futures.reserve(vehicles.size());
    for (const auto& vehicle : vehicles) {
      const VehicleHistory* hist = findHistory(histories, vehicle.id);
      futures.push_back(std::async(
          std::launch::async,
          [vehicle, hist, &routeCtx, &matchIndex, dest, goalPos, param]() {
            return predictVehicleToDestination(vehicle, hist, routeCtx, matchIndex, dest, goalPos,
                                             param);
          }));
    }
    for (auto& fut : futures) {
      if (auto row = fut.get()) {
        summary.vehicles.push_back(std::move(*row));
      }
    }
  } else {
    for (const auto& vehicle : vehicles) {
      const VehicleHistory* hist = findHistory(histories, vehicle.id);
      if (auto row = predictVehicleToDestination(vehicle, hist, routeCtx, matchIndex, dest,
                                                 goalPos, param)) {
        summary.vehicles.push_back(std::move(*row));
      }
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

  GraphContext destCtx;
  std::string destErr;
  const std::vector<VehicleInfo> destOnly = {destProbe};
  if (!extractGraphContextForDestinationIndexed(store, matchIndex, destOnly, 8000.0, destCtx,
                                                &destErr)) {
    std::cerr << "[mmlp] destination snap extract failed: " << destErr << "\n" << std::flush;
    return summary;
  }

  const GraphPosition goalPos =
      matchVehicleToGraph(destCtx.graph, destProbe, &matchIndex);
  summary.locationId = graphLocationId(goalPos);
  if (!goalPos.valid) {
    return summary;
  }

  const auto t0 = std::chrono::steady_clock::now();
  std::size_t pruned = 0;
  std::size_t routed = 0;
  summary.vehicles.reserve(vehicles.size());

  for (const auto& vehicle : vehicles) {
    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    const double speedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, hist, nullptr, vehicle.type));
    if (!vehicleMayReachDestination(vehicle, dest, speedMs)) {
      ++pruned;
      continue;
    }

    const double dist =
        haversineMeters({vehicle.lat, vehicle.lon}, {dest.lat, dest.lon});
    const double corridorW =
        dist > 40000.0 ? maxCorridorWidthM
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
      ++routed;
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  std::cerr << "[mmlp] destination indexed vehicles=" << vehicles.size() << " pruned=" << pruned
            << " routed=" << routed << " reachable=" << summary.vehicles.size()
            << " total_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n"
            << std::flush;

  sortDestinationArrivals(summary.vehicles, dest.sortBy);
  return summary;
}

}  // namespace mmlp
