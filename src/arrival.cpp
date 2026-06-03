#include "mmlp/arrival.hpp"

#include "mmlp/geo.hpp"
#include "mmlp/matching.hpp"
#include "mmlp/meeting.hpp"
#include "mmlp/motion.hpp"
#include "mmlp/routing.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

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

std::string locationIdForPosition(const GraphPosition& pos) {
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

double polylineLengthMeters(const RoutePolyline& route) {
  double sum = 0.0;
  for (std::size_t i = 1; i < route.points.size(); ++i) {
    sum += haversineMeters(route.points[i - 1], route.points[i]);
  }
  return sum;
}

void sortArrivals(std::vector<VehicleArrivalResult>& rows, ArrivalSortBy sortBy) {
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

}  // namespace

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
  summary.locationId = locationIdForPosition(goalPos);
  if (!goalPos.valid) {
    return summary;
  }

  const LatLon goalLoc{dest.lat, dest.lon};
  summary.vehicles.reserve(vehicles.size());

  for (const auto& vehicle : vehicles) {
    if (vehicle.type != dest.type) {
      continue;
    }

    VehicleArrivalResult row;
    row.vehicleId = vehicle.id;

    const VehicleHistory* hist = findHistory(histories, vehicle.id);
    const PreparedVehicle prepared =
        prepareVehicle(graph, vehicle, hist, vehicle.timestamp, param, &matchIndex);
    if (!prepared.valid) {
      continue;
    }

    const double maxHorizon =
        std::min(param.maxTime, static_cast<double>(dest.arriveByUnix - vehicle.timestamp));
    if (maxHorizon < 1.0) {
      continue;
    }

    const TimeField field = computeTimeField(graph, prepared.position, prepared.speedMs,
                                             vehicle.type, param, maxHorizon, &goalLoc);
    const double travel =
        timeAtPosition(graph, field, goalPos, prepared.speedMs, vehicle.type, param);
    if (travel >= kInfTime / 2.0 || travel > maxHorizon + 1e-6) {
      continue;
    }

    const double eta = static_cast<double>(vehicle.timestamp) + travel;
    if (eta > static_cast<double>(dest.arriveByUnix) + 1e-6) {
      continue;
    }

    row.reachable = true;
    row.travelDurationSec = travel;
    row.etaUnix = eta;
    row.route = computeRoutePolyline(graph, prepared.position, goalPos, prepared.speedMs,
                                     vehicle.type, param, maxHorizon);
    row.routeDistanceM = polylineLengthMeters(row.route);
    summary.vehicles.push_back(std::move(row));
  }

  sortArrivals(summary.vehicles, dest.sortBy);
  return summary;
}

}  // namespace mmlp
