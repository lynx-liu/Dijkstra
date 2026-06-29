#pragma once

#include "mmlp/region_loader.hpp"
#include "mmlp/routing.hpp"
#include "mmlp/spatial_index.hpp"
#include "mmlp/types.hpp"

#include <optional>
#include <vector>

namespace mmlp {

struct DestinationQuery {
  double lat = 0.0;
  double lon = 0.0;
  int64_t arriveByUnix = 0;
  VehicleType type = VehicleType::TRUCK;
  ArrivalSortBy sortBy = ArrivalSortBy::DURATION;
};

// Vehicles that can reach destination before arriveByUnix; order controlled by sortBy.
DestinationArrivalSummary predictVehiclesToDestination(
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const GraphContext& routeCtx, const SpatialIndex& matchIndex, const DestinationQuery& dest,
    const PredictParam& param = PredictParam{});

// Index-only: per-vehicle corridors (avoids national bbox when fleet is spread out).
DestinationArrivalSummary predictVehiclesToDestinationIndexed(
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const GraphFileStore& store, const SpatialIndex& matchIndex, const DestinationQuery& dest,
    double maxCorridorWidthM, const PredictParam& param = PredictParam{});

void sortDestinationArrivals(std::vector<VehicleArrivalResult>& rows, ArrivalSortBy sortBy);

std::string graphLocationId(const GraphPosition& pos);

// One vehicle on an already-extracted corridor subgraph (goal must be matchable on routeCtx).
std::optional<VehicleArrivalResult> predictVehicleToDestination(
    const VehicleInfo& vehicle, const VehicleHistory* history, const GraphContext& routeCtx,
    const SpatialIndex& matchIndex, const DestinationQuery& dest, const GraphPosition& goalPos,
    const PredictParam& param = PredictParam{});

}  // namespace mmlp
