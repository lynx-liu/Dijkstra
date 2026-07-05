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
// chOverlayStore: optional national CH overlay for long-haul queries (regional CH may be clipped).
// nationalSnapStore / nationalSnapIndex: national graph for remote vehicle CH routes.
DestinationArrivalSummary predictVehiclesToDestinationIndexed(
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    const GraphFileStore& store, const SpatialIndex& matchIndex, const DestinationQuery& dest,
    double maxCorridorWidthM, const PredictParam& param = PredictParam{},
    const GraphFileStore* chOverlayStore = nullptr,
    const GraphFileStore* nationalSnapStore = nullptr,
    const SpatialIndex* nationalSnapIndex = nullptr);

void sortDestinationArrivals(std::vector<VehicleArrivalResult>& rows, ArrivalSortBy sortBy);

// Cheap straight-line bound before touching the graph.
bool vehicleMayReachDestination(const VehicleInfo& vehicle, const DestinationQuery& dest,
                                const VehicleHistory* history, const PredictParam& param);

std::string graphLocationId(const GraphPosition& pos);

// CH long-haul query on mmap overlay (snap on store, route on chStore).
std::optional<VehicleArrivalResult> predictVehicleToDestinationCh(
    const GraphFileStore& snapStore, const SpatialIndex& snapIndex,
    const GraphFileStore& chStore, const VehicleInfo& vehicle, const VehicleHistory* history,
    const DestinationQuery& dest, const PredictParam& param = PredictParam{});

// One vehicle on an already-extracted corridor subgraph (goal must be matchable on routeCtx).
std::optional<VehicleArrivalResult> predictVehicleToDestination(
    const VehicleInfo& vehicle, const VehicleHistory* history, const GraphContext& routeCtx,
    const SpatialIndex& matchIndex, const DestinationQuery& dest, const GraphPosition& goalPos,
    const PredictParam& param = PredictParam{});

}  // namespace mmlp
