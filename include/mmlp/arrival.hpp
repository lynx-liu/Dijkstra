#pragma once

#include "mmlp/region_loader.hpp"
#include "mmlp/spatial_index.hpp"
#include "mmlp/types.hpp"

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

}  // namespace mmlp
