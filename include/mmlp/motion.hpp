#pragma once

#include "mmlp/graph.hpp"
#include "mmlp/types.hpp"

namespace mmlp {

double averageHistorySpeedKmh(const VehicleHistory* history);

double resolveSpeedKmh(const VehicleInfo& vehicle, const VehicleHistory* history,
                      const Edge* edgeHint, VehicleType type);

double speedMsFromKmh(double kmh);

// Travel time (seconds) for distance (meters) at constant speed with truck rest model.
double travelTimeSeconds(double distanceMeters, double speedMs, VehicleType type,
                         const PredictParam& param);

double edgeEffectiveSpeedMs(const AdjacencyEdge& edge, VehicleType type, double vehicleSpeedMs);

}  // namespace mmlp
