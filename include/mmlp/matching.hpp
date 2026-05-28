#pragma once

#include "mmlp/graph.hpp"
#include "mmlp/routing.hpp"
#include "mmlp/types.hpp"

namespace mmlp {

constexpr double kMaxSnapDistanceMeters = 8000.0;

class SpatialIndex;

GraphPosition matchVehicleToGraph(const MultimodalGraph& graph, const VehicleInfo& vehicle,
                                  const SpatialIndex* index = nullptr);

}  // namespace mmlp
