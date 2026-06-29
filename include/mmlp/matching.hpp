#pragma once

#include "mmlp/graph.hpp"
#include "mmlp/routing.hpp"
#include "mmlp/types.hpp"

namespace mmlp {

class GraphFileStore;
class SpatialIndex;

constexpr double kMaxSnapDistanceMeters = 8000.0;

class SpatialIndex;

GraphPosition matchVehicleToGraph(const MultimodalGraph& graph, const VehicleInfo& vehicle,
                                  const SpatialIndex* index = nullptr);

GraphPosition matchVehicleToGraphIndexed(const GraphFileStore& store, const SpatialIndex& index,
                                         const VehicleInfo& vehicle);

}  // namespace mmlp
