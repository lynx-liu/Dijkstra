#pragma once

#include "mmlp/graph.hpp"
#include "mmlp/spatial_index.hpp"

#include <string>
#include <unordered_set>
#include <vector>

namespace mmlp {

bool loadGraphFromFile(const std::string& path, MultimodalGraph& graph, std::string* error = nullptr);

// Load only edges listed in edgeIds using .eidx/.nidx sidecars (fast subset).
bool loadGraphSubsetFromFile(const std::string& binPath, const std::unordered_set<int64_t>& edgeIds,
                             MultimodalGraph& graph, std::string* error = nullptr);

bool loadSpatialIndexFromFile(const std::string& sidxPath, SpatialIndex& index,
                              std::string* error = nullptr);

bool graphAuxiliaryReady(const std::string& binPath);

}  // namespace mmlp
