#pragma once

#include "mmlp/graph.hpp"

#include <string>

namespace mmlp {

// Load binary graph; if path empty, uses MMLP_GRAPH_PATH or data/graph/china.mmlp.bin.
bool loadDefaultGraph(MultimodalGraph& graph, const std::string& path = {},
                      std::string* error = nullptr);

}  // namespace mmlp
