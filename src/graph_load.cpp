#include "mmlp/graph_load.hpp"

#include "mmlp/graph_io.hpp"

#include <cstdlib>

namespace mmlp {

bool loadDefaultGraph(MultimodalGraph& graph, const std::string& path, std::string* error) {
  std::string graphPath = path;
  if (graphPath.empty()) {
    if (const char* env = std::getenv("MMLP_GRAPH_PATH")) {
      graphPath = env;
    } else {
      graphPath = "data/graph/china.mmlp.bin";
    }
  }
  return loadGraphFromFile(graphPath, graph, error);
}

}  // namespace mmlp
