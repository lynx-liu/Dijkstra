#include "mmlp/graph_io.hpp"

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <string>

int main() {
  const char* root = std::getenv("MMLP_ROOT");
  const std::string base = root ? std::string(root) : ".";
  const std::string binPath = base + "/tests/fixtures/sample.mmlp.bin";
  const std::string osmPath = base + "/tests/fixtures/sample.osm";

  const std::string cmd =
      "python3 " + base + "/tools/osm_to_graph.py -i " + osmPath + " -o " + binPath;
  if (std::system(cmd.c_str()) != 0) {
    std::cerr << "failed to build sample graph\n";
    return 1;
  }

  mmlp::MultimodalGraph graph;
  std::string error;
  assert(mmlp::loadGraphFromFile(binPath, graph, &error));
  assert(graph.nodes().size() >= 3);
  assert(graph.edges().size() >= 2);

  std::cout << "test_graph_io: ok nodes=" << graph.nodes().size()
            << " edges=" << graph.edges().size() << "\n";
  return 0;
}
