#include "fixtures/tiny_graph.hpp"

#include "mmlp/predict.hpp"

#include <cassert>
#include <iostream>

int main() {
  const auto graph = mmlp::test::makeTinyRoadGraph();
  assert(graph.nodes().size() == 3);
  assert(graph.edges().size() == 2);
  assert(graph.neighbors(2).size() == 2);

  const std::vector<mmlp::VehicleInfo> vehicles;
  const std::vector<mmlp::VehicleHistory> histories;
  const auto results = mmlp::predictMeetings(vehicles, histories, graph);
  assert(results.empty());

  std::cout << "test_fixture: ok\n";
  return 0;
}
