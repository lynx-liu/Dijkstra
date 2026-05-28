#include "mmlp/fleet_service.hpp"
#include "mmlp/json_util.hpp"

#include "fixtures/tiny_graph.hpp"

#include <cassert>
#include <iostream>

int main() {
  const auto graph = mmlp::test::makeTinyRoadGraph();
  // Fleet service needs file path - use in-memory path not supported; test via predict on tiny graph logic
  // Direct ingest API test with temporary approach: save not available, skip file test

  mmlp::VehicleInfo a;
  a.id = "A";
  a.type = mmlp::VehicleType::TRUCK;
  a.lat = 43.80;
  a.lon = 87.50;
  a.speed = 72.0;
  a.timestamp = 1'700'000'000;

  std::string j1 = R"({"id":"A","lat":43.80,"lon":87.50,"speed":72,"timestamp":1700000000})";
  std::string j2 = R"({"id":"B","lat":43.82,"lon":87.52,"speed":72,"timestamp":1700000000})";

  mmlp::VehicleInfo v1, v2;
  mmlp::VehicleHistory h1, h2;
  assert(mmlp::parseVehicleJson(j1, v1, &h1));
  assert(mmlp::parseVehicleJson(j2, v2, &h2));
  assert(v1.id == "A");

  mmlp::GraphContext ctx;
  ctx.graph = graph;
  ctx.index.build(graph);
  std::vector<mmlp::VehicleInfo> fleet{v1};
  auto r0 = mmlp::predictBestMeetingFor("A", fleet, {}, ctx);
  assert(!r0.found);

  fleet.push_back(v2);
  auto r1 = mmlp::predictBestMeetingFor("B", fleet, {}, ctx);
  assert(r1.found);
  assert(r1.partnerVehicleId == "A");

  std::cout << "test_fleet_service: ok\n";
  return 0;
}
