#include "fixtures/tiny_graph.hpp"

#include "mmlp/predict.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

mmlp::VehicleInfo truckAtNode(const char* id, int64_t nodeId, double speedKmh, int64_t ts) {
  const auto graph = mmlp::test::makeTinyRoadGraph();
  const mmlp::Node* node = graph.findNode(nodeId);
  assert(node != nullptr);
  mmlp::VehicleInfo v;
  v.id = id;
  v.type = mmlp::VehicleType::TRUCK;
  v.lat = node->lat;
  v.lon = node->lon;
  v.speed = speedKmh;
  v.timestamp = ts;
  return v;
}

void testPairMeetAtMiddle() {
  const auto graph = mmlp::test::makeTinyRoadGraph();
  const int64_t t0 = 1'700'000'000;

  auto a = truckAtNode("A", 1, 72.0, t0);
  auto b = truckAtNode("B", 3, 72.0, t0);

  std::vector<mmlp::VehicleInfo> fleet{a, b};
  std::vector<mmlp::VehicleHistory> histories;

  const auto meetings = mmlp::predictMeetings(fleet, histories, graph);
  assert(meetings.size() == 1);
  assert(meetings[0].vehicleA == "A" || meetings[0].vehicleB == "A");

  const double duration = meetings[0].meetTime - static_cast<double>(t0);
  assert(std::abs(duration - 50.0) < 2.0);

  const mmlp::Node* n2 = graph.findNode(2);
  assert(n2 != nullptr);
  const double dLat = meetings[0].lat - n2->lat;
  const double dLon = meetings[0].lon - n2->lon;
  assert(std::sqrt(dLat * dLat + dLon * dLon) < 0.002);
}

void testFocalBestAmongThree() {
  const auto graph = mmlp::test::makeTinyRoadGraph();
  const int64_t t0 = 1'700'000'000;

  auto focal = truckAtNode("focal", 1, 72.0, t0);
  auto coLocated = truckAtNode("near", 1, 72.0, t0);
  auto far = truckAtNode("far", 3, 72.0, t0);

  std::vector<mmlp::VehicleInfo> fleet{focal, coLocated, far};
  std::vector<mmlp::VehicleHistory> histories;

  const auto best = mmlp::predictBestMeetingFor("focal", fleet, histories, graph);
  assert(best.found);
  assert(best.partnerVehicleId == "near");
  assert(best.meetDuration < 5.0);
}

void testCurrentVehicleApi() {
  const auto graph = mmlp::test::makeTinyRoadGraph();
  const int64_t t0 = 1'700'000'000;

  std::vector<mmlp::VehicleInfo> fleet{
      truckAtNode("current", 1, 72.0, t0),
      truckAtNode("other", 3, 72.0, t0),
  };

  const auto best =
      mmlp::predictBestMeetingForCurrent(fleet, {}, graph);
  assert(best.found);
  assert(best.partnerVehicleId == "other");
}

}  // namespace

int main() {
  testPairMeetAtMiddle();
  testFocalBestAmongThree();
  testCurrentVehicleApi();
  std::cout << "test_meeting: ok\n";
  return 0;
}
