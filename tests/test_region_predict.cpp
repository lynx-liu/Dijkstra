#include "mmlp/predict.hpp"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
  const char* graphPath = std::getenv("MMLP_GRAPH_PATH");
  if (graphPath == nullptr) {
    graphPath = "data/graph/china.mmlp.bin";
  }

  std::vector<mmlp::VehicleInfo> fleet;
  mmlp::VehicleInfo a;
  a.id = "A";
  a.type = mmlp::VehicleType::TRUCK;
  a.lat = 43.9055361;
  a.lon = 87.4561254;
  a.speed = 72.0;
  a.timestamp = 1'700'000'000;

  mmlp::VehicleInfo b = a;
  b.id = "B";
  b.lat = 43.913164;
  b.lon = 87.4920121;

  fleet.push_back(a);
  fleet.push_back(b);

  std::string error;
  const auto best = mmlp::predictBestMeetingFromFile(graphPath, "A", fleet, {}, {}, 80000.0, &error);
  if (!best.found) {
    std::cerr << "predict failed: " << error << "\n";
    return 1;
  }
  assert(best.partnerVehicleId == "B");
  assert(best.meetDuration > 0.0);
  std::cout << "test_region_predict: ok duration=" << best.meetDuration
            << " lat=" << best.lat << " lon=" << best.lon << "\n";
  return 0;
}
