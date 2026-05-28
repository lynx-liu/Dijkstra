#pragma once

#include "mmlp/types.hpp"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mmlp {

class FleetSpatialIndex {
 public:
  void upsert(const VehicleInfo& vehicle);
  void remove(const std::string& vehicleId);
  void clear();

  std::vector<std::string> queryNearbyIds(double lat, double lon, VehicleType type,
                                          double radiusMeters,
                                          const std::string& excludeId) const;

 private:
  struct Entry {
    double lat = 0.0;
    double lon = 0.0;
    VehicleType type = VehicleType::TRUCK;
  };

  double cellSizeDeg_ = 0.25;
  std::unordered_map<int64_t, std::vector<std::string>> cells_;
  std::unordered_map<std::string, std::pair<int, int>> positions_;
  std::unordered_map<std::string, Entry> vehicles_;

  static int64_t cellKey(int gx, int gy);
  void cellOf(double lat, double lon, int& gx, int& gy) const;
};

}  // namespace mmlp
