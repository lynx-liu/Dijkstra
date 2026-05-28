#include "mmlp/fleet_index.hpp"

#include "mmlp/geo.hpp"

#include <algorithm>
#include <cmath>

namespace mmlp {

int64_t FleetSpatialIndex::cellKey(int gx, int gy) {
  return (static_cast<int64_t>(gx) << 32) ^ static_cast<uint32_t>(gy);
}

void FleetSpatialIndex::cellOf(double lat, double lon, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor(lat / cellSizeDeg_));
  gy = static_cast<int>(std::floor(lon / cellSizeDeg_));
}

void FleetSpatialIndex::upsert(const VehicleInfo& vehicle) {
  int gx = 0;
  int gy = 0;
  cellOf(vehicle.lat, vehicle.lon, gx, gy);
  const int64_t key = cellKey(gx, gy);

  const auto prev = positions_.find(vehicle.id);
  if (prev != positions_.end()) {
    if (prev->second.first == gx && prev->second.second == gy) {
      vehicles_[vehicle.id] = Entry{vehicle.lat, vehicle.lon, vehicle.type};
      return;
    }
    const int64_t oldKey = cellKey(prev->second.first, prev->second.second);
    auto& bucket = cells_[oldKey];
    bucket.erase(std::remove(bucket.begin(), bucket.end(), vehicle.id), bucket.end());
  }

  cells_[key].push_back(vehicle.id);
  positions_[vehicle.id] = {gx, gy};
  vehicles_[vehicle.id] = Entry{vehicle.lat, vehicle.lon, vehicle.type};
}

void FleetSpatialIndex::remove(const std::string& vehicleId) {
  const auto prev = positions_.find(vehicleId);
  if (prev == positions_.end()) {
    return;
  }
  const int64_t oldKey = cellKey(prev->second.first, prev->second.second);
  auto& bucket = cells_[oldKey];
  bucket.erase(std::remove(bucket.begin(), bucket.end(), vehicleId), bucket.end());
  positions_.erase(prev);
  vehicles_.erase(vehicleId);
}

void FleetSpatialIndex::clear() {
  cells_.clear();
  positions_.clear();
  vehicles_.clear();
}

std::vector<std::string> FleetSpatialIndex::queryNearbyIds(double lat, double lon,
                                                           VehicleType type,
                                                           double radiusMeters,
                                                           const std::string& excludeId) const {
  int gx = 0;
  int gy = 0;
  cellOf(lat, lon, gx, gy);

  const int cellRadius =
      std::max(1, static_cast<int>(std::ceil(radiusMeters / (cellSizeDeg_ * 111000.0))));

  const LatLon query{lat, lon};
  std::vector<std::pair<double, std::string>> ranked;
  ranked.reserve(128);

  for (int dx = -cellRadius; dx <= cellRadius; ++dx) {
    for (int dy = -cellRadius; dy <= cellRadius; ++dy) {
      const auto it = cells_.find(cellKey(gx + dx, gy + dy));
      if (it == cells_.end()) {
        continue;
      }
      for (const std::string& id : it->second) {
        if (id == excludeId) {
          continue;
        }
        const auto vit = vehicles_.find(id);
        if (vit == vehicles_.end() || vit->second.type != type) {
          continue;
        }
        const double dist = haversineMeters(query, {vit->second.lat, vit->second.lon});
        if (dist <= radiusMeters) {
          ranked.emplace_back(dist, id);
        }
      }
    }
  }

  std::sort(ranked.begin(), ranked.end());
  std::vector<std::string> out;
  out.reserve(ranked.size());
  for (const auto& item : ranked) {
    out.push_back(item.second);
  }
  return out;
}

}  // namespace mmlp
