#include "mmlp/motion.hpp"

#include "mmlp/constants.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace mmlp {

double averageHistorySpeedKmh(const VehicleHistory* history) {
  if (history == nullptr || history->speedSamples.empty()) {
    return 0.0;
  }
  const double sum = std::accumulate(history->speedSamples.begin(), history->speedSamples.end(), 0.0);
  return sum / static_cast<double>(history->speedSamples.size());
}

double resolveSpeedKmh(const VehicleInfo& vehicle, const VehicleHistory* history,
                      const Edge* edgeHint, VehicleType type) {
  const double hist = averageHistorySpeedKmh(history);
  double gps = vehicle.speed;
  if (gps <= 0.0) {
    gps = 0.0;
  }

  double kmh = 0.0;
  if (hist > 0.0 && gps > 0.0) {
    kmh = kSpeedBlendAlpha * hist + (1.0 - kSpeedBlendAlpha) * gps;
  } else if (hist > 0.0) {
    kmh = hist;
  } else if (gps > 0.0) {
    kmh = gps;
  }

  if (kmh <= 0.0) {
    if (edgeHint != nullptr && edgeHint->speedLimit > 0.0) {
      kmh = edgeHint->speedLimit;
    } else {
      kmh = (type == VehicleType::TRAIN) ? kDefaultRailSpeedKmh : kDefaultRoadSpeedKmh;
    }
    kmh *= 0.8;
  }

  if (edgeHint != nullptr && edgeHint->speedLimit > 0.0) {
    kmh = std::min(kmh, edgeHint->speedLimit);
  }
  return std::max(kmh, 1.0);
}

double speedMsFromKmh(double kmh) { return kmh * kMetersPerSecondFromKmh; }

double travelTimeSeconds(double distanceMeters, double speedMs, VehicleType type,
                         const PredictParam& param) {
  if (distanceMeters <= 0.0 || speedMs <= 0.0) {
    return 0.0;
  }
  const double drive = distanceMeters / speedMs;
  if (type != VehicleType::TRUCK) {
    return drive;
  }
  const int rests =
      static_cast<int>(std::floor(drive / param.truckCycle));
  return drive + rests * param.truckRest;
}

double edgeEffectiveSpeedMs(const AdjacencyEdge& edge, VehicleType type, double vehicleSpeedMs) {
  if (edge.speedLimit > 0.0) {
    const double cap = edge.speedLimit * kMetersPerSecondFromKmh;
    return std::min(vehicleSpeedMs, cap);
  }
  (void)type;
  return vehicleSpeedMs;
}

}  // namespace mmlp
