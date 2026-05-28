#pragma once

namespace mmlp {

// Units (see docs/DESIGN.md)
constexpr double kMetersPerSecondFromKmh = 1.0 / 3.6;
constexpr double kDefaultRoadSpeedKmh = 60.0;
constexpr double kDefaultRailSpeedKmh = 80.0;
constexpr double kSpeedBlendAlpha = 0.85;  // weight on historical average vs instantaneous GPS

constexpr double kNoMeetingTime = -1.0;

}  // namespace mmlp
