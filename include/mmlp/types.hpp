#pragma once

#include "mmlp/constants.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mmlp {

enum class VehicleType { TRUCK, TRAIN };

enum class EdgeType : int { ROAD = 0, RAIL = 1 };

enum class NodeKind : int {
  ROAD_JUNCTION = 0,
  RAIL_STATION = 1,
  HUB = 2,
};

struct Edge {
  int64_t id = 0;
  int64_t from = 0;
  int64_t to = 0;
  EdgeType type = EdgeType::ROAD;
  double length = 0.0;       // meters
  double speedLimit = 0.0;   // km/h; 0 => use mode default
};

struct Node {
  int64_t id = 0;
  double lat = 0.0;
  double lon = 0.0;
  NodeKind kind = NodeKind::ROAD_JUNCTION;
};

struct VehicleInfo {
  std::string id;
  VehicleType type = VehicleType::TRUCK;

  double lat = 0.0;
  double lon = 0.0;

  double speed = 0.0;    // km/h from GPS
  double heading = 0.0;  // degrees, 0 = north, clockwise
  int64_t timestamp = 0; // Unix seconds (UTC)
};

struct VehicleHistory {
  std::string id;
  std::vector<double> speedSamples;  // km/h
};

struct PredictParam {
  double meetDistance = 300.0;  // meters
  double maxTime = 172800.0;    // seconds (2 days)
  std::size_t maxVisitedNodes = 400000;  // 0 = unlimited Dijkstra expansion

  double truckCycle = 4.0 * 3600.0;
  double truckRest = 30.0 * 60.0;
};

struct MeetingResult {
  std::string vehicleA;
  std::string vehicleB;

  double meetTime = kNoMeetingTime;  // Unix seconds; kNoMeetingTime if unreachable

  double lat = 0.0;
  double lon = 0.0;

  std::string locationId;
  double distance = 0.0;  // meters between vehicles at meetTime (predicted)
};

// Fastest meeting for one focal vehicle among a fleet.
struct FocalBestMeeting {
  bool found = false;
  std::string focalVehicleId;
  std::string partnerVehicleId;  // empty when found == false

  double meetTime = kNoMeetingTime;  // Unix seconds
  double meetDuration = 0.0;       // seconds from aligned start (max(tA,tB))

  double lat = 0.0;
  double lon = 0.0;
  std::string locationId;
};

}  // namespace mmlp
