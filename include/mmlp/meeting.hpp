#pragma once

#include "mmlp/graph.hpp"
#include "mmlp/routing.hpp"
#include "mmlp/types.hpp"

namespace mmlp {

struct PreparedVehicle {
  VehicleInfo info;
  GraphPosition position;
  double speedMs = 0.0;
  bool valid = false;
};

class SpatialIndex;

PreparedVehicle prepareVehicle(const MultimodalGraph& graph, const VehicleInfo& vehicle,
                               const VehicleHistory* history, int64_t alignTime,
                               const PredictParam& param,
                               const SpatialIndex* index = nullptr);

// Earliest meeting between two prepared vehicles after time alignment to alignTime.
// Returns false if no meeting within maxTime.
bool computePairwiseMeeting(const MultimodalGraph& graph, const PreparedVehicle& a,
                            const PreparedVehicle& b, int64_t alignTime,
                            const PredictParam& param, MeetingResult& out);

bool computePairwiseMeeting(const MultimodalGraph& graph, const PreparedVehicle& a,
                            const PreparedVehicle& b, const TimeField& fieldA,
                            const TimeField& fieldB, int64_t alignTime,
                            const PredictParam& param, MeetingResult& out);

// Online-optimized: bidirectional search + long-range midpoint estimate (no full-graph scan).
bool computePairwiseMeetingFast(const MultimodalGraph& graph, const PreparedVehicle& a,
                                const PreparedVehicle& b, int64_t alignTime,
                                const PredictParam& param, MeetingResult& out);

}  // namespace mmlp
