#pragma once

#include "mmlp/graph.hpp"
#include "mmlp/region_loader.hpp"
#include "mmlp/spatial_index.hpp"
#include "mmlp/types.hpp"

#include <string>
#include <vector>

namespace mmlp {

// Pairwise earliest meetings for all fleet pairs within maxTime.
std::vector<MeetingResult> predictMeetings(const std::vector<VehicleInfo>& vehicles,
                                           const std::vector<VehicleHistory>& histories,
                                           const MultimodalGraph& graph,
                                           const PredictParam& param = PredictParam{});

std::vector<MeetingResult> predictMeetings(const std::vector<VehicleInfo>& vehicles,
                                           const std::vector<VehicleHistory>& histories,
                                           const GraphContext& ctx,
                                           const PredictParam& param = PredictParam{});

// For the focal vehicle: fastest meeting with any other vehicle in the fleet.
FocalBestMeeting predictBestMeetingFor(const std::string& focalVehicleId,
                                       const std::vector<VehicleInfo>& fleet,
                                       const std::vector<VehicleHistory>& histories,
                                       const MultimodalGraph& graph,
                                       const PredictParam& param = PredictParam{});

FocalBestMeeting predictBestMeetingFor(const std::string& focalVehicleId,
                                       const std::vector<VehicleInfo>& fleet,
                                       const std::vector<VehicleHistory>& histories,
                                       const GraphContext& ctx,
                                       const PredictParam& param = PredictParam{});

FocalBestMeeting predictBestMeetingFor(const std::string& focalVehicleId,
                                       const std::vector<VehicleInfo>& fleet,
                                       const std::vector<VehicleHistory>& histories,
                                       const GraphContext& routeCtx,
                                       const SpatialIndex& matchIndex,
                                       const PredictParam& param = PredictParam{});

// Uses computePairwiseMeetingFast for sub-second online latency.
FocalBestMeeting predictBestMeetingForOnline(const std::string& focalVehicleId,
                                             const std::vector<VehicleInfo>& fleet,
                                             const std::vector<VehicleHistory>& histories,
                                             const GraphContext& routeCtx,
                                             const SpatialIndex& matchIndex,
                                             const PredictParam& param = PredictParam{});

// Lead vehicle is fleet[0]: meeting with each other vehicle, sorted by meetDuration ascending.
std::vector<FocalBestMeeting> predictMeetingsWithLead(
    const std::vector<VehicleInfo>& fleet, const std::vector<VehicleHistory>& histories,
    const GraphContext& routeCtx, const SpatialIndex& matchIndex,
    const PredictParam& param = PredictParam{});

// Convenience: focal vehicle is fleet[0]; others are fleet[1..].
FocalBestMeeting predictBestMeetingForCurrent(const std::vector<VehicleInfo>& fleet,
                                              const std::vector<VehicleHistory>& histories,
                                              const MultimodalGraph& graph,
                                              const PredictParam& param = PredictParam{});

FocalBestMeeting predictBestMeetingForCurrent(const std::vector<VehicleInfo>& fleet,
                                              const std::vector<VehicleHistory>& histories,
                                              const GraphContext& ctx,
                                              const PredictParam& param = PredictParam{});

// Load regional graph from nationwide .mmlp.bin (streaming, bbox from vehicles).
FocalBestMeeting predictBestMeetingFromFile(const std::string& graphPath,
                                              const std::string& focalVehicleId,
                                              const std::vector<VehicleInfo>& fleet,
                                              const std::vector<VehicleHistory>& histories,
                                              const PredictParam& param = PredictParam{},
                                              double regionPaddingMeters = 150000.0,
                                              std::string* error = nullptr);

}  // namespace mmlp
