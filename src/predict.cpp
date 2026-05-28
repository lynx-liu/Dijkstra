#include "mmlp/predict.hpp"

#include "mmlp/geo.hpp"
#include "mmlp/meeting.hpp"
#include "mmlp/motion.hpp"
#include "mmlp/routing.hpp"

#include <algorithm>

namespace mmlp {

namespace {

const VehicleHistory* findHistory(const std::vector<VehicleHistory>& histories,
                                  const std::string& id) {
  for (const auto& h : histories) {
    if (h.id == id) {
      return &h;
    }
  }
  return nullptr;
}

int64_t pairAlignTime(const VehicleInfo& a, const VehicleInfo& b) {
  return std::max(a.timestamp, b.timestamp);
}

double vehicleSpeedMs(const VehicleInfo& vehicle, const VehicleHistory* history,
                      const MultimodalGraph& graph) {
  const double speedKmh = resolveSpeedKmh(vehicle, history, nullptr, vehicle.type);
  return speedMsFromKmh(speedKmh);
}

bool pairWithinReach(const VehicleInfo& a, const VehicleInfo& b, double speedAMs,
                     double speedBMs, const PredictParam& param) {
  const double maxReach =
      (speedAMs + speedBMs) * param.maxTime * 0.55 + 5000.0;
  return haversineMeters({a.lat, a.lon}, {b.lat, b.lon}) <= maxReach;
}

MeetingResult runPair(const MultimodalGraph& graph, const SpatialIndex* matchIndex,
                      const VehicleInfo& va, const VehicleInfo& vb,
                      const std::vector<VehicleHistory>& histories, int64_t alignTime,
                      const PredictParam& param) {
  const VehicleHistory* ha = findHistory(histories, va.id);
  const VehicleHistory* hb = findHistory(histories, vb.id);
  const double speedAMs = vehicleSpeedMs(va, ha, graph);
  const double speedBMs = vehicleSpeedMs(vb, hb, graph);
  if (!pairWithinReach(va, vb, speedAMs, speedBMs, param)) {
    return {};
  }

  const PreparedVehicle pa = prepareVehicle(graph, va, ha, alignTime, param, matchIndex);
  const PreparedVehicle pb = prepareVehicle(graph, vb, hb, alignTime, param, matchIndex);
  MeetingResult meeting;
  if (computePairwiseMeetingFast(graph, pa, pb, alignTime, param, meeting)) {
    return meeting;
  }
  return {};
}

}  // namespace

std::vector<MeetingResult> predictMeetings(const std::vector<VehicleInfo>& vehicles,
                                           const std::vector<VehicleHistory>& histories,
                                           const MultimodalGraph& graph,
                                           const PredictParam& param) {
  GraphContext ctx;
  ctx.graph = graph;
  ctx.index.build(graph);
  return predictMeetings(vehicles, histories, ctx, param);
}

std::vector<MeetingResult> predictMeetings(const std::vector<VehicleInfo>& vehicles,
                                           const std::vector<VehicleHistory>& histories,
                                           const GraphContext& ctx,
                                           const PredictParam& param) {
  std::vector<MeetingResult> results;
  const std::size_t n = vehicles.size();
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      const int64_t alignTime = pairAlignTime(vehicles[i], vehicles[j]);
      MeetingResult meeting = runPair(ctx.graph, &ctx.index, vehicles[i], vehicles[j], histories,
                                      alignTime, param);
      if (meeting.meetTime != kNoMeetingTime) {
        results.push_back(meeting);
      }
    }
  }
  return results;
}

FocalBestMeeting predictBestMeetingFor(const std::string& focalVehicleId,
                                       const std::vector<VehicleInfo>& fleet,
                                       const std::vector<VehicleHistory>& histories,
                                       const MultimodalGraph& graph,
                                       const PredictParam& param) {
  GraphContext ctx;
  ctx.graph = graph;
  ctx.index.build(graph);
  return predictBestMeetingFor(focalVehicleId, fleet, histories, ctx, param);
}

FocalBestMeeting predictBestMeetingFor(const std::string& focalVehicleId,
                                       const std::vector<VehicleInfo>& fleet,
                                       const std::vector<VehicleHistory>& histories,
                                       const GraphContext& ctx,
                                       const PredictParam& param) {
  FocalBestMeeting best;
  best.focalVehicleId = focalVehicleId;

  const VehicleInfo* focal = nullptr;
  for (const auto& v : fleet) {
    if (v.id == focalVehicleId) {
      focal = &v;
      break;
    }
  }
  if (focal == nullptr) {
    return best;
  }

  const VehicleHistory* focalHist = findHistory(histories, focalVehicleId);
  const double focalSpeedMs = vehicleSpeedMs(*focal, focalHist, ctx.graph);

  double bestMeetTime = kInfTime;
  MeetingResult bestResult;
  int64_t bestAlignTime = 0;

  for (const auto& other : fleet) {
    if (other.id == focalVehicleId) {
      continue;
    }
    const VehicleHistory* otherHist = findHistory(histories, other.id);
    const double otherSpeedMs = vehicleSpeedMs(other, otherHist, ctx.graph);
    if (!pairWithinReach(*focal, other, focalSpeedMs, otherSpeedMs, param)) {
      continue;
    }

    const int64_t alignTime = pairAlignTime(*focal, other);
    MeetingResult meeting = runPair(ctx.graph, &ctx.index, *focal, other, histories, alignTime, param);
    if (meeting.meetTime == kNoMeetingTime || meeting.meetTime >= kInfTime / 2.0) {
      continue;
    }
    if (meeting.meetTime < bestMeetTime) {
      bestMeetTime = meeting.meetTime;
      bestResult = meeting;
      bestAlignTime = alignTime;
    }
  }

  if (bestMeetTime >= kInfTime / 2.0) {
    return best;
  }

  best.found = true;
  best.partnerVehicleId = bestResult.vehicleB;
  if (bestResult.vehicleA != focalVehicleId) {
    best.partnerVehicleId = bestResult.vehicleA;
  }
  best.meetTime = bestResult.meetTime;
  best.meetDuration = bestResult.meetTime - static_cast<double>(bestAlignTime);
  best.lat = bestResult.lat;
  best.lon = bestResult.lon;
  best.locationId = bestResult.locationId;
  return best;
}

FocalBestMeeting predictBestMeetingFor(const std::string& focalVehicleId,
                                       const std::vector<VehicleInfo>& fleet,
                                       const std::vector<VehicleHistory>& histories,
                                       const GraphContext& routeCtx,
                                       const SpatialIndex& matchIndex,
                                       const PredictParam& param) {
  FocalBestMeeting best;
  best.focalVehicleId = focalVehicleId;

  const VehicleInfo* focal = nullptr;
  for (const auto& v : fleet) {
    if (v.id == focalVehicleId) {
      focal = &v;
      break;
    }
  }
  if (focal == nullptr) {
    return best;
  }

  const VehicleHistory* focalHist = findHistory(histories, focalVehicleId);
  const double focalSpeedMs = vehicleSpeedMs(*focal, focalHist, routeCtx.graph);

  double bestMeetTime = kInfTime;
  MeetingResult bestResult;
  int64_t bestAlignTime = 0;

  for (const auto& other : fleet) {
    if (other.id == focalVehicleId) {
      continue;
    }
    const VehicleHistory* otherHist = findHistory(histories, other.id);
    const double otherSpeedMs = vehicleSpeedMs(other, otherHist, routeCtx.graph);
    if (!pairWithinReach(*focal, other, focalSpeedMs, otherSpeedMs, param)) {
      continue;
    }

    const int64_t alignTime = pairAlignTime(*focal, other);
    MeetingResult meeting = runPair(routeCtx.graph, &matchIndex, *focal, other, histories,
                                    alignTime, param);
    if (meeting.meetTime == kNoMeetingTime || meeting.meetTime >= kInfTime / 2.0) {
      continue;
    }
    if (meeting.meetTime < bestMeetTime) {
      bestMeetTime = meeting.meetTime;
      bestResult = meeting;
      bestAlignTime = alignTime;
    }
  }

  if (bestMeetTime >= kInfTime / 2.0) {
    return best;
  }

  best.found = true;
  best.partnerVehicleId = bestResult.vehicleB;
  if (bestResult.vehicleA != focalVehicleId) {
    best.partnerVehicleId = bestResult.vehicleA;
  }
  best.meetTime = bestResult.meetTime;
  best.meetDuration = bestResult.meetTime - static_cast<double>(bestAlignTime);
  best.lat = bestResult.lat;
  best.lon = bestResult.lon;
  best.locationId = bestResult.locationId;
  return best;
}

FocalBestMeeting predictBestMeetingForOnline(const std::string& focalVehicleId,
                                             const std::vector<VehicleInfo>& fleet,
                                             const std::vector<VehicleHistory>& histories,
                                             const GraphContext& routeCtx,
                                             const SpatialIndex& matchIndex,
                                             const PredictParam& param) {
  return predictBestMeetingFor(focalVehicleId, fleet, histories, routeCtx, matchIndex, param);
}

std::vector<FocalBestMeeting> predictMeetingsWithLead(const std::vector<VehicleInfo>& fleet,
                                                      const std::vector<VehicleHistory>& histories,
                                                      const GraphContext& routeCtx,
                                                      const SpatialIndex& matchIndex,
                                                      const PredictParam& param) {
  std::vector<FocalBestMeeting> results;
  if (fleet.empty()) {
    return results;
  }

  const VehicleInfo& lead = fleet.front();
  const VehicleHistory* leadHist = findHistory(histories, lead.id);
  const double leadSpeedMs = vehicleSpeedMs(lead, leadHist, routeCtx.graph);

  results.reserve(fleet.size() > 0 ? fleet.size() - 1 : 0);

  for (std::size_t i = 1; i < fleet.size(); ++i) {
    const VehicleInfo& other = fleet[i];
    FocalBestMeeting item;
    item.focalVehicleId = lead.id;
    item.partnerVehicleId = other.id;

    const VehicleHistory* otherHist = findHistory(histories, other.id);
    const double otherSpeedMs = vehicleSpeedMs(other, otherHist, routeCtx.graph);
    if (!pairWithinReach(lead, other, leadSpeedMs, otherSpeedMs, param)) {
      results.push_back(item);
      continue;
    }

    const int64_t alignTime = pairAlignTime(lead, other);
    MeetingResult meeting =
        runPair(routeCtx.graph, &matchIndex, lead, other, histories, alignTime, param);
    if (meeting.meetTime == kNoMeetingTime || meeting.meetTime >= kInfTime / 2.0) {
      results.push_back(item);
      continue;
    }

    item.found = true;
    item.meetTime = meeting.meetTime;
    item.meetDuration = meeting.meetTime - static_cast<double>(alignTime);
    item.lat = meeting.lat;
    item.lon = meeting.lon;
    item.locationId = meeting.locationId;
    results.push_back(item);
  }

  std::sort(results.begin(), results.end(),
            [](const FocalBestMeeting& a, const FocalBestMeeting& b) {
              if (a.found != b.found) {
                return a.found && !b.found;
              }
              if (!a.found) {
                return a.partnerVehicleId < b.partnerVehicleId;
              }
              if (a.meetDuration != b.meetDuration) {
                return a.meetDuration < b.meetDuration;
              }
              return a.partnerVehicleId < b.partnerVehicleId;
            });

  return results;
}

FocalBestMeeting predictBestMeetingForCurrent(const std::vector<VehicleInfo>& fleet,
                                                const std::vector<VehicleHistory>& histories,
                                                const MultimodalGraph& graph,
                                                const PredictParam& param) {
  if (fleet.empty()) {
    return {};
  }
  return predictBestMeetingFor(fleet.front().id, fleet, histories, graph, param);
}

FocalBestMeeting predictBestMeetingForCurrent(const std::vector<VehicleInfo>& fleet,
                                                const std::vector<VehicleHistory>& histories,
                                                const GraphContext& ctx,
                                                const PredictParam& param) {
  if (fleet.empty()) {
    return {};
  }
  return predictBestMeetingFor(fleet.front().id, fleet, histories, ctx, param);
}

FocalBestMeeting predictBestMeetingFromFile(const std::string& graphPath,
                                            const std::string& focalVehicleId,
                                            const std::vector<VehicleInfo>& fleet,
                                            const std::vector<VehicleHistory>& histories,
                                            const PredictParam& param,
                                            double regionPaddingMeters,
                                            std::string* error) {
  FocalBestMeeting out;
  out.focalVehicleId = focalVehicleId;

  const GeoBBox bbox = bboxFromVehicles(fleet, regionPaddingMeters);
  GraphContext ctx;
  if (!loadGraphContextRegion(graphPath, bbox, ctx, error)) {
    return out;
  }

  return predictBestMeetingFor(focalVehicleId, fleet, histories, ctx, param);
}

}  // namespace mmlp
