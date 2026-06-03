#include "mmlp/fleet_service.hpp"

#include "mmlp/geo.hpp"
#include "mmlp/motion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>

namespace mmlp {

namespace {

PredictParam servicePredictParam() {
  PredictParam p;
  if (const char* env = std::getenv("MMLP_MAX_TIME_SEC")) {
    p.maxTime = std::atof(env);
  } else {
    p.maxTime = 172800.0;  // 2 days (per technical spec)
  }
  p.maxVisitedNodes = 50000;
  return p;
}

double extractMaxRadiusM() {
  if (const char* env = std::getenv("MMLP_EXTRACT_MAX_RADIUS_M")) {
    return std::max(15000.0, std::atof(env));
  }
  return 55000.0;
}

uint64_t meetingSubgraphCacheKey(const VehicleInfo& focal,
                                 const std::vector<VehicleInfo>& partners, int widthKm) {
  uint64_t h = (static_cast<uint64_t>(static_cast<uint32_t>(
                    static_cast<int>(std::floor(focal.lat / 0.05)))) << 32) ^
               static_cast<uint32_t>(static_cast<int>(std::floor(focal.lon / 0.05)));
  h ^= static_cast<uint64_t>(widthKm) * 1315423911u;
  for (const auto& p : partners) {
    if (p.id == focal.id) {
      continue;
    }
    h ^= static_cast<uint64_t>(static_cast<uint32_t>(
              static_cast<int>(std::floor(p.lat / 0.05)))) *
         2654435761u;
    h ^= static_cast<uint64_t>(static_cast<uint32_t>(
              static_cast<int>(std::floor(p.lon / 0.05)))) *
         1597334677u;
    h ^= std::hash<std::string>{}(p.id);
  }
  return h;
}

std::size_t maxPairChecks() {
  if (const char* env = std::getenv("MMLP_MAX_PAIR_CHECKS")) {
    const int v = std::atoi(env);
    if (v > 0) {
      return static_cast<std::size_t>(v);
    }
  }
  return 32;
}

}  // namespace

FleetMeetingService::FleetMeetingService(std::string graphPath, double regionPaddingMeters)
    : graphPath_(std::move(graphPath)), paddingMeters_(regionPaddingMeters) {}

bool FleetMeetingService::bboxContains(const GeoBBox& outer, const GeoBBox& inner) {
  return outer.minLon <= inner.minLon && outer.maxLon >= inner.maxLon &&
         outer.minLat <= inner.minLat && outer.maxLat >= inner.maxLat;
}

std::vector<VehicleInfo> FleetMeetingService::fleetSnapshot(
    const std::unordered_map<std::string, VehicleInfo>& fleet) {
  std::vector<VehicleInfo> out;
  out.reserve(fleet.size());
  for (const auto& kv : fleet) {
    out.push_back(kv.second);
  }
  return out;
}

std::vector<VehicleHistory> FleetMeetingService::historySnapshot(
    const std::unordered_map<std::string, VehicleHistory>& histories) {
  std::vector<VehicleHistory> out;
  out.reserve(histories.size());
  for (const auto& kv : histories) {
    out.push_back(kv.second);
  }
  return out;
}

bool FleetMeetingService::preloadIndexOnly(std::string* error) {
  if (indexOnlyLoaded_ && graphReady_) {
    return true;
  }
  GraphContext fresh;
  if (!loadGraphContextIndexOnly(graphPath_, fresh, error)) {
    graphReady_ = false;
    indexOnlyLoaded_ = false;
    return false;
  }
  if (!graphStore_.open(graphPath_, error)) {
    graphReady_ = false;
    indexOnlyLoaded_ = false;
    return false;
  }
  ctx_ = std::move(fresh);
  graphReady_ = true;
  indexOnlyLoaded_ = true;
  fullGraphLoaded_ = false;
  loadedBBox_ = {73.0, 15.0, 135.0, 54.0};
  std::cerr << "[mmlp] index-only mode ready (mmap on-demand subgraph)\n" << std::flush;
  return true;
}

bool FleetMeetingService::preloadFullGraph(std::string* error) {
  if (fullGraphLoaded_ && graphReady_) {
    return true;
  }
  GraphContext fresh;
  if (!loadGraphContextFull(graphPath_, fresh, error)) {
    graphReady_ = false;
    fullGraphLoaded_ = false;
    return false;
  }
  ctx_ = std::move(fresh);
  graphReady_ = true;
  fullGraphLoaded_ = true;
  indexOnlyLoaded_ = false;
  loadedBBox_ = {73.0, 15.0, 135.0, 54.0};
  std::cerr << "[mmlp] full graph ready nodes=" << ctx_.graph.nodes().size()
            << " edges=" << ctx_.graph.edges().size() << "\n"
            << std::flush;
  return true;
}

bool FleetMeetingService::preloadRegion(const GeoBBox& bbox, std::string* error) {
  if (fullGraphLoaded_) {
    return true;
  }
  if (graphReady_ && bboxContains(loadedBBox_, bbox)) {
    return true;
  }
  std::cerr << "[mmlp] preloading graph region (may take 1-2 min) ...\n" << std::flush;
  GraphContext fresh;
  if (!loadGraphContextRegion(graphPath_, bbox, fresh, error)) {
    graphReady_ = false;
    return false;
  }
  ctx_ = std::move(fresh);
  loadedBBox_ = bbox;
  graphReady_ = true;
  std::cerr << "[mmlp] graph ready nodes=" << ctx_.graph.nodes().size()
            << " edges=" << ctx_.graph.edges().size() << "\n"
            << std::flush;
  return true;
}

bool FleetMeetingService::ensureGraphForVehicles(const std::vector<VehicleInfo>& vehicles,
                                               std::string* error) {
  if (indexOnlyLoaded_ && graphReady_) {
    return true;
  }
  if (fullGraphLoaded_ && graphReady_) {
    return true;
  }
  if (vehicles.empty()) {
    if (error) {
      *error = "no vehicles";
    }
    return false;
  }

  const GeoBBox need = bboxFromVehicles(vehicles, paddingMeters_);
  if (graphReady_ && bboxContains(loadedBBox_, need)) {
    return true;
  }

  std::cerr << "[mmlp] loading graph for vehicle bbox ...\n" << std::flush;
  GraphContext fresh;
  if (!loadGraphContextRegion(graphPath_, need, fresh, error)) {
    graphReady_ = false;
    return false;
  }
  ctx_ = std::move(fresh);
  loadedBBox_ = need;
  graphReady_ = true;
  return true;
}

bool FleetMeetingService::ensureGraphForFleet(std::string* error) {
  if (indexOnlyLoaded_ && graphReady_) {
    return true;
  }
  if (fullGraphLoaded_ && graphReady_) {
    return true;
  }

  const auto vehicles = fleetSnapshot(fleet_);
  if (vehicles.empty()) {
    return false;
  }

  const GeoBBox need = bboxFromVehicles(vehicles, paddingMeters_);
  if (graphReady_ && bboxContains(loadedBBox_, need)) {
    return true;
  }

  std::cerr << "[mmlp] loading graph for fleet bbox (may take 1-2 min) ...\n" << std::flush;
  GraphContext fresh;
  if (!loadGraphContextRegion(graphPath_, need, fresh, error)) {
    graphReady_ = false;
    return false;
  }

  ctx_ = std::move(fresh);
  loadedBBox_ = need;
  graphReady_ = true;
  std::cerr << "[mmlp] graph ready nodes=" << ctx_.graph.nodes().size()
            << " edges=" << ctx_.graph.edges().size() << "\n"
            << std::flush;
  return true;
}

bool FleetMeetingService::getOrBuildSubgraph(const VehicleInfo& focal,
                                             const std::vector<VehicleInfo>& partners,
                                             double corridorWidthM, const GraphContext*& sub,
                                             std::string* error) {
  corridorWidthM = std::min(corridorWidthM, extractMaxRadiusM());
  const int widthKm = static_cast<int>(corridorWidthM / 5000.0);
  const uint64_t key = meetingSubgraphCacheKey(focal, partners, widthKm);

  const auto hit = subgraphCache_.find(key);
  if (hit != subgraphCache_.end()) {
    sub = &hit->second.ctx;
    subgraphCacheLru_.remove(key);
    subgraphCacheLru_.push_back(key);
    return true;
  }

  if (subgraphCache_.size() >= kSubgraphCacheMax) {
    const uint64_t evict = subgraphCacheLru_.front();
    subgraphCacheLru_.pop_front();
    subgraphCache_.erase(evict);
  }

  SubgraphCacheEntry entry;
  entry.key = key;
  bool ok = false;
  if (indexOnlyLoaded_) {
    ok = extractGraphContextForMeetingIndexed(graphStore_, ctx_.index, focal, partners,
                                              corridorWidthM, entry.ctx, error);
  } else {
    ok = extractGraphContextForMeeting(ctx_, focal, partners, corridorWidthM, entry.ctx, error);
  }
  if (!ok) {
    return false;
  }
  subgraphCache_[key] = std::move(entry);
  subgraphCacheLru_.push_back(key);
  sub = &subgraphCache_[key].ctx;
  return true;
}

FocalBestMeeting FleetMeetingService::ingestVehicle(const VehicleInfo& vehicle,
                                                    const VehicleHistory* history,
                                                    std::string* error) {
  FocalBestMeeting result;
  result.focalVehicleId = vehicle.id;

  fleet_[vehicle.id] = vehicle;
  fleetIndex_.upsert(vehicle);
  if (history != nullptr && !history->speedSamples.empty()) {
    histories_[vehicle.id] = *history;
  }

  if (fleet_.size() < 2) {
    return result;
  }

  if (!ensureGraphForFleet(error)) {
    return result;
  }

  const PredictParam p = servicePredictParam();
  const VehicleHistory* focalHist =
      histories_.count(vehicle.id) ? &histories_.at(vehicle.id) : nullptr;
  const double focalSpeedMs = speedMsFromKmh(resolveSpeedKmh(vehicle, focalHist, nullptr, vehicle.type));
  const double reachM = focalSpeedMs * p.maxTime * 0.55 + 5000.0;

  const auto nearbyIds =
      fleetIndex_.queryNearbyIds(vehicle.lat, vehicle.lon, vehicle.type, reachM, vehicle.id);
  const std::size_t cap = maxPairChecks();
  std::vector<VehicleInfo> compareFleet;
  compareFleet.reserve(std::min(cap, nearbyIds.size()) + 1);
  compareFleet.push_back(vehicle);
  for (std::size_t i = 0; i < nearbyIds.size() && i < cap; ++i) {
    const auto it = fleet_.find(nearbyIds[i]);
    if (it != fleet_.end()) {
      compareFleet.push_back(it->second);
    }
  }

  double padM = std::min(paddingMeters_, focalSpeedMs * p.maxTime * 0.45 + 25000.0);
  for (const auto& v : compareFleet) {
    if (v.id == vehicle.id) {
      continue;
    }
    const double d = haversineMeters({vehicle.lat, vehicle.lon}, {v.lat, v.lon});
    padM = std::min(padM, d + 20000.0);
  }
  padM = std::max(padM, 15000.0);

  std::vector<VehicleHistory> histVec;
  histVec.reserve(compareFleet.size());
  for (const auto& v : compareFleet) {
    const auto hit = histories_.find(v.id);
    if (hit != histories_.end()) {
      histVec.push_back(hit->second);
    }
  }

  const auto t0 = std::chrono::steady_clock::now();

  if (fullGraphLoaded_ || indexOnlyLoaded_) {
    const GraphContext* sub = nullptr;
    std::string subErr;
    if (!getOrBuildSubgraph(vehicle, compareFleet, padM, sub, &subErr)) {
      if (error) {
        *error = subErr;
      }
      return result;
    }
    const auto t1 = std::chrono::steady_clock::now();
    result = predictBestMeetingFor(vehicle.id, compareFleet, histVec, *sub, ctx_.index, p);
    const auto t2 = std::chrono::steady_clock::now();
    std::cerr << "[mmlp] fleet=" << fleet_.size() << " nearby=" << nearbyIds.size()
              << " checked=" << (compareFleet.size() - 1)
              << " sub_nodes=" << sub->graph.nodes().size()
              << " extract_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
              << " predict_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " found=" << (result.found ? 1 : 0) << "\n"
              << std::flush;
    return result;
  }

  result = predictBestMeetingFor(vehicle.id, compareFleet, histVec, ctx_, p);
  const auto t1 = std::chrono::steady_clock::now();
  std::cerr << "[mmlp] fleet=" << fleet_.size() << " nearby=" << nearbyIds.size()
            << " checked=" << (compareFleet.size() - 1) << " predict_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
            << " found=" << (result.found ? 1 : 0) << "\n"
            << std::flush;
  return result;
}

std::vector<FocalBestMeeting> FleetMeetingService::meetingsWithLead(
    const std::vector<VehicleInfo>& vehicles, const std::vector<VehicleHistory>& histories,
    std::string* error) {
  std::vector<FocalBestMeeting> empty;
  if (vehicles.empty()) {
    if (error) {
      *error = "vehicles array is empty";
    }
    return empty;
  }
  if (vehicles.size() == 1) {
    return empty;
  }

  if (!ensureGraphForVehicles(vehicles, error)) {
    return empty;
  }

  const PredictParam p = servicePredictParam();
  const VehicleInfo& lead = vehicles.front();
  const VehicleHistory* leadHist = nullptr;
  for (const auto& h : histories) {
    if (h.id == lead.id) {
      leadHist = &h;
      break;
    }
  }
  const double leadSpeedMs =
      speedMsFromKmh(resolveSpeedKmh(lead, leadHist, nullptr, lead.type));

  double padM = std::min(paddingMeters_, leadSpeedMs * p.maxTime * 0.45 + 25000.0);
  for (std::size_t i = 1; i < vehicles.size(); ++i) {
    const double d = haversineMeters({lead.lat, lead.lon}, {vehicles[i].lat, vehicles[i].lon});
    padM = std::min(padM, d + 20000.0);
  }
  padM = std::max(padM, 15000.0);

  if (fullGraphLoaded_ || indexOnlyLoaded_) {
    const GraphContext* sub = nullptr;
    std::string subErr;
    if (!getOrBuildSubgraph(lead, vehicles, padM, sub, &subErr)) {
      if (error) {
        *error = subErr;
      }
      return empty;
    }
    return predictMeetingsWithLead(vehicles, histories, *sub, ctx_.index, p);
  }

  return predictMeetingsWithLead(vehicles, histories, ctx_, ctx_.index, p);
}

DestinationArrivalSummary FleetMeetingService::vehiclesReachDestinationBy(
    double destLat, double destLon, int64_t arriveByUnix, VehicleType destType,
    ArrivalSortBy sortBy, const std::vector<VehicleInfo>* overrideVehicles,
    const std::vector<VehicleHistory>* overrideHistories, std::string* error) {
  DestinationArrivalSummary empty;
  if (arriveByUnix <= 0) {
    if (error) {
      *error = "arriveByUnix must be positive";
    }
    return empty;
  }

  std::vector<VehicleInfo> vehicles;
  std::vector<VehicleHistory> histories;
  if (overrideVehicles != nullptr && !overrideVehicles->empty()) {
    vehicles = *overrideVehicles;
    if (overrideHistories != nullptr) {
      histories = *overrideHistories;
    }
  } else {
    vehicles = fleetSnapshot(fleet_);
    histories = historySnapshot(histories_);
  }

  if (vehicles.empty()) {
    if (error) {
      *error = "no vehicles (ingest fleet or pass vehicles array)";
    }
    return empty;
  }

  int64_t latestVehicleTime = 0;
  for (const auto& v : vehicles) {
    latestVehicleTime = std::max(latestVehicleTime, v.timestamp);
  }
  if (arriveByUnix <= latestVehicleTime) {
    if (error) {
      *error =
          "arriveBy must be later than every vehicle time "
          "(set arriveBy = vehicle time + travel window, e.g. +2 hours)";
    }
    return empty;
  }

  VehicleInfo destProbe;
  destProbe.id = "__destination__";
  destProbe.lat = destLat;
  destProbe.lon = destLon;
  destProbe.type = destType;
  destProbe.speed = 60.0;
  destProbe.timestamp = arriveByUnix;

  std::vector<VehicleInfo> forGraph = vehicles;
  forGraph.push_back(destProbe);

  if (!ensureGraphForVehicles(forGraph, error)) {
    return empty;
  }

  const PredictParam p = servicePredictParam();

  double padM = paddingMeters_;
  for (const auto& v : vehicles) {
    const double d = haversineMeters({destLat, destLon}, {v.lat, v.lon});
    padM = std::min(padM, d + 20000.0);
  }
  padM = std::max(padM, 15000.0);

  if (fullGraphLoaded_ || indexOnlyLoaded_) {
    const GraphContext* sub = nullptr;
    std::string subErr;
    if (!getOrBuildSubgraph(destProbe, forGraph, padM, sub, &subErr)) {
      if (error) {
        *error = subErr;
      }
      return empty;
    }
    DestinationQuery q;
    q.lat = destLat;
    q.lon = destLon;
    q.arriveByUnix = arriveByUnix;
    q.type = destType;
    q.sortBy = sortBy;
    return predictVehiclesToDestination(vehicles, histories, *sub, ctx_.index, q, p);
  }

  DestinationQuery q;
  q.lat = destLat;
  q.lon = destLon;
  q.arriveByUnix = arriveByUnix;
  q.type = destType;
  q.sortBy = sortBy;
  return predictVehiclesToDestination(vehicles, histories, ctx_, ctx_.index, q, p);
}

bool FleetMeetingService::removeVehicle(const std::string& vehicleId) {
  fleetIndex_.remove(vehicleId);
  const bool had = fleet_.erase(vehicleId) > 0;
  histories_.erase(vehicleId);
  return had;
}

void FleetMeetingService::clearFleet() {
  fleet_.clear();
  histories_.clear();
  fleetIndex_.clear();
}

}  // namespace mmlp
