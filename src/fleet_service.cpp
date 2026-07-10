#include "mmlp/fleet_service.hpp"

#include "mmlp/arrival.hpp"
#include "mmlp/geo.hpp"
#include "mmlp/motion.hpp"
#include "mmlp/region_registry.hpp"
#include "mmlp/thread_pool.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <cstring>
#include <iostream>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>

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

uint64_t destinationSubgraphCacheKey(const std::vector<VehicleInfo>& vehicles, int widthKm) {
  uint64_t h = 0xDE57A7110ULL;
  h ^= static_cast<uint64_t>(widthKm) * 1315423911u;
  for (const auto& v : vehicles) {
    h ^= static_cast<uint64_t>(static_cast<uint32_t>(
              static_cast<int>(std::floor(v.lat / 0.05)))) *
         2654435761u;
    h ^= static_cast<uint64_t>(static_cast<uint32_t>(
              static_cast<int>(std::floor(v.lon / 0.05)))) *
         1597334677u;
    h ^= std::hash<std::string>{}(v.id);
  }
  return h;
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

void preloadConfiguredRegions(const std::string& graphPath,
                              const std::function<bool(const std::string&, std::string*)>& ensure) {
  const char* env = std::getenv("MMLP_PRELOAD_REGIONS");
  // Default: preload every provincial tile so first destination query stays <3s.
  // Set MMLP_PRELOAD_REGIONS=off to skip, or a comma list (e.g. gd,bj) to limit.
  if (env != nullptr && (std::strcmp(env, "off") == 0 || std::strcmp(env, "0") == 0 ||
                         std::strcmp(env, "none") == 0)) {
    return;
  }
  std::vector<std::string> suffixes;
  if (env == nullptr || env[0] == '\0' || std::strcmp(env, "all") == 0) {
    const RegionBBoxView* views = chinaRegions();
    for (std::size_t i = 0; i < chinaRegionCount(); ++i) {
      suffixes.emplace_back(views[i].suffix);
    }
  } else {
    std::string list(env);
    std::size_t pos = 0;
    while (pos < list.size()) {
      const std::size_t comma = list.find(',', pos);
      suffixes.push_back(
          list.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos));
      pos = comma == std::string::npos ? list.size() : comma + 1;
    }
  }
  std::vector<std::future<void>> futs;
  futs.reserve(suffixes.size());
  for (const auto& suffix : suffixes) {
    if (suffix.empty() || !regionalGraphFileExists(graphPath, suffix)) {
      continue;
    }
    futs.push_back(ThreadPool::instance().submit([ensure, suffix]() {
      std::string warmErr;
      ensure(suffix, &warmErr);
    }));
  }
  for (auto& f : futs) {
    f.get();
  }
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
  if (graphStore_.hasCh()) {
    const auto tCh0 = std::chrono::steady_clock::now();
    graphStore_.ch().warmReverseDown();
    const auto tCh1 = std::chrono::steady_clock::now();
    std::cerr << "[mmlp] national ch warm_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(tCh1 - tCh0).count() << "\n"
              << std::flush;
  }
  // National dense Full CH: blocking page-in so the first interactive query is hot.
  // Prefer this over async-only warm; provincial preload is skipped when present so
  // ~8GB national full.ch stays in page cache on typical 32GB hosts.
  if (graphStore_.hasFullCh()) {
    const auto tFull0 = std::chrono::steady_clock::now();
    graphStore_.warmMappedRoutingFiles();
    const auto tFull1 = std::chrono::steady_clock::now();
    std::cerr << "[mmlp] national full_ch warm_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(tFull1 - tFull0).count()
              << " mapped_mb=" << (graphStore_.fullCh().mappedSize() / (1024 * 1024))
              << " nodes=" << graphStore_.fullCh().nodeCount() << "\n"
              << std::flush;
  } else {
    graphStore_.warmMappedRoutingFilesAsync();
  }
  ctx_ = std::move(fresh);
  graphReady_ = true;
  indexOnlyLoaded_ = true;
  fullGraphLoaded_ = false;
  loadedBBox_ = {73.0, 15.0, 135.0, 54.0};
  std::cerr << "[mmlp] index-only mode ready (mmap on-demand subgraph)\n" << std::flush;
  // Provincial full.ch preload: skip by default when national full.ch exists (keeps
  // national hot). Override with MMLP_PRELOAD_REGIONS=all|gd,bj|off.
  const bool skipRegionalForNational =
      graphStore_.hasFullCh() && std::getenv("MMLP_PRELOAD_REGIONS") == nullptr;
  if (!skipRegionalForNational) {
    const auto tPre0 = std::chrono::steady_clock::now();
    preloadConfiguredRegions(graphPath_, [this](const std::string& suffix, std::string* err) {
      return ensureRegionalGraph(suffix, err);
    });
    const auto tPre1 = std::chrono::steady_clock::now();
    std::cerr << "[mmlp] regional preload_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(tPre1 - tPre0).count()
              << " regions=" << regionalGraphs_.size() << "\n"
              << std::flush;
  } else {
    std::cerr << "[mmlp] regional preload skipped (national full.ch present; "
                 "set MMLP_PRELOAD_REGIONS=all to force)\n"
              << std::flush;
  }

  // Synthetic destination queries so the first real request is already hot
  // (snap + full.ch settle paths + thread-local caches across major corridors).
  {
    const auto tWarm0 = std::chrono::steady_clock::now();
    std::vector<VehicleInfo> probes;
    const char* seeds[][3] = {
        {"warm_gd", "23.1", "113.3"}, {"warm_sd", "36.7", "117.0"},
        {"warm_sc", "30.6", "104.1"}, {"warm_xj", "43.8", "87.6"},
        {"warm_hl", "45.8", "126.5"}, {"warm_yn", "25.0", "102.7"},
        {"warm_nx", "38.47", "106.27"}, {"warm_ah", "31.82", "117.23"},
    };
    const int64_t now = 1700000000;
    for (const auto& s : seeds) {
      VehicleInfo v;
      v.id = s[0];
      v.lat = std::atof(s[1]);
      v.lon = std::atof(s[2]);
      v.speed = 60.0;
      v.timestamp = now;
      v.type = VehicleType::TRUCK;
      probes.push_back(v);
    }
    std::string warmErr;
    // Multiple dests: cross-province + same-province so both paths are hot.
    const double dests[][2] = {
        {39.9042, 116.4074},  // Beijing
        {23.1291, 113.2644},  // Guangzhou
        {31.8206, 117.2272},  // Hefei
    };
    for (const auto& d : dests) {
      (void)vehiclesReachDestinationBy(d[0], d[1], now + 30 * 86400, VehicleType::TRUCK,
                                       ArrivalSortBy::DURATION, &probes, nullptr, &warmErr);
    }
    const auto tWarm1 = std::chrono::steady_clock::now();
    std::cerr << "[mmlp] destination warm_ms="
              << std::chrono::duration_cast<std::chrono::milliseconds>(tWarm1 - tWarm0).count()
              << " dests=" << (sizeof(dests) / sizeof(dests[0])) << "\n"
              << std::flush;
  }
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

bool FleetMeetingService::getOrBuildDestinationSubgraph(const std::vector<VehicleInfo>& vehicles,
                                                        double paddingMeters,
                                                        const GraphContext*& sub,
                                                        std::string* error) {
  paddingMeters = std::min(paddingMeters, extractMaxRadiusM());
  const int widthKm = static_cast<int>(paddingMeters / 5000.0);
  const uint64_t key = destinationSubgraphCacheKey(vehicles, widthKm);

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
    ok = extractGraphContextForDestinationIndexed(graphStore_, ctx_.index, vehicles,
                                                  paddingMeters, entry.ctx, error);
  } else {
    ok = extractGraphContextForDestination(ctx_, vehicles, paddingMeters, entry.ctx, error);
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

bool FleetMeetingService::ensureRegionalGraph(const std::string& suffix, std::string* error) {
  {
    std::lock_guard<std::mutex> lock(regionalMutex_);
    auto found = regionalGraphs_.find(suffix);
    if (found != regionalGraphs_.end() && found->second && found->second->ready) {
      return true;
    }
  }

  const std::string path = regionalGraphPath(graphPath_, suffix);
  std::ifstream probe(path);
  if (!probe.good()) {
    return false;
  }

  auto loaded = std::make_unique<RegionalGraph>();
  loaded->ready = false;
  loaded->fullGraph = false;
  if (!loadGraphContextIndexOnly(path, loaded->ctx, error)) {
    return false;
  }
  if (!loaded->store.open(path, error)) {
    return false;
  }
  if (loaded->store.hasCh()) {
    loaded->store.ch().warmReverseDown();
  }
  // Blocking warm so first query after preload does not stall on page faults.
  loaded->store.warmMappedRoutingFiles();
  loaded->ready = true;

  {
    std::lock_guard<std::mutex> lock(regionalMutex_);
    auto found = regionalGraphs_.find(suffix);
    if (found != regionalGraphs_.end() && found->second && found->second->ready) {
      return true;
    }
    regionalGraphs_[suffix] = std::move(loaded);
  }
  std::cerr << "[mmlp] regional graph ready suffix=" << suffix << " path=" << path << "\n"
            << std::flush;
  return true;
}

const char* FleetMeetingService::regionSuffixForPoint(double lat, double lon) const {
  return mmlp::regionSuffixForPoint(lat, lon);
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

  int64_t earliestVehicleTime = std::numeric_limits<int64_t>::max();
  int64_t latestVehicleTime = 0;
  for (const auto& v : vehicles) {
    earliestVehicleTime = std::min(earliestVehicleTime, v.timestamp);
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

  PredictParam p = servicePredictParam();
  // Destination API: user-supplied arriveBy is the deadline; do not cap travel at
  // the global 2-day meeting-search default (cross-country hauls need weeks).
  p.maxTime = static_cast<double>(arriveByUnix - earliestVehicleTime);

  // Corridor width cap (55km default); do not use paddingMeters_ (150km) — bbox extract was 10× slower.
  double padM = extractMaxRadiusM();

  DestinationQuery q;
  q.lat = destLat;
  q.lon = destLon;
  q.arriveByUnix = arriveByUnix;
  q.type = destType;
  q.sortBy = sortBy;

  if (fullGraphLoaded_) {
    return predictVehiclesToDestination(vehicles, histories, ctx_, ctx_.index, q, p);
  }

  if (indexOnlyLoaded_) {
    const GraphFileStore* chOverlay = graphStore_.hasCh() ? &graphStore_ : nullptr;

    // Preferred path: national dense Full CH — one A→B query per vehicle for both
    // same-province and cross-province (accurate roads, interactive latency when warm).
    if (graphStore_.hasFullCh()) {
      PredictParam nationalP = p;
      nationalP.requireRoutePolyline = true;  // never accept empty-route geodesic ETA
      nationalP.maxRouteWallMs = 0.0;
      return predictVehiclesToDestinationIndexed(vehicles, histories, graphStore_, ctx_.index, q,
                                                 padM, nationalP, chOverlay, &graphStore_,
                                                 &ctx_.index, /*destRegionSuffix=*/nullptr);
    }

    const std::string destRegion = resolveRegionalSuffix(graphPath_, destLat, destLon);

    // Legacy fallback when china.mmlp.full.ch is absent: split local vs remote and
    // stitch provincial full.ch + sparse national hwy CH.
    std::vector<VehicleInfo> localVehicles;
    std::vector<VehicleInfo> remoteVehicles;
    localVehicles.reserve(vehicles.size());
    remoteVehicles.reserve(vehicles.size());
    if (!destRegion.empty()) {
      for (const auto& v : vehicles) {
        if (pointInRegionSuffix(destRegion, v.lat, v.lon)) {
          localVehicles.push_back(v);
        } else {
          remoteVehicles.push_back(v);
        }
      }
    } else {
      remoteVehicles = vehicles;
    }

    DestinationArrivalSummary summary;
    summary.lat = destLat;
    summary.lon = destLon;
    summary.arriveByUnix = arriveByUnix;
    summary.sortBy = sortBy;

    // Route one point→point on a regional full.ch (returns empty if unreachable).
    auto clampIntoRegion = [](const RegionBBoxView* box, double& lat, double& lon) {
      if (box == nullptr) {
        return;
      }
      // Pull slightly inside the bbox so map-matching lands on in-region roads.
      constexpr double kInset = 0.05;
      const double minLon = box->minLon + kInset;
      const double maxLon = box->maxLon - kInset;
      const double minLat = box->minLat + kInset;
      const double maxLat = box->maxLat - kInset;
      lon = std::min(maxLon, std::max(minLon, lon));
      lat = std::min(maxLat, std::max(minLat, lat));
    };

    // Reject fake "roads": 2-point chords and sparse nearly-straight polylines.
    // These pass older gap metrics but look like geodesic lines on the map.
    auto isDrawableRoad = [](const RoutePolyline& route) -> bool {
      if (route.points.size() < 2) {
        return false;
      }
      double path = 0.0;
      double maxSeg = 0.0;
      for (std::size_t i = 1; i < route.points.size(); ++i) {
        const double d = haversineMeters(route.points[i - 1], route.points[i]);
        path += d;
        maxSeg = std::max(maxSeg, d);
      }
      if (path < 1.0) {
        return false;
      }
      // Any single hop over 50km is a visual chord at map scale.
      if (maxSeg > 50000.0) {
        return false;
      }
      const double chord =
          haversineMeters(route.points.front(), route.points.back());
      const double dens = path / static_cast<double>(route.points.size() - 1);
      if (route.points.size() <= 4 && path > 5000.0 && chord / path > 0.95) {
        return false;
      }
      if (path > 100000.0 && dens > 40000.0 && chord / path > 0.90) {
        return false;
      }
      return true;
    };

    auto routeLegOnRegion = [&](const std::string& regionSuffix, double fromLat, double fromLon,
                                double toLat, double toLon, double speedKmh, int64_t timeUnix,
                                const std::string& vehicleId,
                                bool corridorFast = true) -> VehicleArrivalResult {
      VehicleArrivalResult miss;
      miss.vehicleId = vehicleId;
      miss.reachable = false;
      if (regionSuffix.empty() || !ensureRegionalGraph(regionSuffix, error)) {
        return miss;
      }
      const GraphFileStore* store = nullptr;
      const SpatialIndex* index = nullptr;
      {
        std::lock_guard<std::mutex> lock(regionalMutex_);
        RegionalGraph& regional = *regionalGraphs_.at(regionSuffix);
        if (!regional.store.hasFullCh()) {
          return miss;
        }
        store = &regional.store;
        index = &regional.ctx.index;
      }
      const RegionBBoxView* box = regionBBoxForSuffix(regionSuffix);
      clampIntoRegion(box, fromLat, fromLon);
      clampIntoRegion(box, toLat, toLon);
      VehicleInfo probe;
      probe.id = vehicleId;
      probe.lat = fromLat;
      probe.lon = fromLon;
      probe.speed = speedKmh;
      probe.timestamp = timeUnix;
      probe.type = destType;
      DestinationQuery legQ = q;
      legQ.lat = toLat;
      legQ.lon = toLon;
      PredictParam legP = p;
      legP.requireRoutePolyline = true;
      if (corridorFast) {
        const double legM = haversineMeters({fromLat, fromLon}, {toLat, toLon});
        const double speedMs = speedMsFromKmh(std::max(speedKmh, 1.0));
        legP.maxTime = std::min(p.maxTime, legM / std::max(speedMs, 0.5) * 2.5 + 3600.0);
        legP.maxVisitedNodes = 200000;
        legP.maxRouteWallMs = 2500.0;
      }
      DestinationArrivalSummary part = predictVehiclesToDestinationIndexed(
          {probe}, histories, *store, *index, legQ, padM, legP, nullptr, nullptr, nullptr,
          regionSuffix.c_str());
      if (part.vehicles.empty() || !part.vehicles.front().reachable ||
          !isDrawableRoad(part.vehicles.front().route)) {
        return miss;
      }
      return part.vehicles.front();
    };

    // Batch many vehicles → same exit on one regional full.ch.
    // Prefer a few high-quality exits with full CH budget (no wall). Short
    // wall-clock under batch contention previously forced every NX vehicle onto
    // a 2-point geodesic stub that looked like a straight line on the map.
    auto routeBatchFirstLegs =
        [&](const std::string& regionSuffix, const std::vector<VehicleInfo>& batch,
            double toLat, double toLon) -> std::unordered_map<std::string, VehicleArrivalResult> {
      std::unordered_map<std::string, VehicleArrivalResult> out;
      if (regionSuffix.empty() || batch.empty() || !ensureRegionalGraph(regionSuffix, error)) {
        return out;
      }
      const RegionBBoxView* box = regionBBoxForSuffix(regionSuffix);
      std::vector<LatLon> exits;
      auto pushExit = [&](double lat, double lon) {
        if (box != nullptr) {
          clampIntoRegion(box, lat, lon);
        }
        for (const auto& e : exits) {
          if (haversineMeters(e, {lat, lon}) < 5000.0) {
            return;
          }
        }
        exits.push_back({lat, lon});
      };
      if (box != nullptr) {
        // Median vehicle position — NX (and similar) graphs are fragmented; exits
        // far from the fleet often land in a disconnected component.
        double medLat = 0.0;
        double medLon = 0.0;
        for (const auto& v : batch) {
          medLat += v.lat;
          medLon += v.lon;
        }
        medLat /= static_cast<double>(batch.size());
        medLon /= static_cast<double>(batch.size());
        clampIntoRegion(box, medLat, medLon);

        double rayLat = toLat;
        double rayLon = toLon;
        regionBorderToward(*box, toLat, toLon, rayLat, rayLon);

        // Prefer exits that face the destination, then fall back to fleet-latitude
        // east/west rims (Ningxia Full CH often only connects on the east rim).
        pushExit(rayLat, rayLon);
        pushExit(medLat * 0.4 + rayLat * 0.6, medLon * 0.4 + rayLon * 0.6);
        pushExit(medLat, box->maxLon - 0.05);
        pushExit(medLat, box->minLon + 0.05);
        pushExit(box->minLat + 0.05, medLon);
        pushExit(box->maxLat - 0.05, medLon);
        // Cap for latency (pushExit already dedupes).
        if (exits.size() > 5) {
          exits.resize(5);
        }
      } else {
        pushExit(toLat, toLon);
      }

      std::vector<VehicleInfo> remaining = batch;
      for (auto& v : remaining) {
        if (box != nullptr) {
          clampIntoRegion(box, v.lat, v.lon);
        }
        if (v.speed <= 1.0) {
          v.speed = 60.0;
        }
      }
      const GraphFileStore* store = nullptr;
      const SpatialIndex* index = nullptr;
      {
        std::lock_guard<std::mutex> lock(regionalMutex_);
        RegionalGraph& regional = *regionalGraphs_.at(regionSuffix);
        if (!regional.store.hasFullCh()) {
          return out;
        }
        store = &regional.store;
        index = &regional.ctx.index;
      }
      PredictParam firstP = p;
      firstP.maxVisitedNodes = 200000;
      firstP.maxRouteWallMs = 0.0;
      firstP.requireRoutePolyline = true;  // never accept empty-route geodesic ETA
      for (const auto& exitPt : exits) {
        if (remaining.empty()) {
          break;
        }
        DestinationQuery legQ = q;
        legQ.lat = exitPt.lat;
        legQ.lon = exitPt.lon;
        DestinationArrivalSummary part = predictVehiclesToDestinationIndexed(
            remaining, histories, *store, *index, legQ, padM, firstP, nullptr, nullptr, nullptr,
            regionSuffix.c_str());
        std::unordered_set<std::string> got;
        for (auto& row : part.vehicles) {
          if (row.reachable && isDrawableRoad(row.route)) {
            got.insert(row.vehicleId);
            out.emplace(row.vehicleId, std::move(row));
          }
        }
        if (got.empty()) {
          continue;
        }
        std::vector<VehicleInfo> next;
        next.reserve(remaining.size() - got.size());
        for (const auto& v : remaining) {
          if (got.count(v.id) == 0) {
            next.push_back(v);
          }
        }
        remaining.swap(next);
      }
      if (!remaining.empty()) {
        std::cerr << "[mmlp] first_leg miss region=" << regionSuffix << " left=" << remaining.size()
                  << " exits=" << exits.size() << " — dest-finish only (no geodesic stub)\n"
                  << std::flush;
      }
      return out;
    };

    auto appendPolyline = [](RoutePolyline& dst, const RoutePolyline& src) {
      for (const auto& pt : src.points) {
        if (dst.points.empty() || haversineMeters(dst.points.back(), pt) > 5.0) {
          dst.points.push_back(pt);
        }
      }
    };

    // National highway CH bridge: fills the long gap between home-province exit
    // and dest-province entry with a real road polyline (not a geodesic chord).
    // One query is shared across all vehicles with the same rounded endpoints.
    struct HwyBridge {
      RoutePolyline route;
      double travelSecAt60 = 0.0;
      double distM = 0.0;
      bool ok = false;
    };
    std::mutex hwyBridgeMutex;
    std::unordered_map<std::string, HwyBridge> hwyCache;
    auto hwyBridgeKey = [](const LatLon& from, const LatLon& to) {
      char buf[96];
      std::snprintf(buf, sizeof(buf), "%.3f,%.3f|%.3f,%.3f", from.lat, from.lon, to.lat, to.lon);
      return std::string(buf);
    };
    auto routeHwyBridgeOnce = [&](const LatLon& from, const LatLon& to) -> HwyBridge {
      HwyBridge miss;
      if (!graphStore_.hasCh() || !graphStore_.hasHwyCsr()) {
        return miss;
      }
      const double gap = haversineMeters(from, to);
      if (gap < 5000.0) {
        miss.ok = true;
        miss.route.points = {from, to};
        miss.distM = gap;
        miss.travelSecAt60 = miss.distM / speedMsFromKmh(60.0);
        return miss;
      }
      VehicleInfo probe;
      probe.id = "__hwy_bridge__";
      probe.lat = from.lat;
      probe.lon = from.lon;
      probe.speed = 60.0;
      probe.timestamp = arriveByUnix - 7 * 86400;
      probe.type = destType;
      DestinationQuery bridgeQ = q;
      bridgeQ.lat = to.lat;
      bridgeQ.lon = to.lon;
      PredictParam bridgeP = p;
      bridgeP.maxVisitedNodes = 0;
      bridgeP.requireRoutePolyline = true;
      auto opt = predictVehicleToDestinationCh(graphStore_, ctx_.index, graphStore_, probe, nullptr,
                                               bridgeQ, bridgeP);
      HwyBridge out;
      if (opt && opt->reachable && isDrawableRoad(opt->route)) {
        out.ok = true;
        out.route = std::move(opt->route);
        out.travelSecAt60 = opt->travelDurationSec;
        out.distM = opt->routeDistanceM;
      }
      return out;
    };
    // National highway CH bridge: one direct query, then one midpoint split.
    // Avoid candidate grids / multi-hop chains here — those explode latency and
    // still often miss; regional Full CH in appendWithBridge is the real filler.
    auto routeHwyBridge = [&](const LatLon& from, const LatLon& to) -> HwyBridge {
      HwyBridge miss;
      if (!graphStore_.hasCh() || !graphStore_.hasHwyCsr()) {
        return miss;
      }
      const std::string key = hwyBridgeKey(from, to);
      {
        std::lock_guard<std::mutex> lock(hwyBridgeMutex);
        auto it = hwyCache.find(key);
        if (it != hwyCache.end()) {
          return it->second;
        }
      }
      HwyBridge out = routeHwyBridgeOnce(from, to);
      if ((!out.ok || !isDrawableRoad(out.route)) && haversineMeters(from, to) > 200000.0) {
        const LatLon mid{0.5 * (from.lat + to.lat), 0.5 * (from.lon + to.lon)};
        HwyBridge a = routeHwyBridgeOnce(from, mid);
        HwyBridge b = routeHwyBridgeOnce(mid, to);
        if (a.ok && b.ok && isDrawableRoad(a.route) && isDrawableRoad(b.route)) {
          out.ok = true;
          out.route = a.route;
          appendPolyline(out.route, b.route);
          out.travelSecAt60 = a.travelSecAt60 + b.travelSecAt60;
          out.distM = a.distM + b.distM;
        }
      }
      if (!out.ok || !isDrawableRoad(out.route)) {
        out = HwyBridge{};
        std::cerr << "[mmlp] hwy_bridge miss from=(" << from.lat << "," << from.lon << ") to=("
                  << to.lat << "," << to.lon << ")\n"
                  << std::flush;
      }
      {
        std::lock_guard<std::mutex> lock(hwyBridgeMutex);
        hwyCache[key] = out;
      }
      return out;
    };

    // Corridor hops: each entry is (region, exit waypoint inside that region toward dest).
    // Sample the corridor from the HOME REGION CENTER (not a vehicle GPS) so every
    // vehicle in the same province shares one stable province sequence.
    auto buildCorridorWaypoints = [&](const std::string& homeSuffix)
        -> std::vector<std::pair<std::string, LatLon>> {
      double sampleLat = destLat;
      double sampleLon = destLon;
      if (const RegionBBoxView* homeBox = regionBBoxForSuffix(homeSuffix)) {
        sampleLat = 0.5 * (homeBox->minLat + homeBox->maxLat);
        sampleLon = 0.5 * (homeBox->minLon + homeBox->maxLon);
      }
      std::vector<std::string> suffixes =
          corridorRegionSuffixes(graphPath_, sampleLat, sampleLon, destLat, destLon, 48);
      if (suffixes.empty() || suffixes.front() != homeSuffix) {
        suffixes.insert(suffixes.begin(), homeSuffix);
      }
      if (!destRegion.empty() && (suffixes.empty() || suffixes.back() != destRegion)) {
        suffixes.push_back(destRegion);
      }
      std::vector<std::string> uniq;
      for (const auto& s : suffixes) {
        if (uniq.empty() || uniq.back() != s) {
          uniq.push_back(s);
        }
      }
      // Prefer province tiles over metro overlays for long-haul stitching
      // (prd/hk/mo are too small and cause border snap failures).
      auto isMetro = [](const std::string& s) {
        return s == "prd" || s == "hk" || s == "mo";
      };
      std::vector<std::string> filtered;
      for (const auto& s : uniq) {
        if (isMetro(s) && s != homeSuffix && s != destRegion) {
          continue;
        }
        if (filtered.empty() || filtered.back() != s) {
          filtered.push_back(s);
        }
      }
      if (filtered.empty()) {
        filtered = uniq;
      }
      // Cap hop count: home + up to 3 middle + dest. Too few middles left
      // 500–1000km holes that national hwy CH often cannot bridge from border snaps.
      if (filtered.size() > 5) {
        std::vector<std::string> thin;
        thin.push_back(filtered.front());
        const std::size_t midCount = filtered.size() - 2;
        thin.push_back(filtered[1 + midCount / 4]);
        thin.push_back(filtered[1 + midCount / 2]);
        thin.push_back(filtered[1 + (3 * midCount) / 4]);
        thin.push_back(filtered.back());
        filtered.clear();
        for (const auto& s : thin) {
          if (filtered.empty() || filtered.back() != s) {
            filtered.push_back(s);
          }
        }
      }

      std::vector<std::pair<std::string, LatLon>> hops;
      hops.reserve(filtered.size());
      // Home exit: border toward dest. Middle hops: province centers (on-network).
      // Final hop: dest (owned by destFinish; skipped when building shared legs).
      for (std::size_t i = 0; i < filtered.size(); ++i) {
        const RegionBBoxView* box = regionBBoxForSuffix(filtered[i]);
        LatLon exitPt{destLat, destLon};
        if (i + 1 == filtered.size()) {
          exitPt = {destLat, destLon};
          if (box != nullptr) {
            clampIntoRegion(box, exitPt.lat, exitPt.lon);
          }
        } else if (i == 0 && box != nullptr) {
          regionBorderToward(*box, destLat, destLon, exitPt.lat, exitPt.lon);
          clampIntoRegion(box, exitPt.lat, exitPt.lon);
        } else if (box != nullptr) {
          const double cx = 0.5 * (box->minLat + box->maxLat);
          const double cy = 0.5 * (box->minLon + box->maxLon);
          exitPt = {cx + 0.2 * (destLat - cx), cy + 0.2 * (destLon - cy)};
          clampIntoRegion(box, exitPt.lat, exitPt.lon);
        }
        hops.push_back({filtered[i], exitPt});
      }
      return hops;
    };

    if (!remoteVehicles.empty()) {
      std::unordered_map<std::string, std::vector<VehicleInfo>> byHome;
      for (const auto& v : remoteVehicles) {
        const std::string home = resolveRegionalSuffix(graphPath_, v.lat, v.lon);
        byHome[home].push_back(v);
      }

      struct HomePlan {
        std::string home;
        std::vector<VehicleInfo> vehicles;
        std::vector<std::pair<std::string, LatLon>> hops;
      };
      std::vector<HomePlan> plans;
      plans.reserve(byHome.size());
      std::unordered_set<std::string> needRegions;
      for (auto& kv : byHome) {
        if (kv.first.empty()) {
          continue;
        }
        HomePlan plan;
        plan.home = kv.first;
        plan.vehicles = std::move(kv.second);
        plan.hops = buildCorridorWaypoints(plan.home);
        for (const auto& hop : plan.hops) {
          needRegions.insert(hop.first);
        }
        needRegions.insert(plan.home);
        plans.push_back(std::move(plan));
      }

      // Ensure all corridor regions (already preloaded at startup; cheap if hot).
      {
        std::vector<std::string> regionList(needRegions.begin(), needRegions.end());
        std::vector<std::future<void>> futs;
        futs.reserve(regionList.size());
        for (const auto& suffix : regionList) {
          futs.push_back(std::async(std::launch::async, [this, suffix]() {
            std::string err;
            (void)ensureRegionalGraph(suffix, &err);
          }));
        }
        for (auto& f : futs) {
          f.get();
        }
      }

      // Deduplicate shared middle legs across homes: key = region|from|to
      struct SharedLegKey {
        std::string region;
        LatLon from{};
        LatLon to{};
      };
      struct SharedLeg {
        RoutePolyline route;
        double travelSecAt60 = 0.0;
        double distM = 0.0;
        bool ok = false;
      };
      auto legKey = [](const std::string& region, const LatLon& from, const LatLon& to) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s|%.4f,%.4f|%.4f,%.4f", region.c_str(), from.lat,
                      from.lon, to.lat, to.lon);
        return std::string(buf);
      };
      std::vector<SharedLegKey> uniqueLegs;
      std::unordered_map<std::string, std::size_t> legIndex;
      for (const auto& plan : plans) {
        for (std::size_t i = 1; i < plan.hops.size(); ++i) {
          // Dest-province approach is owned by destFinish. Routing the last hop
          // into dest ends at the destination, then appending destFinish walks
          // back to the border — a visible backtrack on the map.
          if (!destRegion.empty() && plan.hops[i].first == destRegion) {
            continue;
          }
          const LatLon from = plan.hops[i - 1].second;
          const LatLon to = plan.hops[i].second;
          // Skip zero-length / near-duplicate hops (e.g. dest already inside region).
          if (haversineMeters(from, to) < 500.0) {
            continue;
          }
          const std::string& region = plan.hops[i].first;
          const std::string key = legKey(region, from, to);
          if (legIndex.count(key) == 0) {
            legIndex[key] = uniqueLegs.size();
            uniqueLegs.push_back({region, from, to});
          }
        }
      }

      std::vector<SharedLeg> sharedPool(uniqueLegs.size());
      // Stitch a few middle provincial hops (capped parallelism). Skipping them
      // left 800–1600km holes on the map (home stub + dest stub only).
      const bool stitchMiddle = true;
      if (stitchMiddle) {
        constexpr std::size_t kMaxParallel = 4;
        for (std::size_t begin = 0; begin < uniqueLegs.size(); begin += kMaxParallel) {
          std::vector<std::future<void>> futs;
          const std::size_t end = std::min(uniqueLegs.size(), begin + kMaxParallel);
          futs.reserve(end - begin);
          for (std::size_t i = begin; i < end; ++i) {
            futs.push_back(std::async(std::launch::async, [&, i]() {
              const auto& leg = uniqueLegs[i];
              // Fast corridor hop with a generous wall — miss falls through to
              // hwy bridge, then to a split gap (no geodesic chord drawn).
              // Fast corridor hop only — miss falls through to hwy / gap split.
              // A second uncapped Full-CH retry here made interactive clicks hang.
              VehicleArrivalResult r = routeLegOnRegion(
                  leg.region, leg.from.lat, leg.from.lon, leg.to.lat, leg.to.lon, 60.0,
                  arriveByUnix - 7 * 86400, "__corridor__", /*corridorFast=*/true);
              if (r.reachable && isDrawableRoad(r.route)) {
                sharedPool[i].ok = true;
                sharedPool[i].route = std::move(r.route);
                sharedPool[i].travelSecAt60 = r.travelDurationSec;
                sharedPool[i].distM = r.routeDistanceM;
                return;
              }
              HwyBridge bridge = routeHwyBridge(leg.from, leg.to);
              if (bridge.ok && isDrawableRoad(bridge.route)) {
                sharedPool[i].ok = true;
                sharedPool[i].route = std::move(bridge.route);
                sharedPool[i].travelSecAt60 = bridge.travelSecAt60;
                sharedPool[i].distM = bridge.distM;
              } else {
                std::cerr << "[mmlp] corridor_leg miss region=" << leg.region << " from=("
                          << leg.from.lat << "," << leg.from.lon << ") to=(" << leg.to.lat << ","
                          << leg.to.lon << ")\n"
                          << std::flush;
              }
            }));
          }
          for (auto& f : futs) {
            f.get();
          }
        }
      }

      // First legs per home — also concurrency-capped.
      std::vector<std::unordered_map<std::string, VehicleArrivalResult>> firstByHome(plans.size());
      {
        constexpr std::size_t kMaxParallel = 4;
        for (std::size_t begin = 0; begin < plans.size(); begin += kMaxParallel) {
          std::vector<std::future<void>> futs;
          const std::size_t end = std::min(plans.size(), begin + kMaxParallel);
          futs.reserve(end - begin);
          for (std::size_t pi = begin; pi < end; ++pi) {
            futs.push_back(std::async(std::launch::async, [&, pi]() {
              const auto& plan = plans[pi];
              if (plan.hops.empty()) {
                return;
              }
              firstByHome[pi] =
                  routeBatchFirstLegs(plan.home, plan.vehicles, plan.hops.front().second.lat,
                                      plan.hops.front().second.lon);
            }));
          }
          for (auto& f : futs) {
            f.get();
          }
        }
      }

      // One shared dest-region finish for the whole request (not per home).
      // Entry = dest-bbox border toward the centroid of remote homes — cheap and
      // usually on-network; a few mid-edge fallbacks if the first miss.
      SharedLeg destFinish;
      if (!destRegion.empty() && !plans.empty()) {
        const RegionBBoxView* dbox = regionBBoxForSuffix(destRegion);
        double homeLat = destLat;
        double homeLon = destLon;
        double wLat = 0.0;
        double wLon = 0.0;
        int wN = 0;
        for (const auto& plan : plans) {
          if (const RegionBBoxView* hbox = regionBBoxForSuffix(plan.home)) {
            wLat += 0.5 * (hbox->minLat + hbox->maxLat);
            wLon += 0.5 * (hbox->minLon + hbox->maxLon);
            ++wN;
          }
        }
        if (wN > 0) {
          homeLat = wLat / wN;
          homeLon = wLon / wN;
        }
        std::vector<LatLon> entries;
        auto pushEntry = [&](double lat, double lon) {
          if (dbox != nullptr) {
            clampIntoRegion(dbox, lat, lon);
          }
          for (const auto& e : entries) {
            if (haversineMeters(e, {lat, lon}) < 5000.0) {
              return;
            }
          }
          entries.push_back({lat, lon});
        };
        if (dbox != nullptr) {
          double eLat = destLat;
          double eLon = destLon;
          regionBorderToward(*dbox, homeLat, homeLon, eLat, eLon);
          pushEntry(eLat, eLon);
          // Mid-edge samples (avoid corners).
          pushEntry(dbox->minLat + 0.05, 0.5 * (dbox->minLon + dbox->maxLon));
          pushEntry(dbox->maxLat - 0.05, 0.5 * (dbox->minLon + dbox->maxLon));
          pushEntry(0.5 * (dbox->minLat + dbox->maxLat), dbox->minLon + 0.05);
          pushEntry(0.5 * (dbox->minLat + dbox->maxLat), dbox->maxLon - 0.05);
        }
        pushEntry(destLat, destLon);
        for (const auto& entry : entries) {
          if (haversineMeters(entry, {destLat, destLon}) <= 500.0) {
            destFinish.ok = true;
            destFinish.route.points = {{destLat, destLon}};
            break;
          }
          VehicleArrivalResult fin =
              routeLegOnRegion(destRegion, entry.lat, entry.lon, destLat, destLon, 60.0,
                               arriveByUnix - 7 * 86400, "__dest_finish__", false);
          if (fin.reachable && isDrawableRoad(fin.route)) {
            destFinish.ok = true;
            destFinish.route = std::move(fin.route);
            destFinish.travelSecAt60 = fin.travelDurationSec;
            destFinish.distM = fin.routeDistanceM;
            break;
          }
        }
        if (!destFinish.ok) {
          std::cerr << "[mmlp] dest_finish miss region=" << destRegion << " tried="
                    << entries.size() << "\n"
                    << std::flush;
        }
      }

      // Append next polyline; fill join gaps with real roads only (never a chord).
      auto appendWithBridge = [&](RoutePolyline& route, double& travel, double& distM,
                                  double speedKmh, const RoutePolyline& next, double nextTravel60,
                                  double nextDistM) {
        if (next.points.empty() || !isDrawableRoad(next)) {
          return;
        }
        if (!route.points.empty()) {
          const LatLon gapFrom = route.points.back();
          const LatLon gapTo = next.points.front();
          const double gap = haversineMeters(gapFrom, gapTo);
          if (gap > 5000.0) {
            HwyBridge bridge = routeHwyBridge(gapFrom, gapTo);
            if (bridge.ok && isDrawableRoad(bridge.route)) {
              appendPolyline(route, bridge.route);
              const double scale = 60.0 / std::max(speedKmh, 1.0);
              travel += bridge.travelSecAt60 * scale;
              distM += bridge.distM;
            } else if (gap > 5000.0) {
              // One regional Full-CH attempt on the midpoint province, then split.
              const std::string suf = resolveRegionalSuffix(
                  graphPath_, 0.5 * (gapFrom.lat + gapTo.lat), 0.5 * (gapFrom.lon + gapTo.lon));
              bool filled = false;
              if (!suf.empty() && suf != "prd" && suf != "hk" && suf != "mo") {
                VehicleArrivalResult r = routeLegOnRegion(
                    suf, gapFrom.lat, gapFrom.lon, gapTo.lat, gapTo.lon, speedKmh,
                    arriveByUnix - 7 * 86400, "__gap_fill__", /*corridorFast=*/true);
                if (r.reachable && isDrawableRoad(r.route)) {
                  appendPolyline(route, r.route);
                  travel += r.travelDurationSec;
                  distM += r.routeDistanceM;
                  filled = true;
                }
              }
              if (!filled) {
                // Leave a split for the UI — do NOT invent a chord.
                travel += (gap * 1.25) / std::max(speedMsFromKmh(speedKmh), 0.5);
                distM += gap * 1.25;
              }
            }
          }
        }
        appendPolyline(route, next);
        const double scale = 60.0 / std::max(speedKmh, 1.0);
        travel += nextTravel60 * scale;
        distM += nextDistM;
      };

      // Attach destFinish only when it advances toward dest. If the corridor
      // already ended near dest, appending entry→dest would walk backward to the
      // border and look broken on the map.
      auto appendDestFinish = [&](RoutePolyline& route, double& travel, double& distM,
                                  double speedKmh) {
        if (!destFinish.ok || destFinish.route.points.empty()) {
          return;
        }
        if (!route.points.empty()) {
          const LatLon cur = route.points.back();
          const LatLon dest{destLat, destLon};
          const double toDest = haversineMeters(cur, dest);
          const LatLon entry = destFinish.route.points.front();
          const double toEntry = haversineMeters(cur, entry);
          if (toDest <= 5000.0) {
            return;
          }
          // Already past the dest entry (closer to dest than to entry): just
          // close to dest, do not rewind to the provincial border.
          if (toDest + 20000.0 < toEntry) {
            return;
          }
        }
        appendWithBridge(route, travel, distM, speedKmh, destFinish.route,
                         destFinish.travelSecAt60, destFinish.distM);
      };

      // Always finish at the clicked destination when possible (real roads only).
      auto closeToDest = [&](RoutePolyline& route, double& travel, double& distM, double speedKmh) {
        if (route.points.empty()) {
          return;
        }
        const LatLon cur = route.points.back();
        const LatLon dest{destLat, destLon};
        const double gap = haversineMeters(cur, dest);
        if (gap <= 5.0) {
          return;
        }
        if (gap <= 5000.0) {
          travel += (gap * 1.25) / std::max(speedMsFromKmh(speedKmh), 0.5);
          distM += gap * 1.25;
          route.points.push_back(dest);
          return;
        }
        HwyBridge bridge = routeHwyBridge(cur, dest);
        if (bridge.ok && isDrawableRoad(bridge.route)) {
          appendWithBridge(route, travel, distM, speedKmh, bridge.route, bridge.travelSecAt60,
                           bridge.distM);
        } else if (!destRegion.empty()) {
          VehicleArrivalResult fin =
              routeLegOnRegion(destRegion, cur.lat, cur.lon, destLat, destLon, speedKmh,
                               arriveByUnix - 7 * 86400, "__close_dest__", false);
          if (fin.reachable && isDrawableRoad(fin.route)) {
            appendWithBridge(route, travel, distM, speedKmh, fin.route, fin.travelDurationSec,
                             fin.routeDistanceM);
          } else {
            travel += (gap * 1.25) / std::max(speedMsFromKmh(speedKmh), 0.5);
            distM += gap * 1.25;
          }
        } else {
          travel += (gap * 1.25) / std::max(speedMsFromKmh(speedKmh), 0.5);
          distM += gap * 1.25;
        }
        if (!route.points.empty()) {
          const double rem = haversineMeters(route.points.back(), dest);
          if (rem > 5.0 && rem <= 5000.0) {
            travel += (rem * 1.25) / std::max(speedMsFromKmh(speedKmh), 0.5);
            distM += rem * 1.25;
            route.points.push_back(dest);
          }
        }
      };

      std::vector<VehicleInfo> remoteLeft;
      for (std::size_t pi = 0; pi < plans.size(); ++pi) {
        const auto& plan = plans[pi];
        if (plan.hops.empty()) {
          remoteLeft.insert(remoteLeft.end(), plan.vehicles.begin(), plan.vehicles.end());
          continue;
        }
        std::vector<SharedLeg> shared;
        if (plan.hops.size() >= 2) {
          for (std::size_t i = 1; i < plan.hops.size(); ++i) {
            if (!destRegion.empty() && plan.hops[i].first == destRegion) {
              continue;
            }
            const LatLon from = plan.hops[i - 1].second;
            const LatLon to = plan.hops[i].second;
            if (haversineMeters(from, to) < 500.0) {
              continue;
            }
            const std::string key = legKey(plan.hops[i].first, from, to);
            auto it = legIndex.find(key);
            if (it == legIndex.end()) {
              continue;
            }
            const SharedLeg& leg = sharedPool[it->second];
            if (leg.ok && !leg.route.points.empty()) {
              shared.push_back(leg);
            }
          }
        }
        const auto& firstLegs = firstByHome[pi];
        for (const auto& vehicle : plan.vehicles) {
          auto itFirst = firstLegs.find(vehicle.id);
          if (itFirst == firstLegs.end() || !itFirst->second.reachable ||
              itFirst->second.route.points.size() < 2) {
            remoteLeft.push_back(vehicle);
            continue;
          }
          const VehicleArrivalResult& first = itFirst->second;
          RoutePolyline route = first.route;
          double travel = first.travelDurationSec;
          double distM = first.routeDistanceM;
          const double speedKmh = vehicle.speed > 1.0 ? vehicle.speed : 60.0;
          for (const auto& leg : shared) {
            appendWithBridge(route, travel, distM, speedKmh, leg.route, leg.travelSecAt60,
                             leg.distM);
          }
          if (destFinish.ok && !destFinish.route.points.empty()) {
            appendDestFinish(route, travel, distM, speedKmh);
          }
          closeToDest(route, travel, distM, speedKmh);
          const double maxHorizon =
              std::min(p.maxTime, static_cast<double>(arriveByUnix - vehicle.timestamp));
          if (travel > maxHorizon + 1e-6) {
            continue;
          }
          VehicleArrivalResult row;
          row.vehicleId = vehicle.id;
          row.reachable = true;
          row.travelDurationSec = travel;
          row.etaUnix = static_cast<double>(vehicle.timestamp) + travel;
          row.routeDistanceM = distM;
          row.route = std::move(route);
          summary.vehicles.push_back(std::move(row));
        }
      }

      if (!remoteLeft.empty()) {
        // Last resort: retry home exit; if still missing, emit dest-finish only
        // (real roads near dest — never a geodesic chord from home).
        std::unordered_map<std::string, std::vector<VehicleInfo>> leftByHome;
        for (const auto& vehicle : remoteLeft) {
          const std::string home = resolveRegionalSuffix(graphPath_, vehicle.lat, vehicle.lon);
          if (!home.empty()) {
            leftByHome[home].push_back(vehicle);
          }
        }
        std::unordered_set<std::string> recovered;
        for (auto& kv : leftByHome) {
          const RegionBBoxView* box = regionBBoxForSuffix(kv.first);
          double bLat = destLat;
          double bLon = destLon;
          if (box != nullptr) {
            regionBorderToward(*box, destLat, destLon, bLat, bLon);
          }
          auto firsts = routeBatchFirstLegs(kv.first, kv.second, bLat, bLon);
          for (const auto& vehicle : kv.second) {
            auto it = firsts.find(vehicle.id);
            if (it == firsts.end() || !it->second.reachable || it->second.route.points.size() < 2) {
              continue;
            }
            VehicleArrivalResult leg = std::move(it->second);
            const double speedKmh = vehicle.speed > 1.0 ? vehicle.speed : 60.0;
            double travel = leg.travelDurationSec;
            double distM = leg.routeDistanceM;
            RoutePolyline route = std::move(leg.route);
            if (destFinish.ok && !destFinish.route.points.empty()) {
              appendDestFinish(route, travel, distM, speedKmh);
            }
            closeToDest(route, travel, distM, speedKmh);
            const double maxHorizon =
                std::min(p.maxTime, static_cast<double>(arriveByUnix - vehicle.timestamp));
            if (travel > maxHorizon + 1e-6) {
              continue;
            }
            recovered.insert(vehicle.id);
            VehicleArrivalResult row;
            row.vehicleId = vehicle.id;
            row.reachable = true;
            row.travelDurationSec = travel;
            row.etaUnix = static_cast<double>(vehicle.timestamp) + travel;
            row.routeDistanceM = distM;
            row.route = std::move(route);
            summary.vehicles.push_back(std::move(row));
          }
        }
        // Dest-finish only for vehicles whose home first-leg never found a road path.
        // Still try a national hwy bridge from the vehicle GPS → dest entry so the
        // map shows a continuous road (not just the last provincial approach).
        if (destFinish.ok && !destFinish.route.points.empty()) {
          for (const auto& vehicle : remoteLeft) {
            if (recovered.count(vehicle.id) != 0) {
              continue;
            }
            const double speedKmh = vehicle.speed > 1.0 ? vehicle.speed : 60.0;
            RoutePolyline route;
            double travel = 0.0;
            double distM = 0.0;
            const LatLon from{vehicle.lat, vehicle.lon};
            const LatLon to = destFinish.route.points.front();
            const double gap = haversineMeters(from, to);
            if (gap > 5000.0) {
              HwyBridge bridge = routeHwyBridge(from, to);
              if (bridge.ok && bridge.route.points.size() >= 2) {
                route = bridge.route;
                travel = bridge.travelSecAt60 * (60.0 / std::max(speedKmh, 1.0));
                distM = bridge.distM;
              }
            }
            if (route.points.empty()) {
              // No drawable home/bridge path — still show dest approach only.
              travel = (gap * 1.25) / std::max(speedMsFromKmh(speedKmh), 0.5);
              distM = gap * 1.25;
            }
            appendDestFinish(route, travel, distM, speedKmh);
            closeToDest(route, travel, distM, speedKmh);
            const double maxHorizon =
                std::min(p.maxTime, static_cast<double>(arriveByUnix - vehicle.timestamp));
            if (travel > maxHorizon + 1e-6) {
              continue;
            }
            VehicleArrivalResult row;
            row.vehicleId = vehicle.id;
            row.reachable = true;
            row.travelDurationSec = travel;
            row.etaUnix = static_cast<double>(vehicle.timestamp) + travel;
            row.routeDistanceM = distM;
            row.route = std::move(route);
            summary.vehicles.push_back(std::move(row));
          }
        }
      }
    }

    if (!localVehicles.empty()) {
      if (!destRegion.empty() && ensureRegionalGraph(destRegion, error)) {
        const GraphFileStore* store = nullptr;
        const SpatialIndex* index = nullptr;
        {
          std::lock_guard<std::mutex> lock(regionalMutex_);
          RegionalGraph& regional = *regionalGraphs_.at(destRegion);
          store = &regional.store;
          index = &regional.ctx.index;
        }
        DestinationArrivalSummary local = predictVehiclesToDestinationIndexed(
            localVehicles, histories, *store, *index, q, padM, p, chOverlay, &graphStore_,
            &ctx_.index, destRegion.c_str());
        if (summary.locationId.empty()) {
          summary.locationId = local.locationId;
        }
        for (auto& row : local.vehicles) {
          summary.vehicles.push_back(std::move(row));
        }
      } else {
        DestinationArrivalSummary local = predictVehiclesToDestinationIndexed(
            localVehicles, histories, graphStore_, ctx_.index, q, padM, p, chOverlay, &graphStore_,
            &ctx_.index, nullptr);
        for (auto& row : local.vehicles) {
          summary.vehicles.push_back(std::move(row));
        }
      }
    }

    sortDestinationArrivals(summary.vehicles, sortBy);
    return summary;
  }

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
