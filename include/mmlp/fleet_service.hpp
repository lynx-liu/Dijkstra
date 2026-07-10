#pragma once

#include "mmlp/arrival.hpp"
#include "mmlp/fleet_index.hpp"
#include "mmlp/graph_store.hpp"
#include "mmlp/predict.hpp"
#include "mmlp/region_loader.hpp"

#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace mmlp {

// Long-running service: ingest vehicles one-by-one, return fastest meeting vs fleet.
class FleetMeetingService {
 public:
  explicit FleetMeetingService(std::string graphPath, double regionPaddingMeters = 150000.0);

  const std::string& graphPath() const { return graphPath_; }

  // Load regional graph without vehicles (legacy; use preloadFullGraph for nationwide).
  bool preloadRegion(const GeoBBox& bbox, std::string* error = nullptr);

  // Load entire graph + index at startup (slow ~5min; use preloadIndexOnly when .sidx exists).
  bool preloadFullGraph(std::string* error = nullptr);

  // Load prebuilt spatial index only (~seconds); subgraphs loaded on demand.
  bool preloadIndexOnly(std::string* error = nullptr);

  bool isFullGraphLoaded() const { return fullGraphLoaded_; }
  bool isIndexOnlyMode() const { return indexOnlyLoaded_; }

  // Add or update a vehicle (same id overwrites). Returns best meeting for this vehicle.
  FocalBestMeeting ingestVehicle(const VehicleInfo& vehicle,
                                 const VehicleHistory* history = nullptr,
                                 std::string* error = nullptr);

  // Stateless batch: fleet[0] vs each other vehicle; sorted by meetDuration (shortest first).
  std::vector<FocalBestMeeting> meetingsWithLead(
      const std::vector<VehicleInfo>& vehicles,
      const std::vector<VehicleHistory>& histories, std::string* error = nullptr);

  // Vehicles that reach (destLat, destLon) before arriveByUnix; sorted fastest first.
  // If overrideVehicles is null or empty, uses ingested fleet.
  DestinationArrivalSummary vehiclesReachDestinationBy(
      double destLat, double destLon, int64_t arriveByUnix,
      VehicleType destType = VehicleType::TRUCK,
      ArrivalSortBy sortBy = ArrivalSortBy::DURATION,
      const std::vector<VehicleInfo>* overrideVehicles = nullptr,
      const std::vector<VehicleHistory>* overrideHistories = nullptr,
      std::string* error = nullptr);

  bool removeVehicle(const std::string& vehicleId);
  void clearFleet();

  std::size_t fleetSize() const { return fleet_.size(); }
  bool hasGraph() const { return graphReady_; }

 private:
  bool ensureGraphForFleet(std::string* error);
  bool ensureGraphForVehicles(const std::vector<VehicleInfo>& vehicles, std::string* error);
  static bool bboxContains(const GeoBBox& outer, const GeoBBox& inner);
  static std::vector<VehicleInfo> fleetSnapshot(const std::unordered_map<std::string, VehicleInfo>& fleet);
  static std::vector<VehicleHistory> historySnapshot(
      const std::unordered_map<std::string, VehicleHistory>& histories);

  bool getOrBuildSubgraph(const VehicleInfo& focal, const std::vector<VehicleInfo>& partners,
                          double corridorWidthM, const GraphContext*& sub, std::string* error);

  bool getOrBuildDestinationSubgraph(const std::vector<VehicleInfo>& vehicles,
                                     double paddingMeters, const GraphContext*& sub,
                                     std::string* error);

  const char* regionSuffixForPoint(double lat, double lon) const;
  bool ensureRegionalGraph(const std::string& suffix, std::string* error);

  std::string graphPath_;
  double paddingMeters_;
  GraphFileStore graphStore_;
  GraphContext ctx_;
  struct RegionalGraph {
    GraphFileStore store;
    GraphContext ctx;
    bool ready = false;
    bool fullGraph = false;
  };
  std::unordered_map<std::string, std::unique_ptr<RegionalGraph>> regionalGraphs_;
  mutable std::mutex regionalMutex_;
  GeoBBox loadedBBox_{};
  bool graphReady_ = false;
  bool fullGraphLoaded_ = false;
  bool indexOnlyLoaded_ = false;

  std::unordered_map<std::string, VehicleInfo> fleet_;
  std::unordered_map<std::string, VehicleHistory> histories_;
  FleetSpatialIndex fleetIndex_;

  struct SubgraphCacheEntry {
    uint64_t key = 0;
    GraphContext ctx;
  };
  std::unordered_map<uint64_t, SubgraphCacheEntry> subgraphCache_;
  std::list<uint64_t> subgraphCacheLru_;
  static constexpr std::size_t kSubgraphCacheMax = 48;
};

}  // namespace mmlp
