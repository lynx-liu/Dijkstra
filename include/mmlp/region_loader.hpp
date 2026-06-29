#pragma once

#include "mmlp/bbox.hpp"
#include "mmlp/graph.hpp"
#include "mmlp/graph_store.hpp"
#include "mmlp/spatial_index.hpp"
#include "mmlp/types.hpp"

#include <string>
#include <vector>

namespace mmlp {

GeoBBox bboxFromVehicles(const std::vector<VehicleInfo>& vehicles, double paddingMeters);

GeoBBox expandBBox(const GeoBBox& box, double paddingMeters);

// Stream-load only nodes/edges inside bbox (no full 5GB graph in RAM).
bool loadGraphRegionFromFile(const std::string& path, const GeoBBox& bbox,
                             MultimodalGraph& graph, std::string* error = nullptr);

struct GraphContext {
  MultimodalGraph graph;
  SpatialIndex index;
};

// Load regional graph + build spatial index for matching/routing.
bool loadGraphContextRegion(const std::string& path, const GeoBBox& bbox, GraphContext& ctx,
                            std::string* error = nullptr);

// Load full graph + spatial index (same as loadGraphContextFull in graph_load.hpp).
bool loadGraphContextFull(const std::string& path, GraphContext& ctx,
                          std::string* error = nullptr);

// Fast startup: load .sidx only; graph filled on demand per request.
bool loadGraphContextIndexOnly(const std::string& binPath, GraphContext& ctx,
                               std::string* error = nullptr);

// Extract a regional subgraph from an already-loaded nationwide context (fast).
bool extractGraphContextInBBox(const GraphContext& full, const GeoBBox& bbox, GraphContext& out,
                               std::string* error = nullptr, bool buildLocalIndex = false);

// Union of road/rail discs around each vehicle (tighter than rectangular fleet bbox).
bool extractGraphContextNearVehicles(const GraphContext& full,
                                     const std::vector<VehicleInfo>& vehicles,
                                     double radiusMeters, GraphContext& out,
                                     std::string* error = nullptr);

// Accurate online extract: union of corridors focal→each partner (not midpoint guess).
bool extractGraphContextForMeeting(const GraphContext& full, const VehicleInfo& focal,
                                   const std::vector<VehicleInfo>& partners,
                                   double maxCorridorWidthM, GraphContext& out,
                                   std::string* error = nullptr);

// Index-only: mmap filter corridor edges, then sequential subset load (<1s typical).
bool extractGraphContextForMeetingIndexed(const GraphFileStore& store, const SpatialIndex& index,
                                         const VehicleInfo& focal,
                                         const std::vector<VehicleInfo>& partners,
                                         double maxCorridorWidthM, GraphContext& out,
                                         std::string* error = nullptr);

// Index-only: corridor between one vehicle and a destination point (not national bbox).
bool extractGraphContextForPairIndexed(const GraphFileStore& store, const SpatialIndex& index,
                                       const VehicleInfo& vehicle, double destLat, double destLon,
                                       double maxCorridorWidthM, GraphContext& out,
                                       std::string* error = nullptr);

// Destination batch: one bbox around all vehicles + destination (faster than N corridors).
bool extractGraphContextForDestination(const GraphContext& full,
                                       const std::vector<VehicleInfo>& vehicles,
                                       double paddingMeters, GraphContext& out,
                                       std::string* error = nullptr);

bool extractGraphContextForDestinationIndexed(const GraphFileStore& store,
                                              const SpatialIndex& index,
                                              const std::vector<VehicleInfo>& vehicles,
                                              double paddingMeters, GraphContext& out,
                                              std::string* error = nullptr);

}  // namespace mmlp
