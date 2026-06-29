#include "mmlp/region_loader.hpp"

#include "mmlp/geo.hpp"
#include "mmlp/graph_io.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mmlp {

namespace {

constexpr char kMagic[] = "MMLPGRPH";
constexpr uint32_t kVersion = 1;
constexpr std::size_t kNodeRecord = 28;
constexpr std::size_t kEdgeRecord = 44;

struct EdgeRecord {
  int64_t id = 0;
  int64_t from = 0;
  int64_t to = 0;
  int32_t type = 0;
  double length = 0.0;
  double speedLimit = 0.0;
};

bool inBBox(double lat, double lon, const GeoBBox& b) {
  return b.minLon <= lon && lon <= b.maxLon && b.minLat <= lat && lat <= b.maxLat;
}

double metersToDegLat(double m) { return m / 111000.0; }

double metersToDegLon(double m, double lat) {
  const double c = std::cos(lat * M_PI / 180.0);
  return m / (111000.0 * std::max(0.2, c));
}

bool readHeader(std::ifstream& in, uint64_t& nodeCount, uint64_t& edgeCount, std::string& err) {
  char magic[8];
  if (!in.read(magic, 8)) {
    err = "truncated header";
    return false;
  }
  if (std::string(magic, 8) != kMagic) {
    err = "invalid magic";
    return false;
  }
  uint32_t version = 0;
  if (!in.read(reinterpret_cast<char*>(&version), 4) ||
      !in.read(reinterpret_cast<char*>(&nodeCount), 8) ||
      !in.read(reinterpret_cast<char*>(&edgeCount), 8)) {
    err = "truncated header";
    return false;
  }
  if (version != kVersion) {
    err = "unsupported version";
    return false;
  }
  return true;
}

}  // namespace

GeoBBox expandBBox(const GeoBBox& box, double paddingMeters) {
  const double midLat = 0.5 * (box.minLat + box.maxLat);
  const double dLat = metersToDegLat(paddingMeters);
  const double dLon = metersToDegLon(paddingMeters, midLat);
  GeoBBox out = box;
  out.minLat -= dLat;
  out.maxLat += dLat;
  out.minLon -= dLon;
  out.maxLon += dLon;
  return out;
}

GeoBBox bboxFromVehicles(const std::vector<VehicleInfo>& vehicles, double paddingMeters) {
  GeoBBox box{180, 90, -180, -90};
  for (const auto& v : vehicles) {
    box.minLon = std::min(box.minLon, v.lon);
    box.maxLon = std::max(box.maxLon, v.lon);
    box.minLat = std::min(box.minLat, v.lat);
    box.maxLat = std::max(box.maxLat, v.lat);
  }
  if (vehicles.empty()) {
    return box;
  }
  return expandBBox(box, paddingMeters);
}

bool loadGraphRegionFromFile(const std::string& path, const GeoBBox& bbox,
                             MultimodalGraph& graph, std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return fail("cannot open: " + path);
  }

  uint64_t nodeCount = 0;
  uint64_t edgeCount = 0;
  std::string err;
  if (!readHeader(in, nodeCount, edgeCount, err)) {
    return fail(err);
  }

  graph.clear();
  std::unordered_set<int64_t> seedNodeIds;
  std::unordered_map<int64_t, Node> allNodesNeeded;

  // Pass 1: seed nodes inside bbox
  for (uint64_t i = 0; i < nodeCount; ++i) {
    char buf[kNodeRecord];
    if (!in.read(buf, kNodeRecord)) {
      return fail("truncated node");
    }
    int64_t id = 0;
    double lat = 0.0;
    double lon = 0.0;
    int32_t kind = 0;
    std::memcpy(&id, buf, 8);
    std::memcpy(&lat, buf + 8, 8);
    std::memcpy(&lon, buf + 16, 8);
    std::memcpy(&kind, buf + 24, 4);
    if (!inBBox(lat, lon, bbox)) {
      continue;
    }
    Node node;
    node.id = id;
    node.lat = lat;
    node.lon = lon;
    node.kind = static_cast<NodeKind>(kind);
    allNodesNeeded[id] = node;
    seedNodeIds.insert(id);
  }

  // Pass 2: edges touching seed region; pull in outside endpoints for connectivity
  std::vector<EdgeRecord> regionEdges;
  regionEdges.reserve(200000);

  for (uint64_t i = 0; i < edgeCount; ++i) {
    char buf[kEdgeRecord];
    if (!in.read(buf, kEdgeRecord)) {
      return fail("truncated edge");
    }
    EdgeRecord e;
    std::memcpy(&e.id, buf, 8);
    std::memcpy(&e.from, buf + 8, 8);
    std::memcpy(&e.to, buf + 16, 8);
    std::memcpy(&e.type, buf + 24, 4);
    std::memcpy(&e.length, buf + 28, 8);
    std::memcpy(&e.speedLimit, buf + 36, 8);

    const bool touch = seedNodeIds.count(e.from) > 0 || seedNodeIds.count(e.to) > 0;
    if (!touch) {
      continue;
    }
    regionEdges.push_back(e);
  }

  std::unordered_set<int64_t> needExtra;
  for (const auto& e : regionEdges) {
    if (seedNodeIds.count(e.from) == 0) {
      needExtra.insert(e.from);
    }
    if (seedNodeIds.count(e.to) == 0) {
      needExtra.insert(e.to);
    }
  }

  if (!needExtra.empty()) {
    in.clear();
    in.seekg(28, std::ios::beg);
    for (uint64_t i = 0; i < nodeCount; ++i) {
      char buf[kNodeRecord];
      if (!in.read(buf, kNodeRecord)) {
        break;
      }
      int64_t id = 0;
      double lat = 0.0;
      double lon = 0.0;
      int32_t kind = 0;
      std::memcpy(&id, buf, 8);
      if (needExtra.count(id) == 0) {
        continue;
      }
      std::memcpy(&lat, buf + 8, 8);
      std::memcpy(&lon, buf + 16, 8);
      std::memcpy(&kind, buf + 24, 4);
      Node node;
      node.id = id;
      node.lat = lat;
      node.lon = lon;
      node.kind = static_cast<NodeKind>(kind);
      allNodesNeeded[id] = node;
    }
  }

  for (const auto& kv : allNodesNeeded) {
    graph.addNode(kv.second);
  }
  for (const auto& e : regionEdges) {
    if (allNodesNeeded.count(e.from) == 0 || allNodesNeeded.count(e.to) == 0) {
      continue;
    }
    Edge edge;
    edge.id = e.id;
    edge.from = e.from;
    edge.to = e.to;
    edge.type = static_cast<EdgeType>(e.type);
    edge.length = e.length;
    edge.speedLimit = e.speedLimit;
    graph.addEdge(edge);
  }

  if (graph.nodes().empty()) {
    return fail("no nodes in bbox (check coordinates or expand region)");
  }
  return true;
}

bool loadGraphContextRegion(const std::string& path, const GeoBBox& bbox, GraphContext& ctx,
                            std::string* error) {
  if (!loadGraphRegionFromFile(path, bbox, ctx.graph, error)) {
    return false;
  }
  ctx.index.build(ctx.graph);
  return true;
}

bool extractGraphContextInBBox(const GraphContext& full, const GeoBBox& bbox, GraphContext& out,
                             std::string* error, bool buildLocalIndex) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  std::unordered_set<int64_t> edgeIds;
  full.index.collectEdgesInBBox(bbox, edgeIds);
  if (edgeIds.empty()) {
    return fail("no graph edges in bbox");
  }

  std::unordered_map<int64_t, Node> nodesNeeded;
  nodesNeeded.reserve(edgeIds.size() * 2);
  for (int64_t edgeId : edgeIds) {
    const Edge* edge = full.graph.findEdge(edgeId);
    if (edge == nullptr) {
      continue;
    }
    const Node* from = full.graph.findNode(edge->from);
    const Node* to = full.graph.findNode(edge->to);
    if (from != nullptr) {
      nodesNeeded[from->id] = *from;
    }
    if (to != nullptr) {
      nodesNeeded[to->id] = *to;
    }
  }

  out.graph.clear();
  out.graph.reserveGraph(nodesNeeded.size(), edgeIds.size());
  for (const auto& kv : nodesNeeded) {
    out.graph.addNodeBulk(kv.second);
  }
  for (int64_t edgeId : edgeIds) {
    const Edge* edge = full.graph.findEdge(edgeId);
    if (edge != nullptr) {
      out.graph.addEdgeBulk(*edge);
    }
  }

  if (out.graph.nodes().empty()) {
    return fail("extracted empty subgraph");
  }
  if (buildLocalIndex) {
    out.index.build(out.graph);
  }
  return true;
}

namespace {

void collectCorridorEdges(const GraphContext& full, const LatLon& a, const LatLon& b,
                         double widthMeters, std::unordered_set<int64_t>& edgeIds) {
  const GeoBBox box = bboxAroundSegment(a, b, widthMeters);
  std::unordered_set<int64_t> candidates;
  full.index.collectEdgesInBBox(box, candidates);
  for (int64_t edgeId : candidates) {
    const Edge* edge = full.graph.findEdge(edgeId);
    if (edge == nullptr) {
      continue;
    }
    const Node* from = full.graph.findNode(edge->from);
    const Node* to = full.graph.findNode(edge->to);
    if (from == nullptr || to == nullptr) {
      continue;
    }
    const LatLon mid{0.5 * (from->lat + to->lat), 0.5 * (from->lon + to->lon)};
    if (pointToSegmentDistanceLatLon(mid, a, b) <= widthMeters) {
      edgeIds.insert(edgeId);
    }
  }
}

bool buildGraphFromEdgeIds(const GraphContext& full, const std::unordered_set<int64_t>& edgeIds,
                           GraphContext& out, std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };
  if (edgeIds.empty()) {
    return fail("no graph edges in corridor");
  }

  std::unordered_map<int64_t, Node> nodesNeeded;
  nodesNeeded.reserve(edgeIds.size() * 2);
  for (int64_t edgeId : edgeIds) {
    const Edge* edge = full.graph.findEdge(edgeId);
    if (edge == nullptr) {
      continue;
    }
    const Node* from = full.graph.findNode(edge->from);
    const Node* to = full.graph.findNode(edge->to);
    if (from != nullptr) {
      nodesNeeded[from->id] = *from;
    }
    if (to != nullptr) {
      nodesNeeded[to->id] = *to;
    }
  }

  out.graph.clear();
  out.graph.reserveGraph(nodesNeeded.size(), edgeIds.size());
  for (const auto& kv : nodesNeeded) {
    out.graph.addNodeBulk(kv.second);
  }
  for (int64_t edgeId : edgeIds) {
    const Edge* edge = full.graph.findEdge(edgeId);
    if (edge != nullptr) {
      out.graph.addEdgeBulk(*edge);
    }
  }
  if (out.graph.nodes().empty()) {
    return fail("extracted empty subgraph");
  }
  return true;
}

}  // namespace

bool extractGraphContextForMeeting(const GraphContext& full, const VehicleInfo& focal,
                                   const std::vector<VehicleInfo>& partners,
                                   double maxCorridorWidthM, GraphContext& out,
                                   std::string* error) {
  const LatLon focalLl{focal.lat, focal.lon};
  std::unordered_set<int64_t> edgeIds;
  edgeIds.reserve(50000);

  for (const auto& partner : partners) {
    if (partner.id == focal.id) {
      continue;
    }
    const LatLon partnerLl{partner.lat, partner.lon};
    const double dist = haversineMeters(focalLl, partnerLl);
    const double width = std::min(maxCorridorWidthM, dist * 0.35 + 12000.0);
    collectCorridorEdges(full, focalLl, partnerLl, width, edgeIds);
  }

  return buildGraphFromEdgeIds(full, edgeIds, out, error);
}

bool extractGraphContextNearVehicles(const GraphContext& full,
                                     const std::vector<VehicleInfo>& vehicles,
                                     double radiusMeters, GraphContext& out,
                                     std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };
  if (vehicles.empty()) {
    return fail("no vehicles");
  }

  std::unordered_set<int64_t> edgeIds;
  for (const auto& v : vehicles) {
    full.index.collectEdgesInRadius(full.graph, v.lat, v.lon, radiusMeters, edgeIds);
  }
  if (edgeIds.empty()) {
    return fail("no graph edges near vehicles");
  }

  std::unordered_map<int64_t, Node> nodesNeeded;
  nodesNeeded.reserve(edgeIds.size() * 2);
  for (int64_t edgeId : edgeIds) {
    const Edge* edge = full.graph.findEdge(edgeId);
    if (edge == nullptr) {
      continue;
    }
    const Node* from = full.graph.findNode(edge->from);
    const Node* to = full.graph.findNode(edge->to);
    if (from != nullptr) {
      nodesNeeded[from->id] = *from;
    }
    if (to != nullptr) {
      nodesNeeded[to->id] = *to;
    }
  }

  out.graph.clear();
  out.graph.reserveGraph(nodesNeeded.size(), edgeIds.size());
  for (const auto& kv : nodesNeeded) {
    out.graph.addNodeBulk(kv.second);
  }
  for (int64_t edgeId : edgeIds) {
    const Edge* edge = full.graph.findEdge(edgeId);
    if (edge != nullptr) {
      out.graph.addEdgeBulk(*edge);
    }
  }
  if (out.graph.nodes().empty()) {
    return fail("extracted empty subgraph");
  }
  return true;
}

bool loadGraphContextFull(const std::string& path, GraphContext& ctx, std::string* error) {
  std::cerr << "[mmlp] loading full graph from " << path << " ...\n" << std::flush;
  if (!loadGraphFromFile(path, ctx.graph, error)) {
    return false;
  }
  std::cerr << "[mmlp] building spatial index ...\n" << std::flush;
  ctx.index.build(ctx.graph);
  return true;
}

bool loadGraphContextIndexOnly(const std::string& binPath, GraphContext& ctx, std::string* error) {
  if (!graphAuxiliaryReady(binPath)) {
    if (error) {
      *error = "missing .sidx/.nidx/.eidx — run: python3 tools/build_graph_auxiliary.py " + binPath;
    }
    return false;
  }
  const auto dot = binPath.rfind('.');
  const std::string base = (dot == std::string::npos) ? binPath : binPath.substr(0, dot);
  std::cerr << "[mmlp] loading spatial index from " << base << ".sidx ...\n" << std::flush;
  ctx.graph.clear();
  if (!ctx.index.loadFromFile(base + ".sidx", error)) {
    return false;
  }
  return true;
}

namespace {

void collectCorridorEdgeIdsIndexed(const GraphFileStore& store, const SpatialIndex& index,
                                   const LatLon& a, const LatLon& b, double widthMeters,
                                   std::unordered_set<int64_t>& edgeIds) {
  const GeoBBox box = bboxAroundSegment(a, b, widthMeters);
  std::unordered_set<int64_t> candidates;
  candidates.reserve(80000);
  index.collectEdgesInBBox(box, candidates);

  for (int64_t edgeId : candidates) {
    double flat = 0.0;
    double flon = 0.0;
    double tlat = 0.0;
    double tlon = 0.0;
    if (!store.edgeEndpointLatLon(edgeId, flat, flon, tlat, tlon)) {
      continue;
    }
    const LatLon mid{0.5 * (flat + tlat), 0.5 * (flon + tlon)};
    if (pointToSegmentDistanceLatLon(mid, a, b) <= widthMeters) {
      edgeIds.insert(edgeId);
    }
  }
}

}  // namespace

bool extractGraphContextForMeetingIndexed(const GraphFileStore& store, const SpatialIndex& index,
                                          const VehicleInfo& focal,
                                          const std::vector<VehicleInfo>& partners,
                                          double maxCorridorWidthM, GraphContext& out,
                                          std::string* error) {
  const LatLon focalLl{focal.lat, focal.lon};
  std::unordered_set<int64_t> edgeIds;
  edgeIds.reserve(50000);

  for (const auto& partner : partners) {
    if (partner.id == focal.id) {
      continue;
    }
    const LatLon partnerLl{partner.lat, partner.lon};
    const double dist = haversineMeters(focalLl, partnerLl);
    const double width = std::min(maxCorridorWidthM, dist * 0.35 + 12000.0);
    collectCorridorEdgeIdsIndexed(store, index, focalLl, partnerLl, width, edgeIds);
  }

  if (!store.loadGraphSubset(edgeIds, out.graph, error)) {
    return false;
  }
  return true;
}

bool extractGraphContextForPairIndexed(const GraphFileStore& store, const SpatialIndex& index,
                                       const VehicleInfo& vehicle, double destLat, double destLon,
                                       double maxCorridorWidthM, GraphContext& out,
                                       std::string* error) {
  const LatLon vehicleLl{vehicle.lat, vehicle.lon};
  const LatLon destLl{destLat, destLon};
  const double dist = haversineMeters(vehicleLl, destLl);
  const double width = std::min(maxCorridorWidthM, dist * 0.35 + 12000.0);

  std::unordered_set<int64_t> edgeIds;
  edgeIds.reserve(50000);
  collectCorridorEdgeIdsIndexed(store, index, vehicleLl, destLl, width, edgeIds);

  const GeoBBox destBox = bboxAroundSegment(destLl, destLl, 8000.0);
  index.collectEdgesInBBox(destBox, edgeIds);

  const GeoBBox vehBox = bboxAroundSegment(vehicleLl, vehicleLl, 8000.0);
  index.collectEdgesInBBox(vehBox, edgeIds);

  if (edgeIds.empty()) {
    if (error) {
      *error = "no edges in vehicle-destination corridor";
    }
    return false;
  }
  if (!store.loadGraphSubset(edgeIds, out.graph, error)) {
    return false;
  }
  return true;
}

bool extractGraphContextForDestination(const GraphContext& full,
                                       const std::vector<VehicleInfo>& vehicles,
                                       double paddingMeters, GraphContext& out,
                                       std::string* error) {
  if (vehicles.empty()) {
    if (error) {
      *error = "no vehicles for destination extract";
    }
    return false;
  }
  const GeoBBox box = bboxFromVehicles(vehicles, paddingMeters);
  return extractGraphContextInBBox(full, box, out, error, false);
}

bool extractGraphContextForDestinationIndexed(const GraphFileStore& store,
                                              const SpatialIndex& index,
                                              const std::vector<VehicleInfo>& vehicles,
                                              double paddingMeters, GraphContext& out,
                                              std::string* error) {
  std::unordered_set<int64_t> edgeIds;
  if (!collectDestinationBBoxEdgeIdsIndexed(store, index, vehicles, paddingMeters, edgeIds,
                                          error)) {
    return false;
  }
  if (!store.loadGraphSubset(edgeIds, out.graph, error)) {
    return false;
  }
  return true;
}

bool collectDestinationBBoxEdgeIdsIndexed(const GraphFileStore& store, const SpatialIndex& index,
                                          const std::vector<VehicleInfo>& vehicles,
                                          double paddingMeters,
                                          std::unordered_set<int64_t>& edgeIds,
                                          std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };
  if (vehicles.empty()) {
    return fail("no vehicles for destination extract");
  }

  const GeoBBox box = bboxFromVehicles(vehicles, paddingMeters);
  index.collectEdgesInBBox(box, edgeIds);
  if (edgeIds.empty()) {
    return fail("no graph edges in destination bbox");
  }
  return true;
}

bool collectDestinationCorridorEdgeIdsIndexed(
    const GraphFileStore& store, const SpatialIndex& index,
    const std::vector<VehicleInfo>& vehicles, double destLat, double destLon,
    double maxCorridorWidthM, std::unordered_set<int64_t>& edgeIds, std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };
  if (vehicles.empty()) {
    return fail("no vehicles for corridor extract");
  }

  const LatLon destLl{destLat, destLon};
  edgeIds.reserve(80000);

  for (const auto& vehicle : vehicles) {
    if (vehicle.id == "__destination__") {
      continue;
    }
    const LatLon vehLl{vehicle.lat, vehicle.lon};
    const double dist = haversineMeters(vehLl, destLl);
    const double width =
        dist > 40000.0 ? std::min(maxCorridorWidthM, 22000.0)
                       : std::min(maxCorridorWidthM, dist * 0.35 + 12000.0);
    collectCorridorEdgeIdsIndexed(store, index, vehLl, destLl, width, edgeIds);
    const GeoBBox vehBox = bboxAroundSegment(vehLl, vehLl, 8000.0);
    index.collectEdgesInBBox(vehBox, edgeIds);
  }

  const GeoBBox destBox = bboxAroundSegment(destLl, destLl, 8000.0);
  index.collectEdgesInBBox(destBox, edgeIds);

  if (edgeIds.empty()) {
    return fail("no edges in destination corridors");
  }
  return true;
}

bool extractGraphContextForDestinationCorridorsIndexed(
    const GraphFileStore& store, const SpatialIndex& index,
    const std::vector<VehicleInfo>& vehicles, double destLat, double destLon,
    double maxCorridorWidthM, GraphContext& out, std::string* error) {
  std::unordered_set<int64_t> edgeIds;
  if (!collectDestinationCorridorEdgeIdsIndexed(store, index, vehicles, destLat, destLon,
                                                maxCorridorWidthM, edgeIds, error)) {
    return false;
  }
  if (!store.loadGraphSubset(edgeIds, out.graph, error)) {
    return false;
  }
  return true;
}

}  // namespace mmlp
