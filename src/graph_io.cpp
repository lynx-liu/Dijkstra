#include "mmlp/graph_io.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace mmlp {

namespace {

constexpr char kMagic[] = "MMLPGRPH";
constexpr uint32_t kFormatVersion = 1;

struct FileHeader {
  char magic[8];
  uint32_t version = 0;
  uint64_t nodeCount = 0;
  uint64_t edgeCount = 0;
};

bool readBytes(std::ifstream& in, void* data, std::size_t n) {
  in.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(n));
  return static_cast<std::size_t>(in.gcount()) == n;
}

}  // namespace

bool loadGraphFromFile(const std::string& path, MultimodalGraph& graph, std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return fail("cannot open file: " + path);
  }

  FileHeader header{};
  if (!readBytes(in, header.magic, sizeof(header.magic))) {
    return fail("truncated header");
  }
  if (std::string(header.magic, 8) != kMagic) {
    return fail("invalid magic (not an mmlp graph file)");
  }
  if (!readBytes(in, &header.version, sizeof(header.version)) ||
      !readBytes(in, &header.nodeCount, sizeof(header.nodeCount)) ||
      !readBytes(in, &header.edgeCount, sizeof(header.edgeCount))) {
    return fail("truncated header");
  }
  if (header.version != kFormatVersion) {
    return fail("unsupported graph version");
  }

  graph.clear();
  graph.reserveGraph(static_cast<std::size_t>(header.nodeCount),
                     static_cast<std::size_t>(header.edgeCount));

  const uint64_t nodeReportEvery = std::max<uint64_t>(1, header.nodeCount / 20);
  for (uint64_t i = 0; i < header.nodeCount; ++i) {
    int64_t id = 0;
    double lat = 0.0;
    double lon = 0.0;
    int32_t kind = 0;
    if (!readBytes(in, &id, sizeof(id)) || !readBytes(in, &lat, sizeof(lat)) ||
        !readBytes(in, &lon, sizeof(lon)) || !readBytes(in, &kind, sizeof(kind))) {
      return fail("truncated node record");
    }
    Node node;
    node.id = id;
    node.lat = lat;
    node.lon = lon;
    node.kind = static_cast<NodeKind>(kind);
    graph.addNodeBulk(std::move(node));
    if (i > 0 && i % nodeReportEvery == 0) {
      std::cerr << "[mmlp] load nodes " << (100 * i / header.nodeCount) << "%\n" << std::flush;
    }
  }

  const uint64_t edgeReportEvery = std::max<uint64_t>(1, header.edgeCount / 20);
  for (uint64_t i = 0; i < header.edgeCount; ++i) {
    int64_t id = 0;
    int64_t from = 0;
    int64_t to = 0;
    int32_t type = 0;
    double length = 0.0;
    double speedLimit = 0.0;
    if (!readBytes(in, &id, sizeof(id)) || !readBytes(in, &from, sizeof(from)) ||
        !readBytes(in, &to, sizeof(to)) || !readBytes(in, &type, sizeof(type)) ||
        !readBytes(in, &length, sizeof(length)) ||
        !readBytes(in, &speedLimit, sizeof(speedLimit))) {
      return fail("truncated edge record");
    }
    Edge edge;
    edge.id = id;
    edge.from = from;
    edge.to = to;
    edge.type = static_cast<EdgeType>(type);
    edge.length = length;
    edge.speedLimit = speedLimit;
    graph.addEdgeBulk(edge);
    if (i > 0 && i % edgeReportEvery == 0) {
      std::cerr << "[mmlp] load edges " << (100 * i / header.edgeCount) << "%\n" << std::flush;
    }
  }

  if (!in) {
    return fail("read error after records");
  }

  std::cerr << "[mmlp] graph loaded nodes=" << graph.nodes().size()
            << " edges=" << graph.edges().size() << "\n"
            << std::flush;
  return true;
}

namespace {

constexpr char kNdxMagic[8] = {'M', 'M', 'L', 'P', 'N', 'D', 'X', '\0'};
constexpr char kSidxMagic[8] = {'M', 'M', 'L', 'P', 'S', 'I', 'D', 'X'};
constexpr std::size_t kNodeRecord = 28;
constexpr std::size_t kEdgeRecord = 44;

struct IdOffset {
  int64_t id = 0;
  uint64_t offset = 0;
};

bool loadIdOffsetTable(const std::string& path, std::vector<IdOffset>& table, std::string* error) {
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
  char magic[8];
  if (!in.read(magic, 8) || std::memcmp(magic, kNdxMagic, 8) != 0) {
    return fail("invalid index magic: " + path);
  }
  uint32_t version = 0;
  uint64_t count = 0;
  if (!in.read(reinterpret_cast<char*>(&version), 4) ||
      !in.read(reinterpret_cast<char*>(&count), 8)) {
    return fail("truncated index header");
  }
  if (version != 1) {
    return fail("unsupported index version");
  }
  table.resize(static_cast<std::size_t>(count));
  for (uint64_t i = 0; i < count; ++i) {
    if (!in.read(reinterpret_cast<char*>(&table[static_cast<std::size_t>(i)].id), 8) ||
        !in.read(reinterpret_cast<char*>(&table[static_cast<std::size_t>(i)].offset), 8)) {
      return fail("truncated index record");
    }
  }
  return true;
}

const IdOffset* findOffset(const std::vector<IdOffset>& table, int64_t id) {
  auto it = std::lower_bound(table.begin(), table.end(), id,
                             [](const IdOffset& row, int64_t key) { return row.id < key; });
  if (it == table.end() || it->id != id) {
    return nullptr;
  }
  return &*it;
}

}  // namespace

bool loadSpatialIndexFromFile(const std::string& sidxPath, SpatialIndex& index, std::string* error) {
  return index.loadFromFile(sidxPath, error);
}

bool graphAuxiliaryReady(const std::string& binPath) {
  const auto dot = binPath.rfind('.');
  const std::string base = (dot == std::string::npos) ? binPath : binPath.substr(0, dot);
  std::ifstream sidx(base + ".sidx", std::ios::binary);
  std::ifstream nidx(base + ".nidx", std::ios::binary);
  std::ifstream eidx(base + ".eidx", std::ios::binary);
  return static_cast<bool>(sidx) && static_cast<bool>(nidx) && static_cast<bool>(eidx);
}

bool loadGraphSubsetFromFile(const std::string& binPath,
                             const std::unordered_set<int64_t>& edgeIds, MultimodalGraph& graph,
                             std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };
  if (edgeIds.empty()) {
    return fail("empty edge set");
  }

  const auto dot = binPath.rfind('.');
  const std::string base = (dot == std::string::npos) ? binPath : binPath.substr(0, dot);

  std::vector<IdOffset> nodeTable;
  std::vector<IdOffset> edgeTable;
  if (!loadIdOffsetTable(base + ".nidx", nodeTable, error) ||
      !loadIdOffsetTable(base + ".eidx", edgeTable, error)) {
    return false;
  }

  std::ifstream in(binPath, std::ios::binary);
  if (!in) {
    return fail("cannot open: " + binPath);
  }
  char magic[8];
  uint32_t version = 0;
  uint64_t nodeCount = 0;
  uint64_t edgeCount = 0;
  if (!in.read(magic, 8) || std::string(magic, 8) != kMagic ||
      !in.read(reinterpret_cast<char*>(&version), 4) ||
      !in.read(reinterpret_cast<char*>(&nodeCount), 8) ||
      !in.read(reinterpret_cast<char*>(&edgeCount), 8) || version != kFormatVersion) {
    return fail("invalid graph header");
  }

  std::unordered_set<int64_t> nodeIds;
  nodeIds.reserve(edgeIds.size() * 2);
  std::vector<Edge> edges;
  edges.reserve(edgeIds.size());

  for (int64_t edgeId : edgeIds) {
    const IdOffset* row = findOffset(edgeTable, edgeId);
    if (row == nullptr) {
      continue;
    }
    in.seekg(static_cast<std::streamoff>(row->offset));
    Edge edge;
    int32_t type = 0;
    if (!in.read(reinterpret_cast<char*>(&edge.id), 8) ||
        !in.read(reinterpret_cast<char*>(&edge.from), 8) ||
        !in.read(reinterpret_cast<char*>(&edge.to), 8) ||
        !in.read(reinterpret_cast<char*>(&type), 4) ||
        !in.read(reinterpret_cast<char*>(&edge.length), 8) ||
        !in.read(reinterpret_cast<char*>(&edge.speedLimit), 8)) {
      return fail("truncated edge at offset");
    }
    edge.type = static_cast<EdgeType>(type);
    nodeIds.insert(edge.from);
    nodeIds.insert(edge.to);
    edges.push_back(edge);
  }

  if (edges.empty()) {
    return fail("no edges resolved from subset");
  }

  graph.clear();
  graph.reserveGraph(nodeIds.size(), edges.size());

  for (int64_t nodeId : nodeIds) {
    const IdOffset* row = findOffset(nodeTable, nodeId);
    if (row == nullptr) {
      continue;
    }
    in.seekg(static_cast<std::streamoff>(row->offset));
    Node node;
    int32_t kind = 0;
    if (!in.read(reinterpret_cast<char*>(&node.id), 8) ||
        !in.read(reinterpret_cast<char*>(&node.lat), 8) ||
        !in.read(reinterpret_cast<char*>(&node.lon), 8) ||
        !in.read(reinterpret_cast<char*>(&kind), 4)) {
      return fail("truncated node at offset");
    }
    node.kind = static_cast<NodeKind>(kind);
    graph.addNodeBulk(std::move(node));
  }

  for (const Edge& edge : edges) {
    graph.addEdgeBulk(edge);
  }

  if (graph.nodes().empty()) {
    return fail("subset produced empty graph");
  }
  return true;
}

}  // namespace mmlp
