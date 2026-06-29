#include "mmlp/graph_store.hpp"

#include <algorithm>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mmlp {

namespace {

constexpr char kBinMagic[] = "MMLPGRPH";
constexpr char kNdxMagic[8] = {'M', 'M', 'L', 'P', 'N', 'D', 'X', '\0'};
constexpr char kEgeoMagic[8] = {'M', 'M', 'L', 'P', 'E', 'G', 'E', 'O'};
constexpr std::size_t kNodeRecord = 28;
constexpr std::size_t kEdgeRecord = 44;
constexpr std::size_t kIndexHeader = 20;  // magic(8) + version(4) + count(8)
constexpr std::size_t kEdgeGeoRecord = 16;  // 4 x float32

}  // namespace

GraphFileStore::~GraphFileStore() {
  bin_.unmap();
  nidx_.unmap();
  eidx_.unmap();
  egeo_.unmap();
  edgeGeo_ = nullptr;
  edgeGeoCount_ = 0;
}

bool GraphFileStore::MmapFile::map(const std::string& path, std::string* error) {
  unmap();
  const int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    if (error) {
      *error = "cannot open: " + path;
    }
    return false;
  }
  struct stat st {};
  if (::fstat(fd, &st) != 0 || st.st_size <= 0) {
    ::close(fd);
    if (error) {
      *error = "cannot stat: " + path;
    }
    return false;
  }
  void* ptr = ::mmap(nullptr, static_cast<std::size_t>(st.st_size), PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);
  if (ptr == MAP_FAILED) {
    if (error) {
      *error = "mmap failed: " + path;
    }
    return false;
  }
  data = ptr;
  size = static_cast<std::size_t>(st.st_size);
  return true;
}

void GraphFileStore::MmapFile::unmap() {
  if (data != nullptr && size > 0) {
    ::munmap(data, size);
  }
  data = nullptr;
  size = 0;
}

const GraphFileStore::IdOffset* GraphFileStore::findOffset(const IdOffset* table,
                                                           std::size_t count, int64_t id) {
  if (table == nullptr || count == 0) {
    return nullptr;
  }
  auto it = std::lower_bound(table, table + count, id,
                             [](const IdOffset& row, int64_t key) { return row.id < key; });
  if (it == table + count || it->id != id) {
    return nullptr;
  }
  return it;
}

bool GraphFileStore::open(const std::string& binPath, std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  const auto dot = binPath.rfind('.');
  const std::string base = (dot == std::string::npos) ? binPath : binPath.substr(0, dot);

  if (!bin_.map(binPath, error) || !nidx_.map(base + ".nidx", error) ||
      !eidx_.map(base + ".eidx", error)) {
    return false;
  }

  if (nidx_.size < kIndexHeader || eidx_.size < kIndexHeader) {
    return fail("truncated index file");
  }
  if (std::memcmp(nidx_.data, kNdxMagic, 8) != 0 || std::memcmp(eidx_.data, kNdxMagic, 8) != 0) {
    return fail("invalid index magic");
  }

  uint32_t nver = 0;
  uint64_t ncnt = 0;
  std::memcpy(&nver, static_cast<const char*>(nidx_.data) + 8, 4);
  std::memcpy(&ncnt, static_cast<const char*>(nidx_.data) + 12, 8);
  if (nver != 1) {
    return fail("unsupported nidx version");
  }
  nodeCount_ = static_cast<std::size_t>(ncnt);
  nodeIndex_ = reinterpret_cast<const IdOffset*>(static_cast<const char*>(nidx_.data) + kIndexHeader);
  if (nidx_.size < kIndexHeader + nodeCount_ * sizeof(IdOffset)) {
    return fail("truncated nidx records");
  }

  uint32_t ever = 0;
  uint64_t ecnt = 0;
  std::memcpy(&ever, static_cast<const char*>(eidx_.data) + 8, 4);
  std::memcpy(&ecnt, static_cast<const char*>(eidx_.data) + 12, 8);
  if (ever != 1) {
    return fail("unsupported eidx version");
  }
  edgeCount_ = static_cast<std::size_t>(ecnt);
  edgeIndex_ = reinterpret_cast<const IdOffset*>(static_cast<const char*>(eidx_.data) + kIndexHeader);
  if (eidx_.size < kIndexHeader + edgeCount_ * sizeof(IdOffset)) {
    return fail("truncated eidx records");
  }

  if (bin_.size < 28) {
    return fail("truncated graph");
  }
  if (std::memcmp(bin_.data, kBinMagic, 8) != 0) {
    return fail("invalid graph magic");
  }
  uint32_t version = 0;
  uint64_t nodeCount = 0;
  uint64_t edgeCount = 0;
  std::memcpy(&version, static_cast<const char*>(bin_.data) + 8, 4);
  std::memcpy(&nodeCount, static_cast<const char*>(bin_.data) + 12, 8);
  std::memcpy(&edgeCount, static_cast<const char*>(bin_.data) + 20, 8);
  if (version != 1) {
    return fail("unsupported graph version");
  }
  edgeRegionOffset_ = 28 + nodeCount * kNodeRecord;
  const uint64_t expectedSize = edgeRegionOffset_ + edgeCount * kEdgeRecord;
  if (bin_.size < expectedSize) {
    return fail("graph file shorter than header claims");
  }

  const std::string egeoPath = base + ".egeo";
  if (egeo_.map(egeoPath, nullptr)) {
    if (egeo_.size >= 20 && std::memcmp(egeo_.data, kEgeoMagic, 8) == 0) {
      uint32_t gver = 0;
      uint64_t gcnt = 0;
      std::memcpy(&gver, static_cast<const char*>(egeo_.data) + 8, 4);
      std::memcpy(&gcnt, static_cast<const char*>(egeo_.data) + 12, 8);
      if (gver == 1 && gcnt == edgeCount &&
          egeo_.size >= 20 + static_cast<std::size_t>(gcnt) * kEdgeGeoRecord) {
        edgeGeo_ = reinterpret_cast<const float*>(static_cast<const char*>(egeo_.data) + 20);
        edgeGeoCount_ = static_cast<std::size_t>(gcnt);
      }
    }
    if (edgeGeo_ == nullptr) {
      egeo_.unmap();
    }
  }

  const std::string csrPath = base + ".csr";
  (void)csrPath;
  // CSR mmap (~5GB) is optional; do not load at startup (reserved for future CH).

  binPath_ = binPath;
  return true;
}

int GraphFileStore::nodeRowIndex(int64_t nodeId) const {
  const IdOffset* row = findOffset(nodeIndex_, nodeCount_, nodeId);
  if (row == nullptr) {
    return -1;
  }
  return static_cast<int>(row - nodeIndex_);
}

bool GraphFileStore::readEdge(int64_t edgeId, EdgeType& type, double& length,
                              double& speedLimit) const {
  const IdOffset* row = findOffset(edgeIndex_, edgeCount_, edgeId);
  if (row == nullptr || row->offset + kEdgeRecord > bin_.size) {
    return false;
  }
  const char* p = static_cast<const char*>(bin_.data) + row->offset;
  int32_t t = 0;
  int64_t id = 0;
  std::memcpy(&id, p, 8);
  (void)id;
  std::memcpy(&t, p + 24, 4);
  std::memcpy(&length, p + 28, 8);
  std::memcpy(&speedLimit, p + 36, 8);
  type = static_cast<EdgeType>(t);
  return true;
}

bool GraphFileStore::nodeLatLon(int64_t nodeId, double& lat, double& lon) const {
  const IdOffset* row = findOffset(nodeIndex_, nodeCount_, nodeId);
  if (row == nullptr || row->offset + kNodeRecord > bin_.size) {
    return false;
  }
  const char* p = static_cast<const char*>(bin_.data) + row->offset;
  int64_t id = 0;
  std::memcpy(&id, p, 8);
  if (id != nodeId) {
    return false;
  }
  std::memcpy(&lat, p + 8, 8);
  std::memcpy(&lon, p + 16, 8);
  return true;
}

bool GraphFileStore::edgeEndpoints(int64_t edgeId, int64_t& from, int64_t& to) const {
  const IdOffset* row = findOffset(edgeIndex_, edgeCount_, edgeId);
  if (row == nullptr || row->offset + 24 > bin_.size) {
    return false;
  }
  const char* p = static_cast<const char*>(bin_.data) + row->offset + 8;
  std::memcpy(&from, p, 8);
  std::memcpy(&to, p + 8, 8);
  return true;
}

bool GraphFileStore::edgeEndpointLatLon(int64_t edgeId, double& flat, double& flon, double& tlat,
                                        double& tlon) const {
  const IdOffset* row = findOffset(edgeIndex_, edgeCount_, edgeId);
  if (row == nullptr) {
    return false;
  }
  if (edgeGeo_ != nullptr) {
    const uint64_t idx = (row->offset - edgeRegionOffset_) / kEdgeRecord;
    if (idx < edgeGeoCount_) {
      const float* geo = edgeGeo_ + idx * 4;
      flat = geo[0];
      flon = geo[1];
      tlat = geo[2];
      tlon = geo[3];
      if (flat == flat && tlat == tlat) {  // not NaN
        return true;
      }
    }
  }
  int64_t from = 0;
  int64_t to = 0;
  if (!edgeEndpoints(edgeId, from, to)) {
    return false;
  }
  return nodeLatLon(from, flat, flon) && nodeLatLon(to, tlat, tlon);
}

bool GraphFileStore::loadGraphSubset(const std::unordered_set<int64_t>& edgeIds,
                                    MultimodalGraph& graph, std::string* error) const {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };
  if (!isOpen()) {
    return fail("graph store not open");
  }
  if (edgeIds.empty()) {
    return fail("empty edge set");
  }

  struct EdgeAt {
    Edge edge;
  };
  std::vector<EdgeAt> edges;
  edges.reserve(edgeIds.size());

  for (int64_t edgeId : edgeIds) {
    const IdOffset* row = findOffset(edgeIndex_, edgeCount_, edgeId);
    if (row == nullptr || row->offset + kEdgeRecord > bin_.size) {
      continue;
    }
    const char* p = static_cast<const char*>(bin_.data) + row->offset;
    EdgeAt item;
    int32_t type = 0;
    std::memcpy(&item.edge.id, p, 8);
    std::memcpy(&item.edge.from, p + 8, 8);
    std::memcpy(&item.edge.to, p + 16, 8);
    std::memcpy(&type, p + 24, 4);
    std::memcpy(&item.edge.length, p + 28, 8);
    std::memcpy(&item.edge.speedLimit, p + 36, 8);
    item.edge.type = static_cast<EdgeType>(type);
    edges.push_back(item);
  }
  if (edges.empty()) {
    return fail("no edges resolved");
  }

  std::unordered_set<int64_t> nodeIds;
  nodeIds.reserve(edges.size() * 2);
  for (const auto& item : edges) {
    nodeIds.insert(item.edge.from);
    nodeIds.insert(item.edge.to);
  }

  std::vector<Node> nodes;
  nodes.reserve(nodeIds.size());
  for (int64_t nodeId : nodeIds) {
    const IdOffset* row = findOffset(nodeIndex_, nodeCount_, nodeId);
    if (row == nullptr || row->offset + kNodeRecord > bin_.size) {
      continue;
    }
    const char* p = static_cast<const char*>(bin_.data) + row->offset;
    Node node;
    int32_t kind = 0;
    std::memcpy(&node.id, p, 8);
    std::memcpy(&node.lat, p + 8, 8);
    std::memcpy(&node.lon, p + 16, 8);
    std::memcpy(&kind, p + 24, 4);
    node.kind = static_cast<NodeKind>(kind);
    nodes.push_back(node);
  }

  std::vector<Edge> edgeList;
  edgeList.reserve(edges.size());
  for (const auto& item : edges) {
    edgeList.push_back(item.edge);
  }

  graph.buildFromSubset(std::move(nodes), std::move(edgeList));
  if (graph.nodes().empty()) {
    return fail("subset empty");
  }
  return true;
}

}  // namespace mmlp
