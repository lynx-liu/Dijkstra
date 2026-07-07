#include "mmlp/csr_graph.hpp"

#include "mmlp/graph_store.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mmlp {

namespace {

constexpr char kCsrMagic[8] = {'M', 'M', 'L', 'P', 'C', 'S', 'R', '\0'};
constexpr std::size_t kCsrHeader = 28;  // magic(8) + ver(4) + nodeCount(8) + arcCount(8)

#pragma pack(push, 1)
struct CsrArcRecord {
  int64_t toNodeId = 0;
  int64_t edgeId = 0;
  int32_t edgeType = 0;
  float length = 0.0f;
  float speedLimit = 0.0f;
};
#pragma pack(pop)
static_assert(sizeof(CsrArcRecord) == 28, "CsrArcRecord layout");

}  // namespace

CsrGraph::~CsrGraph() {
  if (data_ != nullptr && size_ > 0) {
    ::munmap(data_, size_);
  }
  data_ = nullptr;
  size_ = 0;
  rowPtr_ = nullptr;
  arcs_ = nullptr;
}

bool CsrGraph::open(const std::string& csrPath, std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return false;
  };

  if (data_ != nullptr) {
    ::munmap(data_, size_);
    data_ = nullptr;
  }

  const int fd = ::open(csrPath.c_str(), O_RDONLY);
  if (fd < 0) {
    return fail("cannot open: " + csrPath);
  }
  struct stat st {};
  if (::fstat(fd, &st) != 0 || st.st_size <= 0) {
    ::close(fd);
    return fail("cannot stat: " + csrPath);
  }
  int flags = MAP_SHARED;
#if defined(MAP_POPULATE)
  std::size_t populateMaxMb = 256;
  if (const char* env = std::getenv("MMLP_MMAP_POPULATE_MAX_MB")) {
    const int v = std::atoi(env);
    if (v > 0) {
      populateMaxMb = static_cast<std::size_t>(v);
    }
  }
  const std::size_t populateMaxBytes = populateMaxMb * 1024 * 1024;
  if (st.st_size > 0 && static_cast<std::size_t>(st.st_size) <= populateMaxBytes) {
    flags |= MAP_POPULATE;
  }
#endif
  void* ptr = ::mmap(nullptr, static_cast<std::size_t>(st.st_size), PROT_READ, flags, fd, 0);
  ::close(fd);
  if (ptr == MAP_FAILED) {
    return fail("mmap failed: " + csrPath);
  }

  data_ = ptr;
  size_ = static_cast<std::size_t>(st.st_size);
  if (size_ < kCsrHeader) {
    return fail("truncated csr");
  }
  if (std::memcmp(data_, kCsrMagic, 8) != 0) {
    return fail("invalid csr magic");
  }

  uint32_t version = 0;
  uint64_t ncnt = 0;
  uint64_t acnt = 0;
  std::memcpy(&version, static_cast<const char*>(data_) + 8, 4);
  std::memcpy(&ncnt, static_cast<const char*>(data_) + 12, 8);
  std::memcpy(&acnt, static_cast<const char*>(data_) + 20, 8);
  if (version != 1) {
    return fail("unsupported csr version");
  }

  nodeCount_ = static_cast<std::size_t>(ncnt);
  arcCount_ = static_cast<std::size_t>(acnt);
  const std::size_t rowBytes = (nodeCount_ + 1) * sizeof(uint64_t);
  const std::size_t arcBytes = arcCount_ * sizeof(CsrArcRecord);
  const std::size_t expected = kCsrHeader + rowBytes + arcBytes;
  if (size_ < expected) {
    return fail("truncated csr body");
  }

  rowPtr_ = reinterpret_cast<const uint64_t*>(static_cast<const char*>(data_) + kCsrHeader);
  arcs_ = reinterpret_cast<const char*>(rowPtr_) + rowBytes;
  return true;
}

int CsrGraph::nodeRow(const GraphFileStore& store, int64_t nodeId) const {
  return store.nodeRowIndex(nodeId);
}

void CsrGraph::forEachNeighbor(const GraphFileStore& store, int64_t nodeId,
                               const std::unordered_set<int64_t>* allowedEdgeIds,
                               const NeighborFn& fn) const {
  if (!isOpen() || !fn) {
    return;
  }
  const int row = nodeRow(store, nodeId);
  if (row < 0) {
    return;
  }
  const std::size_t r = static_cast<std::size_t>(row);
  if (r >= nodeCount_) {
    return;
  }
  const uint64_t begin = rowPtr_[r];
  const uint64_t end = rowPtr_[r + 1];
  if (begin >= end || end > arcCount_) {
    return;
  }
  const auto* arcTable = static_cast<const CsrArcRecord*>(arcs_);
  for (uint64_t i = begin; i < end; ++i) {
    const CsrArcRecord& rec = arcTable[static_cast<std::size_t>(i)];
    if (allowedEdgeIds != nullptr && allowedEdgeIds->count(rec.edgeId) == 0) {
      continue;
    }
    CsrArc arc;
    arc.toNodeId = rec.toNodeId;
    arc.edgeId = rec.edgeId;
    arc.type = static_cast<EdgeType>(rec.edgeType);
    arc.length = rec.length;
    arc.speedLimit = rec.speedLimit;
    fn(arc);
  }
}

}  // namespace mmlp
