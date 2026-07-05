#include "mmlp/ch_graph.hpp"
#include "mmlp/geo.hpp"
#include "mmlp/motion.hpp"
#include "mmlp/routing.hpp"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <chrono>
#include <iostream>
#include <mutex>
#include <queue>
#include <sys/mman.h>
#include <sys/stat.h>
#include <tuple>
#include <unistd.h>
#include <vector>

namespace mmlp {

namespace {

constexpr char kMagic[8] = {'M', 'M', 'L', 'P', 'C', 'H', '0', '2'};
constexpr std::size_t kHeader = 32;  // magic(8)+ver(4)+nodeCount(8)+upArcCount(8)+profileKmh(4)

constexpr double kInf = std::numeric_limits<double>::infinity();

#pragma pack(push, 1)
struct ChUpArcRec {
  int64_t toNodeId = 0;
  int64_t edgeId = 0;
  float length = 0.0f;
};
#pragma pack(pop)
static_assert(sizeof(ChUpArcRec) == 20, "ChUpArcRec layout");

double arcWeightSec(float lengthM, float profileKmh, VehicleType type, const PredictParam& param) {
  const double speedMs = speedMsFromKmh(static_cast<double>(profileKmh));
  return travelTimeSeconds(static_cast<double>(lengthM), speedMs, type, param);
}

}  // namespace

ChGraph::~ChGraph() {
  if (data_ != nullptr && size_ > 0) {
    ::munmap(data_, size_);
  }
  data_ = nullptr;
  reverseDown_.clear();
  reverseDownBuilt_ = false;
}

bool ChGraph::open(const std::string& path, std::string* error) {
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
  reverseDown_.clear();
  reverseDownBuilt_ = false;

  const int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    return fail("cannot open: " + path);
  }
  struct stat st {};
  if (::fstat(fd, &st) != 0 || st.st_size <= 0) {
    ::close(fd);
    return fail("cannot stat: " + path);
  }
  void* ptr = ::mmap(nullptr, static_cast<std::size_t>(st.st_size), PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);
  if (ptr == MAP_FAILED) {
    return fail("mmap failed: " + path);
  }

  data_ = ptr;
  size_ = static_cast<std::size_t>(st.st_size);
  if (size_ < kHeader) {
    return fail("truncated ch");
  }
  if (std::memcmp(data_, kMagic, 8) != 0) {
    return fail("invalid ch magic");
  }

  uint32_t version = 0;
  uint64_t ncnt = 0;
  uint64_t acnt = 0;
  const char* base = static_cast<const char*>(data_);
  std::memcpy(&version, base + 8, 4);
  std::memcpy(&ncnt, base + 12, 8);
  std::memcpy(&acnt, base + 20, 8);
  std::memcpy(&profileKmh_, base + 28, 4);
  if (version != 1 || ncnt == 0) {
    return fail("unsupported or empty ch");
  }

  nodeCount_ = static_cast<std::size_t>(ncnt);
  upArcCount_ = static_cast<std::size_t>(acnt);
  const std::size_t idsBytes = nodeCount_ * sizeof(int64_t);
  const std::size_t ranksBytes = nodeCount_ * sizeof(uint32_t);
  const std::size_t rowBytes = (nodeCount_ + 1) * sizeof(uint64_t);
  const std::size_t arcBytes = upArcCount_ * sizeof(ChUpArcRec);
  const std::size_t expected = kHeader + idsBytes + ranksBytes + rowBytes + arcBytes;
  if (size_ < expected) {
    return fail("truncated ch body");
  }

  std::size_t off = kHeader;
  nodeIds_ = reinterpret_cast<const int64_t*>(base + off);
  off += idsBytes;
  ranks_ = reinterpret_cast<const uint32_t*>(base + off);
  off += ranksBytes;
  upRow_ = reinterpret_cast<const uint64_t*>(base + off);
  off += rowBytes;
  upArcs_ = base + off;
  return true;
}

void ChGraph::ensureReverseDown() const {
  if (reverseDownBuilt_ || !isOpen()) {
    return;
  }
  std::lock_guard<std::mutex> lock(reverseDownMu_);
  if (reverseDownBuilt_) {
    return;
  }
  reverseDown_.assign(nodeCount_, {});
  for (std::size_t u = 0; u < nodeCount_; ++u) {
    const uint64_t begin = upRow_[u];
    const uint64_t end = upRow_[u + 1];
    for (uint64_t i = begin; i < end; ++i) {
      const auto* arc = reinterpret_cast<const ChUpArcRec*>(upArcs_) + i;
      const int vIdx = nodeIndex(arc->toNodeId);
      if (vIdx < 0) {
        continue;
      }
      reverseDown_[static_cast<std::size_t>(vIdx)].push_back(
          {static_cast<int>(u), arc->edgeId, arc->length});
    }
  }
  reverseDownBuilt_ = true;
  std::cerr << "[mmlp] ch reverse-down ready nodes=" << nodeCount_ << " arcs=" << upArcCount_
            << "\n"
            << std::flush;
}

int ChGraph::nodeIndex(int64_t nodeId) const {
  if (!isOpen() || nodeCount_ == 0) {
    return -1;
  }
  const int64_t* it =
      std::lower_bound(nodeIds_, nodeIds_ + nodeCount_, nodeId);
  if (it == nodeIds_ + nodeCount_ || *it != nodeId) {
    return -1;
  }
  return static_cast<int>(it - nodeIds_);
}

RouteToGoal ChGraph::query(const GraphFileStore& store, const CsrGraph& hwyCsr,
                           int64_t fromNodeId, int64_t toNodeId, VehicleType type,
                           const PredictParam& param, double maxTime) const {
  RouteToGoal result;
  (void)store;
  (void)hwyCsr;
  if (!isOpen() || maxTime <= 0.0) {
    return result;
  }

  ensureReverseDown();

  const int fromIdx = nodeIndex(fromNodeId);
  const int toIdx = nodeIndex(toNodeId);
  if (fromIdx < 0 || toIdx < 0) {
    return result;
  }
  if (fromIdx == toIdx) {
    result.travelTimeSec = 0.0;
    return result;
  }

  const uint32_t rankFrom = ranks_[static_cast<std::size_t>(fromIdx)];
  const uint32_t rankTo = ranks_[static_cast<std::size_t>(toIdx)];

  const auto weightOf = [&](int64_t edgeId, float lengthM) -> double {
    (void)edgeId;
    return arcWeightSec(lengthM, profileKmh_, type, param);
  };

  using QItem = std::tuple<double, int>;
  std::vector<double> distFwd(nodeCount_, kInf);
  std::vector<double> distBwd(nodeCount_, kInf);
  std::vector<int> parentFwd(nodeCount_, -1);
  std::vector<int64_t> parentEdgeFwd(nodeCount_, 0);
  std::vector<int> parentBwd(nodeCount_, -1);
  std::vector<int64_t> parentEdgeBwd(nodeCount_, 0);

  const bool fwdUp = rankFrom <= rankTo;
  const bool bwdUp = rankTo <= rankFrom;

  auto runSearch = [&](int start, bool upward, std::vector<double>& dist, std::vector<int>& parent,
                       std::vector<int64_t>& parentEdge) {
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    dist[static_cast<std::size_t>(start)] = 0.0;
    pq.push({0.0, start});
    while (!pq.empty()) {
      const auto [du, u] = pq.top();
      pq.pop();
      if (du > dist[static_cast<std::size_t>(u)] + 1e-9 || du > maxTime + 1e-9) {
        continue;
      }
      if (upward) {
        const uint64_t begin = upRow_[static_cast<std::size_t>(u)];
        const uint64_t end = upRow_[static_cast<std::size_t>(u) + 1];
        for (uint64_t i = begin; i < end; ++i) {
          const auto* arc = reinterpret_cast<const ChUpArcRec*>(upArcs_) + i;
          const int v = nodeIndex(arc->toNodeId);
          if (v < 0) {
            continue;
          }
          const double nd = du + weightOf(arc->edgeId, arc->length);
          if (nd < dist[static_cast<std::size_t>(v)] - 1e-9) {
            dist[static_cast<std::size_t>(v)] = nd;
            parent[static_cast<std::size_t>(v)] = u;
            parentEdge[static_cast<std::size_t>(v)] = arc->edgeId;
            pq.push({nd, v});
          }
        }
      } else {
        for (const auto& downArc : reverseDown_[static_cast<std::size_t>(u)]) {
          const int v = std::get<0>(downArc);
          const int64_t edgeId = std::get<1>(downArc);
          const float lengthM = std::get<2>(downArc);
          const double nd = du + weightOf(edgeId, lengthM);
          if (nd < dist[static_cast<std::size_t>(v)] - 1e-9) {
            dist[static_cast<std::size_t>(v)] = nd;
            parent[static_cast<std::size_t>(v)] = u;
            parentEdge[static_cast<std::size_t>(v)] = edgeId;
            pq.push({nd, v});
          }
        }
      }
    }
  };

  runSearch(fromIdx, fwdUp, distFwd, parentFwd, parentEdgeFwd);
  runSearch(toIdx, bwdUp, distBwd, parentBwd, parentEdgeBwd);

  double best = kInf;
  int meet = -1;
  for (std::size_t u = 0; u < nodeCount_; ++u) {
    if (distFwd[u] < kInf / 2.0 && distBwd[u] < kInf / 2.0) {
      const double total = distFwd[u] + distBwd[u];
      if (total < best) {
        best = total;
        meet = static_cast<int>(u);
      }
    }
  }

  if (meet < 0 || best >= kInf / 2.0 || best > maxTime + 1e-6) {
    if (!hwyCsr.isOpen()) {
      return result;
    }
    const int fromRow = hwyCsr.nodeRow(store, fromNodeId);
    const int toRow = hwyCsr.nodeRow(store, toNodeId);
    if (fromRow < 0 || toRow < 0) {
      return result;
    }
    using QItem = std::tuple<double, int64_t>;
    std::vector<double> dist(hwyCsr.nodeCount(), kInf);
    std::vector<int64_t> parent(hwyCsr.nodeCount(), 0);
    std::vector<int64_t> parentEdge(hwyCsr.nodeCount(), 0);
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    dist[static_cast<std::size_t>(fromRow)] = 0.0;
    pq.push({0.0, fromNodeId});
    while (!pq.empty()) {
      const auto [du, u] = pq.top();
      pq.pop();
      const int uRow = hwyCsr.nodeRow(store, u);
      if (uRow < 0 || du > dist[static_cast<std::size_t>(uRow)] + 1e-9 ||
          du > maxTime + 1e-9) {
        continue;
      }
      if (u == toNodeId) {
        result.travelTimeSec = du;
        std::vector<int64_t> edgePath;
        for (int64_t cur = u; cur != fromNodeId;) {
          const int curRow = hwyCsr.nodeRow(store, cur);
          if (curRow < 0) {
            break;
          }
          const int64_t edgeId = parentEdge[static_cast<std::size_t>(curRow)];
          if (edgeId != 0) {
            edgePath.push_back(edgeId);
          }
          const int64_t prev = parent[static_cast<std::size_t>(curRow)];
          if (prev == 0 || prev == cur) {
            break;
          }
          cur = prev;
        }
        std::reverse(edgePath.begin(), edgePath.end());
        RoutePolyline& route = result.polyline;
        route.points.reserve(edgePath.size() + 2);
        double flat = 0.0;
        double flon = 0.0;
        double tlat = 0.0;
        double tlon = 0.0;
        if (store.nodeLatLon(fromNodeId, flat, flon)) {
          route.points.push_back({flat, flon});
        }
        for (int64_t edgeId : edgePath) {
          if (!store.edgeEndpointLatLon(edgeId, flat, flon, tlat, tlon)) {
            continue;
          }
          if (route.points.empty() || haversineMeters(route.points.back(), {flat, flon}) > 1.0) {
            route.points.push_back({flat, flon});
          }
          route.points.push_back({tlat, tlon});
        }
        if (store.nodeLatLon(toNodeId, flat, flon)) {
          if (route.points.empty() || haversineMeters(route.points.back(), {flat, flon}) > 1.0) {
            route.points.push_back({flat, flon});
          }
        }
        if (std::getenv("MMLP_DEBUG_CH")) {
          std::cerr << "[mmlp] ch csr_fallback travel=" << du << " pts=" << route.points.size()
                    << "\n"
                    << std::flush;
        }
        return result;
      }
      hwyCsr.forEachNeighbor(store, u, nullptr, [&](const CsrArc& arc) {
        const int vRow = hwyCsr.nodeRow(store, arc.toNodeId);
        if (vRow < 0) {
          return;
        }
        const double w = arcWeightSec(arc.length, profileKmh_, type, param);
        const double nd = du + w;
        if (nd < dist[static_cast<std::size_t>(vRow)] - 1e-9) {
          dist[static_cast<std::size_t>(vRow)] = nd;
          parent[static_cast<std::size_t>(vRow)] = u;
          parentEdge[static_cast<std::size_t>(vRow)] = arc.edgeId;
          pq.push({nd, arc.toNodeId});
        }
      });
    }
    return result;
  }

  std::vector<int64_t> edgePath;
  for (int cur = meet; cur != fromIdx; cur = parentFwd[static_cast<std::size_t>(cur)]) {
    if (cur < 0) {
      return result;
    }
    edgePath.push_back(parentEdgeFwd[static_cast<std::size_t>(cur)]);
  }
  std::reverse(edgePath.begin(), edgePath.end());
  for (int cur = meet; cur != toIdx; cur = parentBwd[static_cast<std::size_t>(cur)]) {
    if (cur < 0) {
      return result;
    }
    const int64_t edgeId = parentEdgeBwd[static_cast<std::size_t>(cur)];
    if (edgeId != 0) {
      edgePath.push_back(edgeId);
    }
  }

  RoutePolyline& route = result.polyline;
  route.points.reserve(edgePath.size() + 2);
  double flat = 0.0;
  double flon = 0.0;
  double tlat = 0.0;
  double tlon = 0.0;
  if (store.nodeLatLon(fromNodeId, flat, flon)) {
    route.points.push_back({flat, flon});
  }
  for (int64_t edgeId : edgePath) {
    if (!store.edgeEndpointLatLon(edgeId, flat, flon, tlat, tlon)) {
      continue;
    }
    if (route.points.empty() ||
        haversineMeters(route.points.back(), {flat, flon}) > 1.0) {
      route.points.push_back({flat, flon});
    }
    route.points.push_back({tlat, tlon});
  }
  if (store.nodeLatLon(toNodeId, flat, flon)) {
    if (route.points.empty() ||
        haversineMeters(route.points.back(), {flat, flon}) > 1.0) {
      route.points.push_back({flat, flon});
    }
  }

  result.travelTimeSec = best;
  return result;
}

}  // namespace mmlp
