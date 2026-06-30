#include "mmlp/ch_graph.hpp"
#include "mmlp/geo.hpp"
#include "mmlp/motion.hpp"
#include "mmlp/routing.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
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
  return true;
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

  const int fromIdx = nodeIndex(fromNodeId);
  const int toIdx = nodeIndex(toNodeId);
  if (fromIdx < 0 || toIdx < 0) {
    return result;
  }
  if (fromIdx == toIdx) {
    result.travelTimeSec = 0.0;
    return result;
  }

  const auto weightOf = [&](int64_t edgeId, float lengthM) -> double {
  (void)edgeId;
    return arcWeightSec(lengthM, profileKmh_, type, param);
  };

  using QItem = std::tuple<double, int>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pqUp;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pqDown;

  std::vector<double> distUp(nodeCount_, kInf);
  std::vector<double> distDown(nodeCount_, kInf);
  std::vector<int> parentUp(nodeCount_, -1);
  std::vector<int64_t> parentEdgeUp(nodeCount_, 0);
  std::vector<int> parentDown(nodeCount_, -1);
  std::vector<int64_t> parentEdgeDown(nodeCount_, 0);

  distUp[static_cast<std::size_t>(fromIdx)] = 0.0;
  pqUp.push({0.0, fromIdx});
  distDown[static_cast<std::size_t>(toIdx)] = 0.0;
  pqDown.push({0.0, toIdx});

  double best = kInf;
  int meet = -1;

  while (!pqUp.empty() || !pqDown.empty()) {
  if (!pqUp.empty() &&
        (pqDown.empty() || std::get<0>(pqUp.top()) <= std::get<0>(pqDown.top()))) {
      const auto [du, u] = pqUp.top();
      pqUp.pop();
      if (du > distUp[static_cast<std::size_t>(u)] + 1e-9) {
        continue;
      }
      if (distDown[static_cast<std::size_t>(u)] < kInf / 2.0) {
        const double total = du + distDown[static_cast<std::size_t>(u)];
        if (total < best) {
          best = total;
          meet = u;
        }
      }
      if (du > best + 1e-9 || du > maxTime + 1e-9) {
        continue;
      }

      const uint64_t begin = upRow_[static_cast<std::size_t>(u)];
      const uint64_t end = upRow_[static_cast<std::size_t>(u) + 1];
      for (uint64_t i = begin; i < end; ++i) {
        const auto* arc = reinterpret_cast<const ChUpArcRec*>(upArcs_) + i;
        const int v = nodeIndex(arc->toNodeId);
        if (v < 0) {
          continue;
        }
        const double w = weightOf(arc->edgeId, arc->length);
        const double nd = du + w;
        if (nd < distUp[static_cast<std::size_t>(v)] - 1e-9) {
          distUp[static_cast<std::size_t>(v)] = nd;
          parentUp[static_cast<std::size_t>(v)] = u;
          parentEdgeUp[static_cast<std::size_t>(v)] = arc->edgeId;
          pqUp.push({nd, v});
        }
      }
    } else if (!pqDown.empty()) {
      const auto [dd, v] = pqDown.top();
      pqDown.pop();
      if (dd > distDown[static_cast<std::size_t>(v)] + 1e-9) {
        continue;
      }
      if (distUp[static_cast<std::size_t>(v)] < kInf / 2.0) {
        const double total = distUp[static_cast<std::size_t>(v)] + dd;
        if (total < best) {
          best = total;
          meet = v;
        }
      }
      if (dd > best + 1e-9 || dd > maxTime + 1e-9) {
        continue;
      }

      for (const auto& downArc : reverseDown_[static_cast<std::size_t>(v)]) {
        const int u = std::get<0>(downArc);
        const int64_t edgeId = std::get<1>(downArc);
        const float lengthM = std::get<2>(downArc);
        const double w = weightOf(edgeId, lengthM);
        const double nd = dd + w;
        if (nd < distDown[static_cast<std::size_t>(u)] - 1e-9) {
          distDown[static_cast<std::size_t>(u)] = nd;
          parentDown[static_cast<std::size_t>(u)] = v;
          parentEdgeDown[static_cast<std::size_t>(u)] = edgeId;
          pqDown.push({nd, u});
        }
      }
    }
  }

  if (meet < 0 || best >= kInf / 2.0 || best > maxTime + 1e-6) {
    return result;
  }

  std::vector<int64_t> edgePath;
  for (int cur = meet; cur != fromIdx; cur = parentUp[static_cast<std::size_t>(cur)]) {
    if (cur < 0) {
      return result;
    }
    edgePath.push_back(parentEdgeUp[static_cast<std::size_t>(cur)]);
  }
  std::reverse(edgePath.begin(), edgePath.end());
  for (int cur = meet; cur != toIdx; cur = parentDown[static_cast<std::size_t>(cur)]) {
    if (cur < 0) {
      return result;
    }
    const int64_t edgeId = parentEdgeDown[static_cast<std::size_t>(cur)];
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
