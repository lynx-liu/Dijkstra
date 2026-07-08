#include "mmlp/full_ch_graph.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <queue>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <unordered_map>
#include <utility>

namespace mmlp {

namespace {

constexpr char kMagic[8] = {'M', 'M', 'L', 'P', 'C', 'H', '0', '3'};
constexpr std::size_t kHeader = 32;  // magic(8)+ver(4)+nodeCount(8)+arcCount(8)+profileKmh(4)
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr uint32_t kNoChild = 0xFFFFFFFFu;

#pragma pack(push, 1)
struct FullChArcRec {
  uint32_t lo = 0;  // lower-rank endpoint (owner row in upRow)
  uint32_t hi = 0;
  uint32_t childA = kNoChild;  // shortcut: file arc via<->lo
  uint32_t childB = kNoChild;  // shortcut: file arc via<->hi
  int64_t edgeId = 0;          // 0 for shortcuts
  float weightSec = 0.0f;
  float lengthM = 0.0f;
  float speedLimit = 0.0f;
};
#pragma pack(pop)
static_assert(sizeof(FullChArcRec) == 36, "FullChArcRec layout");

struct Label {
  double dist = kInf;
  uint32_t parent = kNoChild;   // node index
  uint32_t viaArc = kNoChild;   // file arc used to reach this node
  bool settled = false;
};

}  // namespace

FullChGraph::~FullChGraph() {
  if (data_ != nullptr && size_ > 0) {
    ::munmap(data_, size_);
  }
  data_ = nullptr;
}

bool FullChGraph::open(const std::string& path, std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error != nullptr) {
      *error = msg;
    }
    return false;
  };

  if (data_ != nullptr) {
    ::munmap(data_, size_);
    data_ = nullptr;
  }

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
  if (size_ < kHeader || std::memcmp(data_, kMagic, 8) != 0) {
    return fail("invalid full ch header: " + path);
  }
  const char* base = static_cast<const char*>(data_);
  uint32_t version = 0;
  uint64_t ncnt = 0;
  uint64_t acnt = 0;
  std::memcpy(&version, base + 8, 4);
  std::memcpy(&ncnt, base + 12, 8);
  std::memcpy(&acnt, base + 20, 8);
  std::memcpy(&profileKmh_, base + 28, 4);
  if (version != 1 || ncnt == 0) {
    return fail("unsupported full ch: " + path);
  }
  nodeCount_ = static_cast<std::size_t>(ncnt);
  arcCount_ = static_cast<std::size_t>(acnt);
  const std::size_t idsBytes = nodeCount_ * sizeof(int64_t);
  const std::size_t ranksBytes = nodeCount_ * sizeof(uint32_t);
  const std::size_t rowBytes = (nodeCount_ + 1) * sizeof(uint64_t);
  const std::size_t arcBytes = arcCount_ * sizeof(FullChArcRec);
  if (size_ < kHeader + idsBytes + ranksBytes + rowBytes + arcBytes) {
    return fail("truncated full ch: " + path);
  }
  std::size_t off = kHeader;
  nodeIds_ = reinterpret_cast<const int64_t*>(base + off);
  off += idsBytes;
  ranks_ = reinterpret_cast<const uint32_t*>(base + off);
  off += ranksBytes;
  upRow_ = reinterpret_cast<const uint64_t*>(base + off);
  off += rowBytes;
  arcs_ = base + off;
  return true;
}

int FullChGraph::nodeIndex(int64_t nodeId) const {
  if (!isOpen() || nodeCount_ == 0) {
    return -1;
  }
  const int64_t* it = std::lower_bound(nodeIds_, nodeIds_ + nodeCount_, nodeId);
  if (it == nodeIds_ + nodeCount_ || *it != nodeId) {
    return -1;
  }
  return static_cast<int>(it - nodeIds_);
}

FullChGraph::PathResult FullChGraph::route(const std::vector<Seed>& from,
                                           const std::vector<Seed>& to, double maxSec,
                                           std::size_t settleCap) const {
  PathResult result;
  if (!isOpen() || from.empty() || to.empty() || maxSec <= 0.0) {
    return result;
  }
  const auto tEnter = std::chrono::steady_clock::now();

  const auto* arcTable = static_cast<const FullChArcRec*>(arcs_);

  std::unordered_map<uint32_t, Label> distF;
  std::unordered_map<uint32_t, Label> distB;
  distF.reserve(4096);
  distB.reserve(4096);

  using QItem = std::pair<double, uint32_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> qf;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> qb;

  for (const Seed& s : from) {
    const int idx = nodeIndex(s.nodeId);
    if (idx < 0 || s.costSec > maxSec) {
      continue;
    }
    Label& l = distF[static_cast<uint32_t>(idx)];
    if (s.costSec < l.dist) {
      l.dist = s.costSec;
      qf.push({s.costSec, static_cast<uint32_t>(idx)});
    }
  }
  for (const Seed& s : to) {
    const int idx = nodeIndex(s.nodeId);
    if (idx < 0 || s.costSec > maxSec) {
      continue;
    }
    Label& l = distB[static_cast<uint32_t>(idx)];
    if (s.costSec < l.dist) {
      l.dist = s.costSec;
      qb.push({s.costSec, static_cast<uint32_t>(idx)});
    }
  }
  if (qf.empty() || qb.empty()) {
    return result;
  }

  double best = kInf;
  uint32_t meet = kNoChild;
  std::size_t settled = 0;
  std::size_t pops = 0;
  std::size_t pushes = 0;

  auto relaxUp = [&](uint32_t u, double du, std::unordered_map<uint32_t, Label>& dist,
                     std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>>& pq) {
    const uint64_t begin = upRow_[u];
    const uint64_t end = upRow_[u + 1];
    for (uint64_t i = begin; i < end; ++i) {
      const FullChArcRec& arc = arcTable[i];
      const uint32_t v = arc.hi;
      const double nd = du + static_cast<double>(arc.weightSec);
      if (nd > maxSec || nd >= best) {
        continue;
      }
      Label& lv = dist[v];
      if (nd < lv.dist - 1e-9) {
        lv.dist = nd;
        lv.parent = u;
        lv.viaArc = static_cast<uint32_t>(i);
        pq.push({nd, v});
        ++pushes;
      }
    }
  };

  auto settleOne = [&](std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>>& pq,
                       std::unordered_map<uint32_t, Label>& mine,
                       std::unordered_map<uint32_t, Label>& other) {
    const auto [du, u] = pq.top();
    pq.pop();
    ++pops;
    Label& lu = mine[u];
    if (lu.settled || du > lu.dist + 1e-9) {
      return;
    }
    lu.settled = true;
    ++settled;
    const auto it = other.find(u);
    if (it != other.end()) {
      const double total = lu.dist + it->second.dist;
      if (total < best) {
        best = total;
        meet = u;
      }
    }
    relaxUp(u, lu.dist, mine, pq);
  };

  while (!qf.empty() || !qb.empty()) {
    const bool fDone = qf.empty() || qf.top().first >= best;
    const bool bDone = qb.empty() || qb.top().first >= best;
    if (fDone && bDone) {
      break;
    }
    if (settled >= settleCap) {
      // Pathological up-closure (rare with heuristic contraction orders). If a
      // meet exists, return the best-so-far path (near-optimal in practice);
      // otherwise report capped so the caller can fall back instead of
      // wrongly concluding "unreachable".
      if (meet == kNoChild) {
        result.capped = true;
        result.settledNodes = settled;
        return result;
      }
      break;
    }
    if (!qf.empty() && (qb.empty() || qf.top().first <= qb.top().first)) {
      settleOne(qf, distF, distB);
    } else {
      settleOne(qb, distB, distF);
    }
  }

  result.settledNodes = settled;
  if (meet == kNoChild || best > maxSec) {
    return result;
  }
  const auto tSearchDone = std::chrono::steady_clock::now();

  // Packed arc chains (file indices) from each side of the meet node.
  std::vector<uint32_t> fwdArcs;
  uint32_t cur = meet;
  while (true) {
    const Label& l = distF.at(cur);
    if (l.viaArc == kNoChild) {
      break;
    }
    fwdArcs.push_back(l.viaArc);
    cur = l.parent;
  }
  const uint32_t startIdx = cur;
  std::reverse(fwdArcs.begin(), fwdArcs.end());

  std::vector<uint32_t> bwdArcs;
  cur = meet;
  while (true) {
    const Label& l = distB.at(cur);
    if (l.viaArc == kNoChild) {
      break;
    }
    bwdArcs.push_back(l.viaArc);
    cur = l.parent;
  }
  const uint32_t endIdx = cur;

  // Unpack shortcuts to original edges, oriented start -> end. Explicit stack:
  // deep chains of contracted degree-2 nodes would overflow recursion.
  const auto expand = [&](uint32_t arcIdx, uint32_t enterNode) {
    std::vector<std::pair<uint32_t, uint32_t>> stack;
    stack.push_back({arcIdx, enterNode});
    while (!stack.empty()) {
      const auto [ai, enter] = stack.back();
      stack.pop_back();
      const FullChArcRec& arc = arcTable[ai];
      if (arc.childA == kNoChild) {
        PathArc pa;
        pa.edgeId = arc.edgeId;
        pa.lengthM = arc.lengthM;
        pa.speedLimitKmh = arc.speedLimit;
        const uint32_t exit = (enter == arc.lo) ? arc.hi : arc.lo;
        pa.fromNodeId = nodeIds_[enter];
        pa.toNodeId = nodeIds_[exit];
        result.arcs.push_back(pa);
        continue;
      }
      const FullChArcRec& childLoArc = arcTable[arc.childA];
      const uint32_t via = (childLoArc.lo == arc.lo || childLoArc.hi == arc.lo)
                               ? ((childLoArc.lo == arc.lo) ? childLoArc.hi : childLoArc.lo)
                               : childLoArc.lo;
      // LIFO: push the second half first so the first half is expanded first.
      if (enter == arc.lo) {
        stack.push_back({arc.childB, via});
        stack.push_back({arc.childA, arc.lo});
      } else {
        stack.push_back({arc.childA, via});
        stack.push_back({arc.childB, arc.hi});
      }
    }
  };

  uint32_t walk = startIdx;
  for (uint32_t ai : fwdArcs) {
    expand(ai, walk);
    const FullChArcRec& arc = arcTable[ai];
    walk = (walk == arc.lo) ? arc.hi : arc.lo;
  }
  // Backward chain runs meet -> end; expand each arc entering from the meet side.
  walk = meet;
  for (uint32_t ai : bwdArcs) {
    expand(ai, walk);
    const FullChArcRec& arc = arcTable[ai];
    walk = (walk == arc.lo) ? arc.hi : arc.lo;
  }

  result.found = true;
  result.profileSec = best;
  result.startNodeId = nodeIds_[startIdx];
  result.endNodeId = nodeIds_[endIdx];
  if (std::getenv("MMLP_FULLCH_DEBUG") != nullptr) {
    const auto tEnd = std::chrono::steady_clock::now();
    std::cerr << "[fullch] settled=" << settled << " pops=" << pops << " pushes=" << pushes
              << " packed=" << (fwdArcs.size() + bwdArcs.size())
              << " arcs=" << result.arcs.size() << " search_us="
              << std::chrono::duration_cast<std::chrono::microseconds>(tSearchDone - tEnter)
                     .count()
              << " unpack_us="
              << std::chrono::duration_cast<std::chrono::microseconds>(tEnd - tSearchDone).count()
              << "\n";
  }
  return result;
}

}  // namespace mmlp
