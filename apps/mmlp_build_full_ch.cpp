// Build full-graph Contraction Hierarchies for a regional graph:
//   input:  <region>.mmlp.csr + <region>.mmlp.nidx  (road arcs only)
//   output: <region>.mmlp.full.ch  (format MMLPCH03, shortcuts carry child arcs
//           so routes can be fully unpacked at query time)
//
// Unlike the hwy overlay CH (highway subgraph, ~1% of nodes), this contracts the
// entire road graph, so any start/destination pair resolves in a millisecond-scale
// bidirectional search instead of a multi-second full Dijkstra field build.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

constexpr char kCsrMagic[8] = {'M', 'M', 'L', 'P', 'C', 'S', 'R', '\0'};
constexpr char kOutMagic[8] = {'M', 'M', 'L', 'P', 'C', 'H', '0', '3'};
constexpr double kProfileKmh = 80.0;
constexpr double kProfileMs = kProfileKmh / 3.6;
constexpr uint32_t kNoChild = 0xFFFFFFFFu;

#pragma pack(push, 1)
struct CsrArcRec {
  int64_t toNodeId = 0;
  int64_t edgeId = 0;
  int32_t edgeType = 0;
  float length = 0.0f;
  float speedLimit = 0.0f;
};
// File arc record (36 bytes). lo = lower-rank endpoint (owner row), hi = other.
// Original arcs: edgeId != 0, length/speedLimit set, children = kNoChild.
// Shortcuts: edgeId == 0, childA = file arc (via<->lo), childB = file arc (via<->hi).
struct FullChArcRec {
  uint32_t lo = 0;
  uint32_t hi = 0;
  uint32_t childA = kNoChild;
  uint32_t childB = kNoChild;
  int64_t edgeId = 0;
  float weightSec = 0.0f;
  float lengthM = 0.0f;
  float speedLimit = 0.0f;
};
#pragma pack(pop)
static_assert(sizeof(FullChArcRec) == 36, "FullChArcRec layout");

struct BuildArc {
  int32_t a = -1;
  int32_t b = -1;
  int64_t edgeId = 0;
  float weight = 0.0f;
  float length = 0.0f;
  float speedLimit = 0.0f;
  uint32_t childToA = kNoChild;  // build id of arc via<->a (for shortcuts)
  uint32_t childToB = kNoChild;  // build id of arc via<->b
};

struct AdjEntry {
  int32_t to = -1;
  uint32_t arcId = 0;
};

float roadWeightSec(float lengthM, float speedLimitKmh) {
  double eff = kProfileMs;
  if (speedLimitKmh > 0.0f) {
    eff = std::min(eff, static_cast<double>(speedLimitKmh) / 3.6);
  }
  return static_cast<float>(static_cast<double>(lengthM) / eff);
}

class Builder {
 public:
  std::vector<int64_t> globalId;
  std::vector<std::vector<AdjEntry>> adj;
  std::vector<BuildArc> arcs;
  std::vector<uint8_t> removed;
  std::vector<int32_t> delNeighbors;

  // Witness-search scratch (epoch-stamped so no per-search clearing).
  std::vector<float> wDist;
  std::vector<uint32_t> wEpoch;
  uint32_t epoch = 0;

  int n() const { return static_cast<int>(globalId.size()); }

  void init(std::size_t nodes) {
    adj.assign(nodes, {});
    removed.assign(nodes, 0);
    delNeighbors.assign(nodes, 0);
    wDist.assign(nodes, 0.0f);
    wEpoch.assign(nodes, 0);
  }

  float arcWeightBetween(int u, int v, uint32_t* arcIdOut = nullptr) const {
    for (const AdjEntry& e : adj[static_cast<std::size_t>(u)]) {
      if (e.to == v) {
        if (arcIdOut != nullptr) {
          *arcIdOut = e.arcId;
        }
        return arcs[e.arcId].weight;
      }
    }
    return std::numeric_limits<float>::infinity();
  }

  // Add or improve arc u<->v. For shortcuts childU/childV are build arc ids
  // (via<->u, via<->v). Returns build id used.
  void addArc(int u, int v, float weight, int64_t edgeId, float length, float limit,
              uint32_t childU, uint32_t childV) {
    for (AdjEntry& e : adj[static_cast<std::size_t>(u)]) {
      if (e.to != v) {
        continue;
      }
      BuildArc& arc = arcs[e.arcId];
      if (weight < arc.weight) {
        arc.weight = weight;
        arc.edgeId = edgeId;
        arc.length = length;
        arc.speedLimit = limit;
        if (arc.a == u) {
          arc.childToA = childU;
          arc.childToB = childV;
        } else {
          arc.childToA = childV;
          arc.childToB = childU;
        }
      }
      return;
    }
    const uint32_t id = static_cast<uint32_t>(arcs.size());
    BuildArc arc;
    arc.a = u;
    arc.b = v;
    arc.edgeId = edgeId;
    arc.weight = weight;
    arc.length = length;
    arc.speedLimit = limit;
    arc.childToA = childU;
    arc.childToB = childV;
    arcs.push_back(arc);
    adj[static_cast<std::size_t>(u)].push_back({v, id});
    adj[static_cast<std::size_t>(v)].push_back({u, id});
  }

  std::vector<AdjEntry> activeNeighbors(int v) const {
    std::vector<AdjEntry> out;
    out.reserve(adj[static_cast<std::size_t>(v)].size());
    for (const AdjEntry& e : adj[static_cast<std::size_t>(v)]) {
      if (!removed[static_cast<std::size_t>(e.to)] && e.to != v) {
        out.push_back(e);
      }
    }
    return out;
  }

  // Witness search scratch reused across simulate() calls.
  std::vector<int> scTargetNode;
  std::vector<float> scTargetBound;
  std::vector<uint8_t> scFound;
  std::vector<int32_t> scTargetSlot;  // node -> slot in current target set (-1 none)
  std::vector<int> scTouchedSlots;

  // Multi-target witness Dijkstra from u avoiding v. Exact within settleCap;
  // an undersized cap only adds redundant (never wrong) shortcuts, but too many
  // redundant shortcuts blow up core degrees, so the cap must stay generous.
  void witness(int u, int v, std::size_t settleCap) {
    ++epoch;
    float maxBound = 0.0f;
    std::size_t remaining = 0;
    for (std::size_t i = 0; i < scTargetBound.size(); ++i) {
      maxBound = std::max(maxBound, scTargetBound[i]);
      if (!scFound[i]) {
        ++remaining;
      }
    }
    if (remaining == 0) {
      return;
    }
    using QItem = std::pair<float, int>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    wDist[static_cast<std::size_t>(u)] = 0.0f;
    wEpoch[static_cast<std::size_t>(u)] = epoch;
    pq.push({0.0f, u});
    std::size_t settled = 0;
    while (!pq.empty() && settled < settleCap && remaining > 0) {
      const auto [d, x] = pq.top();
      pq.pop();
      if (wEpoch[static_cast<std::size_t>(x)] == epoch &&
          d > wDist[static_cast<std::size_t>(x)] + 1e-6f) {
        continue;
      }
      if (d > maxBound + 1e-5f) {
        break;
      }
      ++settled;
      const int32_t slot = scTargetSlot[static_cast<std::size_t>(x)];
      if (slot >= 0 && !scFound[static_cast<std::size_t>(slot)] &&
          d <= scTargetBound[static_cast<std::size_t>(slot)] + 1e-5f) {
        scFound[static_cast<std::size_t>(slot)] = 1;
        --remaining;
      }
      for (const AdjEntry& e : adj[static_cast<std::size_t>(x)]) {
        if (e.to == v || removed[static_cast<std::size_t>(e.to)]) {
          continue;
        }
        const float nd = d + arcs[e.arcId].weight;
        if (nd > maxBound + 1e-5f) {
          continue;
        }
        if (wEpoch[static_cast<std::size_t>(e.to)] != epoch ||
            nd < wDist[static_cast<std::size_t>(e.to)] - 1e-6f) {
          wDist[static_cast<std::size_t>(e.to)] = nd;
          wEpoch[static_cast<std::size_t>(e.to)] = epoch;
          pq.push({nd, e.to});
        }
      }
    }
  }

  struct Shortcut {
    int u = -1;
    int w = -1;
    float weight = 0.0f;
    uint32_t childU = kNoChild;
    uint32_t childW = kNoChild;
  };

  std::vector<Shortcut> scOut;
  std::vector<AdjEntry> scNeigh;

  // Simulate contraction of v: which shortcuts would be needed. Result stays
  // valid until a neighbor of v is contracted.
  const std::vector<Shortcut>& simulate(int v, std::size_t settleCap) {
    scOut.clear();
    scNeigh = activeNeighbors(v);
    if (scNeigh.size() < 2) {
      return scOut;
    }
    if (scTargetSlot.empty()) {
      scTargetSlot.assign(adj.size(), -1);
    }
    for (std::size_t i = 0; i + 1 < scNeigh.size(); ++i) {
      const int u = scNeigh[i].to;
      const float wUV = arcs[scNeigh[i].arcId].weight;
      scTargetNode.clear();
      scTargetBound.clear();
      scFound.clear();
      for (std::size_t j = i + 1; j < scNeigh.size(); ++j) {
        scTargetNode.push_back(scNeigh[j].to);
        scTargetBound.push_back(wUV + arcs[scNeigh[j].arcId].weight);
        scFound.push_back(0);
        scTargetSlot[static_cast<std::size_t>(scNeigh[j].to)] = static_cast<int32_t>(j - i - 1);
      }
      witness(u, v, settleCap);
      for (std::size_t j = 0; j < scTargetNode.size(); ++j) {
        scTargetSlot[static_cast<std::size_t>(scTargetNode[j])] = -1;
        if (!scFound[j]) {
          Shortcut sc;
          sc.u = u;
          sc.w = scTargetNode[j];
          sc.weight = scTargetBound[j];
          sc.childU = scNeigh[i].arcId;
          sc.childW = scNeigh[i + 1 + j].arcId;
          scOut.push_back(sc);
        }
      }
    }
    return scOut;
  }

  // Drop adjacency entries pointing at contracted nodes once they dominate the
  // list. Owned arcs (this node contracted first) are never dropped because
  // compaction only runs on live nodes.
  void compactAdj(int v) {
    std::vector<AdjEntry>& list = adj[static_cast<std::size_t>(v)];
    std::size_t dead = 0;
    for (const AdjEntry& e : list) {
      if (removed[static_cast<std::size_t>(e.to)]) {
        ++dead;
      }
    }
    if (dead * 2 < list.size() || dead < 8) {
      return;
    }
    std::size_t w = 0;
    for (std::size_t r = 0; r < list.size(); ++r) {
      if (!removed[static_cast<std::size_t>(list[r].to)]) {
        list[w++] = list[r];
      }
    }
    list.resize(w);
  }

  std::vector<uint32_t> contract() {
    const int nn = n();
    std::vector<uint32_t> rank(static_cast<std::size_t>(nn), 0);
    using QItem = std::pair<float, int>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;

    const auto prioOf = [&](int v, std::size_t shortcutCount, std::size_t deg) -> float {
      return 2.0f * (static_cast<float>(shortcutCount) - static_cast<float>(deg)) +
             static_cast<float>(delNeighbors[static_cast<std::size_t>(v)]);
    };

    const auto t0 = std::chrono::steady_clock::now();
    // Exact initial priorities (graph is sparse here, witness searches are cheap).
    for (int v = 0; v < nn; ++v) {
      const std::vector<Shortcut>& sc = simulate(v, 500);
      const std::size_t deg = adj[static_cast<std::size_t>(v)].size();
      pq.push({prioOf(v, sc.size(), deg), v});
      if ((v + 1) % 2000000 == 0) {
        std::cerr << "[full_ch] init prio " << (v + 1) << "/" << nn << "\n" << std::flush;
      }
    }
    const auto tInit = std::chrono::steady_clock::now();
    std::cerr << "[full_ch] init prio done ("
              << std::chrono::duration_cast<std::chrono::seconds>(tInit - t0).count() << "s)\n"
              << std::flush;

    uint32_t nextRank = 0;
    int64_t contracted = 0;
    int64_t shortcutsAdded = 0;
    int64_t requeues = 0;
    auto tLast = std::chrono::steady_clock::now();
    while (!pq.empty()) {
      const auto [key, v] = pq.top();
      pq.pop();
      if (removed[static_cast<std::size_t>(v)]) {
        continue;
      }
      // Larger cap near the core: false shortcuts there trigger degree blowup.
      const std::size_t remainingNodes = static_cast<std::size_t>(nn) - contracted;
      const std::size_t settleCap = remainingNodes < 200000 ? 4000 : 800;
      const std::vector<Shortcut>& shortcuts = simulate(v, settleCap);
      const std::size_t deg = scNeigh.size();
      const float prio = prioOf(v, shortcuts.size(), deg);
      if (!pq.empty() && prio > pq.top().first + 1e-6f) {
        pq.push({prio, v});
        ++requeues;
        continue;
      }

      rank[static_cast<std::size_t>(v)] = nextRank++;
      removed[static_cast<std::size_t>(v)] = 1;
      ++contracted;
      // simulate() results reference scNeigh/scOut scratch; copy before addArc.
      const std::vector<Shortcut> scCopy = shortcuts;
      const std::vector<AdjEntry> neighCopy = scNeigh;
      for (const Shortcut& sc : scCopy) {
        addArc(sc.u, sc.w, sc.weight, 0, 0.0f, 0.0f, sc.childU, sc.childW);
        ++shortcutsAdded;
      }
      for (const AdjEntry& e : neighCopy) {
        ++delNeighbors[static_cast<std::size_t>(e.to)];
        compactAdj(e.to);
      }
      if (contracted % 500000 == 0) {
        const auto now = std::chrono::steady_clock::now();
        std::cerr << "[full_ch] contracted " << contracted << "/" << nn
                  << " shortcuts=" << shortcutsAdded << " requeues=" << requeues << " (+"
                  << std::chrono::duration_cast<std::chrono::seconds>(now - tLast).count()
                  << "s)\n"
                  << std::flush;
        tLast = now;
      }
    }
    std::cerr << "[full_ch] contraction done nodes=" << contracted
              << " shortcuts=" << shortcutsAdded << " requeues=" << requeues
              << " arcs_total=" << arcs.size() << "\n"
              << std::flush;
    return rank;
  }
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: mmlp_build_full_ch <region.mmlp.bin> [csrPath] [outPath]\n"
              << "  default csr=<base>.csr out=<base>.full.ch (pass <base>.hwy.csr to\n"
              << "  build a nationwide highway-level CH)\n";
    return 1;
  }
  const std::string binPath = argv[1];
  const std::string base = binPath.substr(0, binPath.size() - 4);
  const std::string csrPath = argc > 2 ? argv[2] : base + ".csr";
  const std::string nidxPath = base + ".nidx";
  const std::string outPath = argc > 3 ? argv[3] : base + ".full.ch";

  const auto tStart = std::chrono::steady_clock::now();

  const int fd = ::open(csrPath.c_str(), O_RDONLY);
  if (fd < 0) {
    std::cerr << "cannot open " << csrPath << " (run mmlp_build_aux --csr-only first)\n";
    return 1;
  }
  struct stat st {};
  if (::fstat(fd, &st) != 0 || st.st_size <= 0) {
    ::close(fd);
    return 1;
  }
  void* map = ::mmap(nullptr, static_cast<std::size_t>(st.st_size), PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);
  if (map == MAP_FAILED) {
    return 1;
  }
  const char* basePtr = static_cast<const char*>(map);
  if (std::memcmp(basePtr, kCsrMagic, 8) != 0) {
    std::cerr << "invalid csr magic\n";
    return 1;
  }
  uint64_t nodeCount = 0;
  uint64_t arcCount = 0;
  std::memcpy(&nodeCount, basePtr + 12, 8);
  std::memcpy(&arcCount, basePtr + 20, 8);
  const auto* rowPtr = reinterpret_cast<const uint64_t*>(basePtr + 28);
  const std::size_t rowBytes = static_cast<std::size_t>(nodeCount + 1) * sizeof(uint64_t);
  const auto* csrArcs = reinterpret_cast<const CsrArcRec*>(basePtr + 28 + rowBytes);
  std::cerr << "[full_ch] csr nodes=" << nodeCount << " arcs=" << arcCount << "\n";

  const int nfd = ::open(nidxPath.c_str(), O_RDONLY);
  if (nfd < 0) {
    std::cerr << "cannot open " << nidxPath << "\n";
    return 1;
  }
  struct stat nst {};
  ::fstat(nfd, &nst);
  void* nmap =
      ::mmap(nullptr, static_cast<std::size_t>(nst.st_size), PROT_READ, MAP_SHARED, nfd, 0);
  ::close(nfd);
  if (nmap == MAP_FAILED) {
    return 1;
  }
  struct IdOff {
    int64_t id = 0;
    uint64_t off = 0;
  };
  const auto* nrows = reinterpret_cast<const IdOff*>(static_cast<const char*>(nmap) + 20);

  // Active nodes: at least one ROAD arc.
  std::vector<uint64_t> activeRows;
  activeRows.reserve(nodeCount);
  for (uint64_t r = 0; r < nodeCount; ++r) {
    for (uint64_t i = rowPtr[r]; i < rowPtr[r + 1]; ++i) {
      if (csrArcs[i].edgeType == 0) {
        activeRows.push_back(r);
        break;
      }
    }
  }
  std::cerr << "[full_ch] active road nodes=" << activeRows.size() << "\n";
  if (activeRows.empty()) {
    return 1;
  }

  std::vector<int32_t> rowToCompact(nodeCount, -1);
  Builder b;
  b.globalId.reserve(activeRows.size());
  for (uint64_t r : activeRows) {
    rowToCompact[r] = static_cast<int32_t>(b.globalId.size());
    b.globalId.push_back(nrows[r].id);
  }
  ::munmap(nmap, static_cast<std::size_t>(nst.st_size));

  // Global node id -> row for arc targets (nidx sorted by id; binary search once here).
  // CSR arc targets are node ids; we need their rows. Build id->compact via sorted ids.
  std::vector<std::pair<int64_t, int32_t>> idToCompact;
  idToCompact.reserve(b.globalId.size());
  for (std::size_t i = 0; i < b.globalId.size(); ++i) {
    idToCompact.push_back({b.globalId[i], static_cast<int32_t>(i)});
  }
  std::sort(idToCompact.begin(), idToCompact.end());
  const auto lookupCompact = [&](int64_t id) -> int32_t {
    auto it = std::lower_bound(idToCompact.begin(), idToCompact.end(),
                               std::make_pair(id, static_cast<int32_t>(-1)),
                               [](const auto& a, const auto& p) { return a.first < p.first; });
    if (it == idToCompact.end() || it->first != id) {
      return -1;
    }
    return it->second;
  };

  b.init(b.globalId.size());
  b.arcs.reserve(arcCount / 2 + arcCount / 4);

  for (uint64_t r = 0; r < nodeCount; ++r) {
    const int32_t u = rowToCompact[r];
    if (u < 0) {
      continue;
    }
    for (uint64_t i = rowPtr[r]; i < rowPtr[r + 1]; ++i) {
      const CsrArcRec& a = csrArcs[i];
      if (a.edgeType != 0) {
        continue;
      }
      const int32_t v = lookupCompact(a.toNodeId);
      if (v < 0 || v == u || v < u) {  // process each undirected pair once (u < v)
        continue;
      }
      b.addArc(u, v, roadWeightSec(a.length, a.speedLimit), a.edgeId, a.length, a.speedLimit,
               kNoChild, kNoChild);
    }
  }
  ::munmap(map, static_cast<std::size_t>(st.st_size));
  std::cerr << "[full_ch] base arcs=" << b.arcs.size() << "\n";

  const std::vector<uint32_t> ranks = b.contract();

  // Emit file: nodes sorted by global id; arcs grouped by lower-rank endpoint.
  const int nn = b.n();
  std::vector<int> order(nn);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int x, int y) { return b.globalId[x] < b.globalId[y]; });
  std::vector<int32_t> remap(nn);
  std::vector<int64_t> sortedIds(nn);
  std::vector<uint32_t> sortedRanks(nn);
  for (int i = 0; i < nn; ++i) {
    remap[order[i]] = i;
    sortedIds[i] = b.globalId[order[i]];
    sortedRanks[i] = ranks[order[i]];
  }

  // First pass: assign file index per arc (owner = lower-rank endpoint).
  std::vector<uint32_t> arcFileIdx(b.arcs.size(), kNoChild);
  std::vector<uint64_t> upRow(static_cast<std::size_t>(nn) + 1, 0);
  uint64_t fileArcCount = 0;
  for (int fi = 0; fi < nn; ++fi) {
    upRow[fi] = fileArcCount;
    const int u = order[fi];
    for (const AdjEntry& e : b.adj[static_cast<std::size_t>(u)]) {
      const BuildArc& arc = b.arcs[e.arcId];
      const int other = (arc.a == u) ? arc.b : arc.a;
      if (ranks[static_cast<std::size_t>(u)] < ranks[static_cast<std::size_t>(other)]) {
        if (arcFileIdx[e.arcId] == kNoChild) {
          arcFileIdx[e.arcId] = static_cast<uint32_t>(fileArcCount++);
        }
      }
    }
  }
  upRow[nn] = fileArcCount;

  std::vector<FullChArcRec> fileArcs(fileArcCount);
  for (int fi = 0; fi < nn; ++fi) {
    const int u = order[fi];
    for (const AdjEntry& e : b.adj[static_cast<std::size_t>(u)]) {
      const BuildArc& arc = b.arcs[e.arcId];
      const int other = (arc.a == u) ? arc.b : arc.a;
      if (ranks[static_cast<std::size_t>(u)] >= ranks[static_cast<std::size_t>(other)]) {
        continue;
      }
      const uint32_t idx = arcFileIdx[e.arcId];
      FullChArcRec rec;
      rec.lo = static_cast<uint32_t>(fi);
      rec.hi = static_cast<uint32_t>(remap[other]);
      rec.edgeId = arc.edgeId;
      rec.weightSec = arc.weight;
      rec.lengthM = arc.length;
      rec.speedLimit = arc.speedLimit;
      if (arc.childToA != kNoChild) {
        // children stored as (via<->a, via<->b); orient to (via<->lo, via<->hi)
        const uint32_t childLo = (arc.a == u) ? arc.childToA : arc.childToB;
        const uint32_t childHi = (arc.a == u) ? arc.childToB : arc.childToA;
        rec.childA = arcFileIdx[childLo];
        rec.childB = arcFileIdx[childHi];
      }
      fileArcs[idx] = rec;
    }
  }

  const std::size_t header = 32;
  const std::size_t idsBytes = sortedIds.size() * sizeof(int64_t);
  const std::size_t ranksBytes = sortedRanks.size() * sizeof(uint32_t);
  const std::size_t rowsBytes = upRow.size() * sizeof(uint64_t);
  const std::size_t arcsBytes = fileArcs.size() * sizeof(FullChArcRec);

  std::ofstream out(outPath, std::ios::binary);
  char head[32] = {};
  std::memcpy(head, kOutMagic, 8);
  const uint32_t ver = 1;
  const uint64_t ncnt = sortedIds.size();
  const uint64_t acnt = fileArcs.size();
  const float profile = static_cast<float>(kProfileKmh);
  std::memcpy(head + 8, &ver, 4);
  std::memcpy(head + 12, &ncnt, 8);
  std::memcpy(head + 20, &acnt, 8);
  std::memcpy(head + 28, &profile, 4);
  out.write(head, 32);
  out.write(reinterpret_cast<const char*>(sortedIds.data()), static_cast<std::streamsize>(idsBytes));
  out.write(reinterpret_cast<const char*>(sortedRanks.data()),
            static_cast<std::streamsize>(ranksBytes));
  out.write(reinterpret_cast<const char*>(upRow.data()), static_cast<std::streamsize>(rowsBytes));
  out.write(reinterpret_cast<const char*>(fileArcs.data()), static_cast<std::streamsize>(arcsBytes));
  out.close();

  const auto totalS = std::chrono::duration_cast<std::chrono::seconds>(
                          std::chrono::steady_clock::now() - tStart)
                          .count();
  std::cerr << "[full_ch] wrote " << outPath << " nodes=" << ncnt << " arcs=" << acnt << " ("
            << ((header + idsBytes + ranksBytes + rowsBytes + arcsBytes) / 1e6) << " MB, "
            << totalS << "s)\n";
  return 0;
}
