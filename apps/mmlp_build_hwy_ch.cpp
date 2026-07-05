// Build highway Contraction Hierarchies overlay: china.mmlp.hwy.ch
#include <algorithm>
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
#include <tuple>
#include <unordered_map>
#include <unistd.h>
#include <vector>

namespace {

constexpr char kCsrMagic[8] = {'M', 'M', 'L', 'P', 'C', 'S', 'R', '\0'};
constexpr char kOutMagic[8] = {'M', 'M', 'L', 'P', 'C', 'H', '0', '2'};
constexpr double kProfileKmh = 80.0;
constexpr double kProfileMs = kProfileKmh / 3.6;

#pragma pack(push, 1)
struct CsrArcRec {
  int64_t toNodeId = 0;
  int64_t edgeId = 0;
  int32_t edgeType = 0;
  float length = 0.0f;
  float speedLimit = 0.0f;
};
struct ChUpArcRec {
  int64_t toNodeId = 0;
  int64_t edgeId = 0;
  float length = 0.0f;
};
#pragma pack(pop)

struct Arc {
  int to = -1;
  float length = 0.0f;
  int64_t edgeId = 0;
  bool shortcut = false;
};

struct Graph {
  std::vector<int64_t> globalId;
  std::vector<std::vector<Arc>> adj;
  std::vector<bool> removed;

  int n() const { return static_cast<int>(globalId.size()); }

  void addEdge(int u, int v, float len, int64_t eid, bool shortcut) {
    if (u < 0 || v < 0 || u >= n() || v >= n() || u == v) {
      return;
    }
    auto& list = adj[static_cast<std::size_t>(u)];
    for (Arc& arc : list) {
      if (arc.to == v && !arc.shortcut) {
        if (len < arc.length) {
          arc.length = len;
          arc.edgeId = eid;
        }
        return;
      }
    }
    list.push_back({v, len, eid, shortcut});
  }

  float edgeLen(int u, int v) const {
    for (const Arc& arc : adj[static_cast<std::size_t>(u)]) {
      if (arc.to == v) {
        return arc.length;
      }
    }
    return std::numeric_limits<float>::infinity();
  }

  std::vector<int> activeNeighbors(int v) const {
    std::vector<int> out;
    for (const Arc& arc : adj[static_cast<std::size_t>(v)]) {
      if (!removed[static_cast<std::size_t>(arc.to)]) {
        out.push_back(arc.to);
      }
    }
    return out;
  }
};

float travelSec(float lengthM) {
  return static_cast<float>(lengthM / kProfileMs);
}

bool witnessPath(const Graph& g, int source, int target, int banned, float maxSec) {
  const int n = g.n();
  std::vector<float> dist(static_cast<std::size_t>(n), std::numeric_limits<float>::infinity());
  using QItem = std::pair<float, int>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
  dist[static_cast<std::size_t>(source)] = 0.0f;
  pq.push({0.0f, source});
  std::size_t visited = 0;
  constexpr std::size_t kWitnessVisitCap = 50000;
  while (!pq.empty() && visited < kWitnessVisitCap) {
    const auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[static_cast<std::size_t>(u)] + 1e-6f) {
      continue;
    }
    if (u == target) {
      return d <= maxSec + 1e-5f;
    }
    ++visited;
    if (d > maxSec + 1e-5f) {
      continue;
    }
    for (const Arc& arc : g.adj[static_cast<std::size_t>(u)]) {
      if (arc.to == banned || g.removed[static_cast<std::size_t>(arc.to)]) {
        continue;
      }
      const float nd = d + travelSec(arc.length);
      if (nd < dist[static_cast<std::size_t>(arc.to)] - 1e-6f) {
        dist[static_cast<std::size_t>(arc.to)] = nd;
        pq.push({nd, arc.to});
      }
    }
  }
  return false;
}

std::vector<uint32_t> contractGraph(Graph& g) {
  const int n = g.n();
  std::vector<uint32_t> rank(static_cast<std::size_t>(n), 0);
  std::vector<int> priority(static_cast<std::size_t>(n), 0);
  using QItem = std::pair<int, int>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;

  auto recompute = [&](int v) {
    if (g.removed[static_cast<std::size_t>(v)]) {
      return;
    }
    priority[static_cast<std::size_t>(v)] =
        static_cast<int>(g.activeNeighbors(v).size());
  };

  for (int v = 0; v < n; ++v) {
    recompute(v);
    pq.push({priority[static_cast<std::size_t>(v)], v});
  }

  uint32_t nextRank = 0;
  int contracted = 0;
  while (!pq.empty()) {
    const auto [prio, v] = pq.top();
    pq.pop();
    if (g.removed[static_cast<std::size_t>(v)]) {
      continue;
    }
    if (prio != priority[static_cast<std::size_t>(v)]) {
      pq.push({priority[static_cast<std::size_t>(v)], v});
      continue;
    }

    rank[static_cast<std::size_t>(v)] = nextRank++;
    g.removed[static_cast<std::size_t>(v)] = true;
    ++contracted;
    if (contracted % 20000 == 0) {
      std::cerr << "[hwy_ch] contracted " << contracted << "/" << n << "\n";
    }

    const std::vector<int> neigh = g.activeNeighbors(v);
    for (std::size_t i = 0; i < neigh.size(); ++i) {
      for (std::size_t j = i + 1; j < neigh.size(); ++j) {
        const int u = neigh[i];
        const int w = neigh[j];
        const float via = g.edgeLen(u, v) + g.edgeLen(v, w);
        if (!std::isfinite(via)) {
          continue;
        }
        const float maxSec = travelSec(via);
        if (!witnessPath(g, u, w, v, maxSec)) {
          g.addEdge(u, w, via, 0, true);
          g.addEdge(w, u, via, 0, true);
        }
      }
    }

    for (int u : neigh) {
      recompute(u);
      pq.push({priority[static_cast<std::size_t>(u)], u});
    }
  }
  return rank;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: mmlp_build_hwy_ch <china.mmlp.bin>\n";
    return 1;
  }
  const std::string base = std::string(argv[1]).substr(0, std::string(argv[1]).size() - 4);
  const std::string csrPath = base + ".hwy.csr";
  const std::string nidxPath = base + ".nidx";
  const std::string outPath = base + ".hwy.ch";

  const int fd = ::open(csrPath.c_str(), O_RDONLY);
  if (fd < 0) {
    std::cerr << "cannot open " << csrPath << " (run mmlp_build_hwy_csr first)\n";
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
  uint64_t nodeCount = 0;
  uint64_t arcCount = 0;
  std::memcpy(&nodeCount, basePtr + 12, 8);
  std::memcpy(&arcCount, basePtr + 20, 8);
  std::cerr << "[hwy_ch] csr nodes=" << nodeCount << " arcs=" << arcCount << "\n";
  if (nodeCount == 0 || arcCount == 0) {
    std::cerr << "[hwy_ch] empty highway csr\n";
    ::munmap(map, static_cast<std::size_t>(st.st_size));
    return 1;
  }

  const auto* rowPtr = reinterpret_cast<const uint64_t*>(basePtr + 28);
  const std::size_t csrRowBytes = static_cast<std::size_t>(nodeCount + 1) * sizeof(uint64_t);
  const std::size_t csrArcBytes = static_cast<std::size_t>(arcCount) * sizeof(CsrArcRec);
  const std::size_t expected = 28 + csrRowBytes + csrArcBytes;
  if (static_cast<std::size_t>(st.st_size) < expected) {
    std::cerr << "[hwy_ch] truncated csr (" << st.st_size << " < " << expected << ")\n";
    ::munmap(map, static_cast<std::size_t>(st.st_size));
    return 1;
  }
  const auto* arcs = reinterpret_cast<const CsrArcRec*>(basePtr + 28 + csrRowBytes);

  const int nfd = ::open(nidxPath.c_str(), O_RDONLY);
  if (nfd < 0) {
    std::cerr << "cannot open " << nidxPath << "\n";
    ::munmap(map, static_cast<std::size_t>(st.st_size));
    return 1;
  }
  struct stat nst {};
  ::fstat(nfd, &nst);
  void* nmap = ::mmap(nullptr, static_cast<std::size_t>(nst.st_size), PROT_READ, MAP_SHARED, nfd, 0);
  ::close(nfd);
  if (nmap == MAP_FAILED) {
    ::munmap(map, static_cast<std::size_t>(st.st_size));
    return 1;
  }
  struct IdOff {
    int64_t id = 0;
    uint64_t off = 0;
  };
  const auto* nrows = reinterpret_cast<const IdOff*>(static_cast<const char*>(nmap) + 20);

  std::vector<uint64_t> activeRows;
  activeRows.reserve(700000);
  for (uint64_t r = 0; r < nodeCount; ++r) {
    if (rowPtr[r + 1] > rowPtr[r]) {
      activeRows.push_back(r);
    }
  }
  std::cerr << "[hwy_ch] active highway rows=" << activeRows.size() << "\n";
  if (activeRows.empty()) {
    ::munmap(nmap, static_cast<std::size_t>(nst.st_size));
    ::munmap(map, static_cast<std::size_t>(st.st_size));
    return 1;
  }

  std::vector<int64_t> activeGlobal;
  activeGlobal.reserve(activeRows.size());
  std::unordered_map<uint64_t, int> rowToCompact;
  rowToCompact.reserve(activeRows.size() * 2);
  for (uint64_t r : activeRows) {
    rowToCompact[r] = static_cast<int>(activeGlobal.size());
    activeGlobal.push_back(nrows[r].id);
  }
  ::munmap(nmap, static_cast<std::size_t>(nst.st_size));

  std::unordered_map<int64_t, int> idToCompact;
  idToCompact.reserve(activeGlobal.size() * 2);
  for (std::size_t i = 0; i < activeGlobal.size(); ++i) {
    idToCompact[activeGlobal[i]] = static_cast<int>(i);
  }

  Graph g;
  g.globalId = std::move(activeGlobal);
  g.adj.assign(g.globalId.size(), {});
  g.removed.assign(g.globalId.size(), false);

  std::cerr << "[hwy_ch] active nodes=" << g.n() << " arcs=" << arcCount << "\n";
  for (uint64_t r : activeRows) {
    const auto rowIt = rowToCompact.find(r);
    if (rowIt == rowToCompact.end()) {
      continue;
    }
    const int u = rowIt->second;
    for (uint64_t i = rowPtr[r]; i < rowPtr[r + 1]; ++i) {
      const CsrArcRec& a = arcs[i];
      const auto it = idToCompact.find(a.toNodeId);
      if (it == idToCompact.end()) {
        continue;
      }
      g.addEdge(u, it->second, a.length, a.edgeId, false);
    }
  }
  ::munmap(map, static_cast<std::size_t>(st.st_size));

  std::cerr << "[hwy_ch] contracting...\n";
  const std::vector<uint32_t> ranks = contractGraph(g);

  std::vector<int> order(g.n());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return g.globalId[static_cast<std::size_t>(a)] < g.globalId[static_cast<std::size_t>(b)];
  });
  std::vector<int64_t> sortedIds(g.n());
  std::vector<uint32_t> sortedRanks(g.n());
  std::vector<int> remap(g.n());
  for (int i = 0; i < g.n(); ++i) {
    sortedIds[static_cast<std::size_t>(i)] = g.globalId[static_cast<std::size_t>(order[i])];
    sortedRanks[static_cast<std::size_t>(i)] = ranks[static_cast<std::size_t>(order[i])];
    remap[order[static_cast<std::size_t>(i)]] = i;
  }

  std::vector<ChUpArcRec> upArcs;
  upArcs.reserve(g.n() * 4);
  std::vector<uint64_t> upRow(static_cast<std::size_t>(g.n()) + 1, 0);
  for (int u = 0; u < g.n(); ++u) {
    const int nu = remap[u];
    const uint32_t ru = ranks[static_cast<std::size_t>(u)];
    upRow[static_cast<std::size_t>(nu)] = upArcs.size();
    for (const Arc& arc : g.adj[static_cast<std::size_t>(u)]) {
      const int v = arc.to;
      if (ranks[static_cast<std::size_t>(v)] <= ru) {
        continue;
      }
      ChUpArcRec out;
      out.toNodeId = g.globalId[static_cast<std::size_t>(v)];
      out.edgeId = arc.edgeId;
      out.length = arc.length;
      upArcs.push_back(out);
    }
  }
  upRow.back() = upArcs.size();

  const std::size_t header = 32;
  const std::size_t idsBytes = sortedIds.size() * sizeof(int64_t);
  const std::size_t ranksBytes = sortedRanks.size() * sizeof(uint32_t);
  const std::size_t rowBytes = upRow.size() * sizeof(uint64_t);
  const std::size_t arcBytes = upArcs.size() * sizeof(ChUpArcRec);
  std::vector<char> file(header + idsBytes + ranksBytes + rowBytes + arcBytes);
  std::memcpy(file.data(), kOutMagic, 8);
  const uint32_t ver = 1;
  const uint64_t ncnt = sortedIds.size();
  const uint64_t acnt = upArcs.size();
  const float profile = static_cast<float>(kProfileKmh);
  std::memcpy(file.data() + 8, &ver, 4);
  std::memcpy(file.data() + 12, &ncnt, 8);
  std::memcpy(file.data() + 20, &acnt, 8);
  std::memcpy(file.data() + 28, &profile, 4);
  std::size_t off = header;
  std::memcpy(file.data() + off, sortedIds.data(), idsBytes);
  off += idsBytes;
  std::memcpy(file.data() + off, sortedRanks.data(), ranksBytes);
  off += ranksBytes;
  std::memcpy(file.data() + off, upRow.data(), rowBytes);
  off += rowBytes;
  std::memcpy(file.data() + off, upArcs.data(), arcBytes);

  std::ofstream out(outPath, std::ios::binary);
  out.write(file.data(), static_cast<std::streamsize>(file.size()));
  std::cerr << "[hwy_ch] wrote " << outPath << " nodes=" << ncnt << " up_arcs=" << acnt
            << " (" << (file.size() / 1e6) << " MB)\n";
  return 0;
}
