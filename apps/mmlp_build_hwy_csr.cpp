// Build highway (arterial) CSR overlay: china.mmlp.hwy.csr
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <numeric>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr char kBinMagic[] = "MMLPGRPH";
constexpr char kCsrMagic[8] = {'M', 'M', 'L', 'P', 'C', 'S', 'R', '\0'};
constexpr std::size_t kNodeRecord = 28;
constexpr double kMinSpeedKmh = 60.0;
constexpr double kSpurGlueM = 25000.0;
constexpr double kMaxGlueDistM = 150000.0;
constexpr double kMaxBridgeCentroidM = 350000.0;
constexpr int kMaxBridgeTries = 300;
constexpr double kMicroGlueDistM = 40000.0;
constexpr double kMicroCentroidM = 50000.0;
constexpr int kMicroCompMaxNodes = 48;

struct NodeRow {
  int64_t id = 0;
  double lat = 0.0;
  double lon = 0.0;
  uint64_t offset = 0;
};

#pragma pack(push, 1)
struct CsrArcRec {
  int64_t toNodeId = 0;
  int64_t edgeId = 0;
  int32_t edgeType = 0;
  float length = 0.0f;
  float speedLimit = 0.0f;
};
#pragma pack(pop)

int nodeRowOf(const std::vector<NodeRow>& nodes, int64_t id) {
  auto it = std::lower_bound(nodes.begin(), nodes.end(), id,
                             [](const NodeRow& r, int64_t k) { return r.id < k; });
  if (it == nodes.end() || it->id != id) {
    return -1;
  }
  return static_cast<int>(it - nodes.begin());
}

bool isPrimaryHighwayEdge(int32_t type, double speedLimit, double length) {
  if (type != 0) {
    return false;
  }
  if (speedLimit >= kMinSpeedKmh) {
    return true;
  }
  if (speedLimit >= 45.0 && length >= 400.0) {
    return true;
  }
  return false;
}

bool includeInOverlay(int32_t type, double speedLimit, double length, int frRow, int toRow,
                      const std::vector<char>& active,
                      const std::vector<uint32_t>& primaryDegree) {
  if (isPrimaryHighwayEdge(type, speedLimit, length)) {
    return true;
  }
  // Glue spur components: any ROAD between two already-active overlay nodes.
  if (type == 0 && frRow >= 0 && toRow >= 0 && active[static_cast<std::size_t>(frRow)] != 0 &&
      active[static_cast<std::size_t>(toRow)] != 0) {
    return true;
  }
  // Short ROAD spur directly off a primary highway node (portal ramp).
  if (type == 0 && length <= kSpurGlueM) {
    if (frRow >= 0 && primaryDegree[static_cast<std::size_t>(frRow)] > 0) {
      return true;
    }
    if (toRow >= 0 && primaryDegree[static_cast<std::size_t>(toRow)] > 0) {
      return true;
    }
  }
  return false;
}

struct OverlayCompInfo {
  double latSum = 0.0;
  double lonSum = 0.0;
  int count = 0;
  std::vector<int> seeds;
};

int countOverlayComponents(const std::vector<std::vector<int>>& overlayAdj,
                           const std::vector<char>& inOverlay,
                           std::vector<int>* outComp = nullptr) {
  const std::size_t n = overlayAdj.size();
  std::vector<int> comp(n, -1);
  int numComp = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (!inOverlay[i] || comp[i] >= 0) {
      continue;
    }
    std::deque<int> q;
    q.push_back(static_cast<int>(i));
    comp[i] = numComp;
    while (!q.empty()) {
      const int u = q.front();
      q.pop_front();
      for (int v : overlayAdj[static_cast<std::size_t>(u)]) {
        if (!inOverlay[static_cast<std::size_t>(v)] || comp[static_cast<std::size_t>(v)] >= 0) {
          continue;
        }
        comp[static_cast<std::size_t>(v)] = numComp;
        q.push_back(v);
      }
    }
    ++numComp;
  }
  if (outComp) {
    *outComp = std::move(comp);
  }
  return numComp;
}

void buildOverlayAdjacency(std::ifstream& in, uint64_t edgeCount, std::streamoff edgeStart,
                           const std::vector<NodeRow>& nodes, const std::vector<char>& active,
                           const std::vector<uint32_t>& primaryDegree,
                           std::vector<std::vector<int>>* overlayAdj,
                           std::vector<char>* inOverlay) {
  const std::size_t nodeCount = nodes.size();
  overlayAdj->assign(nodeCount, {});
  inOverlay->assign(nodeCount, 0);
  in.seekg(static_cast<std::streamoff>(edgeStart));
  for (uint64_t i = 0; i < edgeCount; ++i) {
    int64_t id = 0;
    int64_t fr = 0;
    int64_t to = 0;
    int32_t type = 0;
    double length = 0.0;
    double speed = 0.0;
    if (!in.read(reinterpret_cast<char*>(&id), 8) ||
        !in.read(reinterpret_cast<char*>(&fr), 8) || !in.read(reinterpret_cast<char*>(&to), 8) ||
        !in.read(reinterpret_cast<char*>(&type), 4) ||
        !in.read(reinterpret_cast<char*>(&length), 8) ||
        !in.read(reinterpret_cast<char*>(&speed), 8)) {
      break;
    }
    const int frRow = nodeRowOf(nodes, fr);
    const int toRow = nodeRowOf(nodes, to);
    if (!includeInOverlay(type, speed, length, frRow, toRow, active, primaryDegree)) {
      continue;
    }
    if (frRow >= 0) {
      (*inOverlay)[static_cast<std::size_t>(frRow)] = 1;
    }
    if (toRow >= 0) {
      (*inOverlay)[static_cast<std::size_t>(toRow)] = 1;
    }
    if (frRow >= 0 && toRow >= 0) {
      (*overlayAdj)[static_cast<std::size_t>(frRow)].push_back(toRow);
      (*overlayAdj)[static_cast<std::size_t>(toRow)].push_back(frRow);
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: mmlp_build_hwy_csr <china.mmlp.bin>\n";
    return 1;
  }
  const std::string binPath = argv[1];
  const std::string base = binPath.substr(0, binPath.size() - 4);
  const std::string csrPath = base + ".hwy.csr";

  std::ifstream in(binPath, std::ios::binary);
  if (!in) {
    std::cerr << "cannot open " << binPath << "\n";
    return 1;
  }

  char magic[8] = {};
  uint32_t version = 0;
  uint64_t nodeCount = 0;
  uint64_t edgeCount = 0;
  if (!in.read(magic, 8) || std::memcmp(magic, kBinMagic, 8) != 0 ||
      !in.read(reinterpret_cast<char*>(&version), 4) ||
      !in.read(reinterpret_cast<char*>(&nodeCount), 8) ||
      !in.read(reinterpret_cast<char*>(&edgeCount), 8)) {
    std::cerr << "invalid bin header\n";
    return 1;
  }

  std::vector<NodeRow> nodes(nodeCount);
  std::vector<double> nodeLat(static_cast<std::size_t>(nodeCount));
  std::vector<double> nodeLon(static_cast<std::size_t>(nodeCount));
  for (uint64_t i = 0; i < nodeCount; ++i) {
    int64_t id = 0;
    double lat = 0.0;
    double lon = 0.0;
    int32_t kind = 0;
    if (!in.read(reinterpret_cast<char*>(&id), 8) ||
        !in.read(reinterpret_cast<char*>(&lat), 8) ||
        !in.read(reinterpret_cast<char*>(&lon), 8) ||
        !in.read(reinterpret_cast<char*>(&kind), 4)) {
      std::cerr << "truncated nodes\n";
      return 1;
    }
    nodes[i].id = id;
    nodes[i].lat = lat;
    nodes[i].lon = lon;
    nodes[i].offset = 28 + i * kNodeRecord;
    nodeLat[static_cast<std::size_t>(i)] = lat;
    nodeLon[static_cast<std::size_t>(i)] = lon;
  }
  std::sort(nodes.begin(), nodes.end(),
            [](const NodeRow& a, const NodeRow& b) { return a.id < b.id; });

  const auto edgeStart = 28 + nodeCount * kNodeRecord;
  in.seekg(static_cast<std::streamoff>(edgeStart));

  std::vector<uint32_t> primaryDegree(static_cast<std::size_t>(nodeCount), 0);
  std::cerr << "[hwy_csr] pass1 primary degree\n";
  for (uint64_t i = 0; i < edgeCount; ++i) {
    int64_t id = 0;
    int64_t fr = 0;
    int64_t to = 0;
    int32_t type = 0;
    double length = 0.0;
    double speed = 0.0;
    if (!in.read(reinterpret_cast<char*>(&id), 8) ||
        !in.read(reinterpret_cast<char*>(&fr), 8) || !in.read(reinterpret_cast<char*>(&to), 8) ||
        !in.read(reinterpret_cast<char*>(&type), 4) ||
        !in.read(reinterpret_cast<char*>(&length), 8) ||
        !in.read(reinterpret_cast<char*>(&speed), 8)) {
      std::cerr << "truncated edges\n";
      return 1;
    }
    if (!isPrimaryHighwayEdge(type, speed, length)) {
      continue;
    }
    const int frRow = nodeRowOf(nodes, fr);
    const int toRow = nodeRowOf(nodes, to);
    if (frRow >= 0) {
      ++primaryDegree[static_cast<std::size_t>(frRow)];
    }
    if (toRow >= 0) {
      ++primaryDegree[static_cast<std::size_t>(toRow)];
    }
  }

  std::vector<char> active(static_cast<std::size_t>(nodeCount), 0);
  std::size_t activeNodes = 0;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    if (primaryDegree[i] > 0) {
      active[i] = 1;
      ++activeNodes;
    }
  }
  std::cerr << "[hwy_csr] active_nodes=" << activeNodes << " (primary)\n";

  struct RoadEdge {
    int to = -1;
    double len = 0.0;
  };
  std::vector<std::vector<int>> hwyAdj(static_cast<std::size_t>(nodeCount));
  std::vector<std::vector<RoadEdge>> roadAdj(static_cast<std::size_t>(nodeCount));
  in.seekg(static_cast<std::streamoff>(edgeStart));
  std::cerr << "[hwy_csr] building hwy + road adjacency\n";
  for (uint64_t i = 0; i < edgeCount; ++i) {
    int64_t id = 0;
    int64_t fr = 0;
    int64_t to = 0;
    int32_t type = 0;
    double length = 0.0;
    double speed = 0.0;
    if (!in.read(reinterpret_cast<char*>(&id), 8) ||
        !in.read(reinterpret_cast<char*>(&fr), 8) || !in.read(reinterpret_cast<char*>(&to), 8) ||
        !in.read(reinterpret_cast<char*>(&type), 4) ||
        !in.read(reinterpret_cast<char*>(&length), 8) ||
        !in.read(reinterpret_cast<char*>(&speed), 8)) {
      break;
    }
    const int frRow = nodeRowOf(nodes, fr);
    const int toRow = nodeRowOf(nodes, to);
    if (frRow < 0 || toRow < 0) {
      continue;
    }
    if (isPrimaryHighwayEdge(type, speed, length)) {
      hwyAdj[static_cast<std::size_t>(frRow)].push_back(toRow);
      hwyAdj[static_cast<std::size_t>(toRow)].push_back(frRow);
    }
    if (type == 0) {
      const double len = std::max(0.0, length);
      roadAdj[static_cast<std::size_t>(frRow)].push_back({toRow, len});
      roadAdj[static_cast<std::size_t>(toRow)].push_back({frRow, len});
    }
  }

  // Connected components on the primary highway subgraph (ignore spur-only nodes).
  std::vector<int> comp(static_cast<std::size_t>(nodeCount), -1);
  int numComp = 0;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    if (primaryDegree[i] == 0 || comp[i] >= 0) {
      continue;
    }
    std::deque<int> q;
    q.push_back(static_cast<int>(i));
    comp[i] = numComp;
    while (!q.empty()) {
      const int u = q.front();
      q.pop_front();
      for (int v : hwyAdj[static_cast<std::size_t>(u)]) {
        if (primaryDegree[static_cast<std::size_t>(v)] == 0 ||
            comp[static_cast<std::size_t>(v)] >= 0) {
          continue;
        }
        comp[static_cast<std::size_t>(v)] = numComp;
        q.push_back(v);
      }
    }
    ++numComp;
  }
  std::cerr << "[hwy_csr] primary_components=" << numComp << "\n";

  struct CompInfo {
    double latSum = 0.0;
    double lonSum = 0.0;
    int count = 0;
    std::vector<int> seeds;
  };
  std::vector<CompInfo> comps(static_cast<std::size_t>(numComp));
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    if (primaryDegree[i] == 0 || comp[i] < 0) {
      continue;
    }
    CompInfo& ci = comps[static_cast<std::size_t>(comp[i])];
    ci.latSum += nodeLat[i];
    ci.lonSum += nodeLon[i];
    ++ci.count;
    if (ci.seeds.size() < 24) {
      ci.seeds.push_back(static_cast<int>(i));
    }
  }

  auto haversineM = [](double lat1, double lon1, double lat2, double lon2) {
    constexpr double kR = 6371000.0;
    const double p1 = lat1 * M_PI / 180.0;
    const double p2 = lat2 * M_PI / 180.0;
    const double dp = (lat2 - lat1) * M_PI / 180.0;
    const double dl = (lon2 - lon1) * M_PI / 180.0;
    const double a =
        std::sin(dp / 2) * std::sin(dp / 2) +
        std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
    return 2 * kR * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
  };

  auto activatePath = [&](const std::vector<int>& path) {
    for (int node : path) {
      if (active[static_cast<std::size_t>(node)] == 0) {
        active[static_cast<std::size_t>(node)] = 1;
        ++activeNodes;
      }
    }
  };

  auto bridgeComponents = [&](int compA, int compB, double maxGlueM) -> bool {
    const std::vector<int>& sources = comps[static_cast<std::size_t>(compA)].seeds;
    if (sources.empty()) {
      return false;
    }
    std::vector<double> dist(static_cast<std::size_t>(nodeCount), 1e100);
    std::vector<int> parent(static_cast<std::size_t>(nodeCount), -1);
    using DistNode = std::pair<double, int>;
    std::priority_queue<DistNode, std::vector<DistNode>, std::greater<DistNode>> pq;
    for (int seed : sources) {
      dist[static_cast<std::size_t>(seed)] = 0.0;
      pq.push({0.0, seed});
    }
    int hit = -1;
    while (!pq.empty()) {
      const auto [d, u] = pq.top();
      pq.pop();
      if (d > dist[static_cast<std::size_t>(u)] + 1e-6) {
        continue;
      }
      if (comp[static_cast<std::size_t>(u)] == compB) {
        hit = u;
        break;
      }
      if (d > maxGlueM + 1e-6) {
        continue;
      }
      for (const RoadEdge& edge : roadAdj[static_cast<std::size_t>(u)]) {
        const double nd = d + edge.len;
        if (nd > maxGlueM + 1e-6) {
          continue;
        }
        if (nd + 1e-6 < dist[static_cast<std::size_t>(edge.to)]) {
          dist[static_cast<std::size_t>(edge.to)] = nd;
          parent[static_cast<std::size_t>(edge.to)] = u;
          pq.push({nd, edge.to});
        }
      }
    }
    if (hit < 0) {
      return false;
    }
    std::vector<int> path;
    for (int cur = hit; cur >= 0; cur = parent[static_cast<std::size_t>(cur)]) {
      path.push_back(cur);
      comp[static_cast<std::size_t>(cur)] = compA;
    }
    activatePath(path);
    return true;
  };

  std::vector<int> uf(static_cast<std::size_t>(numComp));
  std::iota(uf.begin(), uf.end(), 0);
  auto findUf = [&](int x) {
    while (uf[static_cast<std::size_t>(x)] != x) {
      x = uf[static_cast<std::size_t>(x)];
    }
    return x;
  };
  auto uniteUf = [&](int a, int b) {
    a = findUf(a);
    b = findUf(b);
    if (a != b) {
      uf[static_cast<std::size_t>(a)] = b;
    }
  };

  std::size_t bridges = 0;
  std::size_t microBridges = 0;
  for (int c = 0; c < numComp; ++c) {
    if (comps[static_cast<std::size_t>(c)].count <= 0 ||
        comps[static_cast<std::size_t>(c)].count > kMicroCompMaxNodes) {
      continue;
    }
    if (findUf(c) != c) {
      continue;
    }
    const CompInfo& ca = comps[static_cast<std::size_t>(c)];
    const double clat = ca.latSum / std::max(1, ca.count);
    const double clon = ca.lonSum / std::max(1, ca.count);
    int bestB = -1;
    double bestDist = 1e100;
    for (int other = 0; other < numComp; ++other) {
      if (other == c || comps[static_cast<std::size_t>(other)].count <= 0 ||
          findUf(other) != other) {
        continue;
      }
      const CompInfo& cb = comps[static_cast<std::size_t>(other)];
      const double dlat = cb.latSum / std::max(1, cb.count);
      const double dlon = cb.lonSum / std::max(1, cb.count);
      const double d = haversineM(clat, clon, dlat, dlon);
      if (d < bestDist) {
        bestDist = d;
        bestB = other;
      }
    }
    if (bestB < 0 || bestDist > kMicroCentroidM) {
      continue;
    }
    if (bridgeComponents(c, bestB, kMicroGlueDistM)) {
      ++microBridges;
      uniteUf(c, bestB);
      for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (comp[i] == bestB) {
          comp[i] = c;
        }
      }
      CompInfo& keep = comps[static_cast<std::size_t>(c)];
      const CompInfo& merged = comps[static_cast<std::size_t>(bestB)];
      keep.latSum += merged.latSum;
      keep.lonSum += merged.lonSum;
      keep.count += merged.count;
    }
  }
  std::cerr << "[hwy_csr] micro_bridges=" << microBridges << "\n";

  for (int attempt = 0; attempt < kMaxBridgeTries; ++attempt) {
    std::vector<int> live;
    live.reserve(static_cast<std::size_t>(numComp));
    for (int c = 0; c < numComp; ++c) {
      if (comps[static_cast<std::size_t>(c)].count > 0 && findUf(c) == c) {
        live.push_back(c);
      }
    }
    if (live.size() <= 1) {
      break;
    }
    int bestA = -1;
    int bestB = -1;
    double bestDist = 1e100;
    for (std::size_t i = 0; i < live.size(); ++i) {
      const CompInfo& ca = comps[static_cast<std::size_t>(live[i])];
      const double clat = ca.latSum / std::max(1, ca.count);
      const double clon = ca.lonSum / std::max(1, ca.count);
      for (std::size_t j = i + 1; j < live.size(); ++j) {
        const CompInfo& cb = comps[static_cast<std::size_t>(live[j])];
        const double dlat = cb.latSum / std::max(1, cb.count);
        const double dlon = cb.lonSum / std::max(1, cb.count);
        const double d = haversineM(clat, clon, dlat, dlon);
        if (d < bestDist) {
          bestDist = d;
          bestA = live[i];
          bestB = live[j];
        }
      }
    }
    if (bestA < 0 || bestDist > kMaxBridgeCentroidM) {
      break;
    }
    if (!bridgeComponents(bestA, bestB, kMaxGlueDistM)) {
      comps[static_cast<std::size_t>(bestB)].count = 0;
      continue;
    }
    ++bridges;
    uniteUf(bestA, bestB);
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (comp[i] == bestB) {
        comp[i] = bestA;
      }
    }
    CompInfo& keep = comps[static_cast<std::size_t>(bestA)];
    const CompInfo& merged = comps[static_cast<std::size_t>(bestB)];
    keep.latSum += merged.latSum;
    keep.lonSum += merged.lonSum;
    keep.count += merged.count;
  }
  std::cerr << "[hwy_csr] component_bridges=" << bridges << " active_nodes=" << activeNodes
            << "\n";

  // Spur glue: activate endpoints of short ROAD edges directly off primary highway nodes.
  in.seekg(static_cast<std::streamoff>(edgeStart));
  std::size_t spurActivated = 0;
  for (uint64_t i = 0; i < edgeCount; ++i) {
    int64_t id = 0;
    int64_t fr = 0;
    int64_t to = 0;
    int32_t type = 0;
    double length = 0.0;
    double speed = 0.0;
    if (!in.read(reinterpret_cast<char*>(&id), 8) ||
        !in.read(reinterpret_cast<char*>(&fr), 8) || !in.read(reinterpret_cast<char*>(&to), 8) ||
        !in.read(reinterpret_cast<char*>(&type), 4) ||
        !in.read(reinterpret_cast<char*>(&length), 8) ||
        !in.read(reinterpret_cast<char*>(&speed), 8)) {
      break;
    }
    if (type != 0 || length > kSpurGlueM) {
      continue;
    }
    const int frRow = nodeRowOf(nodes, fr);
    const int toRow = nodeRowOf(nodes, to);
    if (frRow < 0 || toRow < 0) {
      continue;
    }
    const bool frPrimary = primaryDegree[static_cast<std::size_t>(frRow)] > 0;
    const bool toPrimary = primaryDegree[static_cast<std::size_t>(toRow)] > 0;
    if (!frPrimary && !toPrimary) {
      continue;
    }
    for (int row : {frRow, toRow}) {
      if (active[static_cast<std::size_t>(row)] == 0) {
        active[static_cast<std::size_t>(row)] = 1;
        ++activeNodes;
        ++spurActivated;
      }
    }
  }
  std::cerr << "[hwy_csr] spur_activated=" << spurActivated << " active_nodes=" << activeNodes
            << "\n";

  // Post-merge active regions: glue disconnected active islands via ROAD paths.
  {
    std::vector<int> activeComp(static_cast<std::size_t>(nodeCount), -1);
    int numActiveComp = 0;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (active[i] == 0 || activeComp[i] >= 0) {
        continue;
      }
      std::deque<int> q;
      q.push_back(static_cast<int>(i));
      activeComp[i] = numActiveComp;
      while (!q.empty()) {
        const int u = q.front();
        q.pop_front();
        for (const RoadEdge& edge : roadAdj[static_cast<std::size_t>(u)]) {
          if (active[static_cast<std::size_t>(edge.to)] == 0 ||
              activeComp[static_cast<std::size_t>(edge.to)] >= 0) {
            continue;
          }
          activeComp[static_cast<std::size_t>(edge.to)] = numActiveComp;
          q.push_back(edge.to);
        }
      }
      ++numActiveComp;
    }
    std::cerr << "[hwy_csr] active_components=" << numActiveComp << "\n";

    struct ActiveCompInfo {
      double latSum = 0.0;
      double lonSum = 0.0;
      int count = 0;
      std::vector<int> seeds;
    };
    std::vector<ActiveCompInfo> activeComps(static_cast<std::size_t>(numActiveComp));
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (active[i] == 0 || activeComp[i] < 0) {
        continue;
      }
      ActiveCompInfo& ci = activeComps[static_cast<std::size_t>(activeComp[i])];
      ci.latSum += nodeLat[i];
      ci.lonSum += nodeLon[i];
      ++ci.count;
      if (ci.seeds.size() < 32) {
        ci.seeds.push_back(static_cast<int>(i));
      }
    }

    const double postGlueM = 250000.0;
    const double postCentroidM = 600000.0;
    std::size_t postBridges = 0;
    for (int attempt = 0; attempt < 400 && numActiveComp > 1; ++attempt) {
      std::vector<int> live;
      for (int c = 0; c < numActiveComp; ++c) {
        if (activeComps[static_cast<std::size_t>(c)].count > 0) {
          live.push_back(c);
        }
      }
      if (live.size() <= 1) {
        break;
      }
      int bestA = -1;
      int bestB = -1;
      double bestDist = 1e100;
      for (std::size_t i = 0; i < live.size(); ++i) {
        const ActiveCompInfo& ca = activeComps[static_cast<std::size_t>(live[i])];
        const double clat = ca.latSum / std::max(1, ca.count);
        const double clon = ca.lonSum / std::max(1, ca.count);
        for (std::size_t j = i + 1; j < live.size(); ++j) {
          const ActiveCompInfo& cb = activeComps[static_cast<std::size_t>(live[j])];
          const double d = haversineM(clat, clon, cb.latSum / std::max(1, cb.count),
                                      cb.lonSum / std::max(1, cb.count));
          if (d < bestDist) {
            bestDist = d;
            bestA = live[i];
            bestB = live[j];
          }
        }
      }
      if (bestA < 0 || bestDist > postCentroidM) {
        break;
      }
      const std::vector<int>& sources = activeComps[static_cast<std::size_t>(bestA)].seeds;
      if (sources.empty()) {
        activeComps[static_cast<std::size_t>(bestB)].count = 0;
        continue;
      }
      std::vector<double> dist(static_cast<std::size_t>(nodeCount), 1e100);
      std::vector<int> parent(static_cast<std::size_t>(nodeCount), -1);
      using DistNode = std::pair<double, int>;
      std::priority_queue<DistNode, std::vector<DistNode>, std::greater<DistNode>> pq;
      for (int seed : sources) {
        dist[static_cast<std::size_t>(seed)] = 0.0;
        pq.push({0.0, seed});
      }
      int hit = -1;
      while (!pq.empty()) {
        const auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[static_cast<std::size_t>(u)] + 1e-6) {
          continue;
        }
        if (activeComp[static_cast<std::size_t>(u)] == bestB) {
          hit = u;
          break;
        }
        if (d > postGlueM + 1e-6) {
          continue;
        }
        for (const RoadEdge& edge : roadAdj[static_cast<std::size_t>(u)]) {
          const double nd = d + edge.len;
          if (nd > postGlueM + 1e-6) {
            continue;
          }
          if (nd + 1e-6 < dist[static_cast<std::size_t>(edge.to)]) {
            dist[static_cast<std::size_t>(edge.to)] = nd;
            parent[static_cast<std::size_t>(edge.to)] = u;
            pq.push({nd, edge.to});
          }
        }
      }
      if (hit < 0) {
        activeComps[static_cast<std::size_t>(bestB)].count = 0;
        continue;
      }
      for (int cur = hit; cur >= 0; cur = parent[static_cast<std::size_t>(cur)]) {
        if (active[static_cast<std::size_t>(cur)] == 0) {
          active[static_cast<std::size_t>(cur)] = 1;
          ++activeNodes;
        }
        activeComp[static_cast<std::size_t>(cur)] = bestA;
      }
      ++postBridges;
      ActiveCompInfo& keep = activeComps[static_cast<std::size_t>(bestA)];
      const ActiveCompInfo& merged = activeComps[static_cast<std::size_t>(bestB)];
      keep.latSum += merged.latSum;
      keep.lonSum += merged.lonSum;
      keep.count += merged.count;
      activeComps[static_cast<std::size_t>(bestB)].count = 0;
      for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (activeComp[i] == bestB) {
          activeComp[i] = bestA;
        }
      }
    }
    int finalActiveComp = 0;
    for (const auto& ci : activeComps) {
      if (ci.count > 0) {
        ++finalActiveComp;
      }
    }
    std::cerr << "[hwy_csr] post_bridges=" << postBridges << " active_components=" << finalActiveComp
              << " active_nodes=" << activeNodes << "\n";
  }

  // Overlay-aware merge: active road graph may be "merged" while CSR overlay stays disconnected.
  struct SynBridge {
    int a = -1;
    int b = -1;
    float len = 0.0f;
  };
  std::vector<SynBridge> syntheticBridges;
  {
    std::vector<std::vector<int>> overlayAdj;
    std::vector<char> inOverlay;
    buildOverlayAdjacency(in, edgeCount, edgeStart, nodes, active, primaryDegree, &overlayAdj,
                          &inOverlay);
    std::vector<int> overlayComp;
    int overlayComps = countOverlayComponents(overlayAdj, inOverlay, &overlayComp);
    std::cerr << "[hwy_csr] pre_overlay_merge components=" << overlayComps << "\n";

    const double glueSteps[] = {250000.0, 500000.0, 800000.0, 1200000.0};
    std::size_t overlayBridges = 0;
    for (double maxGlueM : glueSteps) {
      for (int attempt = 0; attempt < 200 && overlayComps > 1; ++attempt) {
        std::vector<OverlayCompInfo> ocomps(static_cast<std::size_t>(overlayComps));
        for (std::size_t i = 0; i < nodes.size(); ++i) {
          if (!inOverlay[i] || overlayComp[i] < 0) {
            continue;
          }
          OverlayCompInfo& ci = ocomps[static_cast<std::size_t>(overlayComp[i])];
          ci.latSum += nodeLat[i];
          ci.lonSum += nodeLon[i];
          ++ci.count;
          if (ci.seeds.size() < 32) {
            ci.seeds.push_back(static_cast<int>(i));
          }
        }

        int bestA = -1;
        int bestB = -1;
        double bestDist = 1e100;
        for (int a = 0; a < overlayComps; ++a) {
          if (ocomps[static_cast<std::size_t>(a)].count <= 0) {
            continue;
          }
          const OverlayCompInfo& ca = ocomps[static_cast<std::size_t>(a)];
          const double clat = ca.latSum / std::max(1, ca.count);
          const double clon = ca.lonSum / std::max(1, ca.count);
          for (int b = a + 1; b < overlayComps; ++b) {
            if (ocomps[static_cast<std::size_t>(b)].count <= 0) {
              continue;
            }
            const OverlayCompInfo& cb = ocomps[static_cast<std::size_t>(b)];
            const double d = haversineM(clat, clon, cb.latSum / std::max(1, cb.count),
                                        cb.lonSum / std::max(1, cb.count));
            if (d < bestDist) {
              bestDist = d;
              bestA = a;
              bestB = b;
            }
          }
        }
        if (bestA < 0) {
          break;
        }

        const std::vector<int>& sources = ocomps[static_cast<std::size_t>(bestA)].seeds;
        if (sources.empty()) {
          break;
        }
        std::vector<double> dist(static_cast<std::size_t>(nodeCount), 1e100);
        std::vector<int> parent(static_cast<std::size_t>(nodeCount), -1);
        using DistNode = std::pair<double, int>;
        std::priority_queue<DistNode, std::vector<DistNode>, std::greater<DistNode>> pq;
        for (int seed : sources) {
          dist[static_cast<std::size_t>(seed)] = 0.0;
          pq.push({0.0, seed});
        }
        int hit = -1;
        while (!pq.empty()) {
          const auto [d, u] = pq.top();
          pq.pop();
          if (d > dist[static_cast<std::size_t>(u)] + 1e-6) {
            continue;
          }
          if (inOverlay[static_cast<std::size_t>(u)] &&
              overlayComp[static_cast<std::size_t>(u)] == bestB) {
            hit = u;
            break;
          }
          if (d > maxGlueM + 1e-6) {
            continue;
          }
          for (const RoadEdge& edge : roadAdj[static_cast<std::size_t>(u)]) {
            const double nd = d + edge.len;
            if (nd > maxGlueM + 1e-6) {
              continue;
            }
            if (nd + 1e-6 < dist[static_cast<std::size_t>(edge.to)]) {
              dist[static_cast<std::size_t>(edge.to)] = nd;
              parent[static_cast<std::size_t>(edge.to)] = u;
              pq.push({nd, edge.to});
            }
          }
        }
        if (hit < 0) {
          break;
        }
        for (int cur = hit; cur >= 0; cur = parent[static_cast<std::size_t>(cur)]) {
          if (active[static_cast<std::size_t>(cur)] == 0) {
            active[static_cast<std::size_t>(cur)] = 1;
            ++activeNodes;
          }
        }
        ++overlayBridges;
        buildOverlayAdjacency(in, edgeCount, edgeStart, nodes, active, primaryDegree, &overlayAdj,
                              &inOverlay);
        overlayComps = countOverlayComponents(overlayAdj, inOverlay, &overlayComp);
      }
    }
    std::cerr << "[hwy_csr] overlay_bridges=" << overlayBridges << " overlay_components="
              << overlayComps << " active_nodes=" << activeNodes << "\n";

    if (overlayComps > 1) {
      std::vector<std::vector<int>> compNodes(static_cast<std::size_t>(overlayComps));
      for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (!inOverlay[i] || overlayComp[i] < 0) {
          continue;
        }
        compNodes[static_cast<std::size_t>(overlayComp[i])].push_back(static_cast<int>(i));
      }
      int mainComp = 0;
      std::size_t mainCount = 0;
      for (int c = 0; c < overlayComps; ++c) {
        if (compNodes[static_cast<std::size_t>(c)].size() > mainCount) {
          mainCount = compNodes[static_cast<std::size_t>(c)].size();
          mainComp = c;
        }
      }
      auto sampleNodes = [](const std::vector<int>& nodes, std::size_t maxN) {
        if (nodes.size() <= maxN) {
          return nodes;
        }
        std::vector<int> out;
        out.reserve(maxN);
        const std::size_t step = std::max<std::size_t>(1, nodes.size() / maxN);
        for (std::size_t i = 0; i < nodes.size() && out.size() < maxN; i += step) {
          out.push_back(nodes[i]);
        }
        return out;
      };
      const std::vector<int> mainNodes =
          sampleNodes(compNodes[static_cast<std::size_t>(mainComp)], 128);
      for (int c = 0; c < overlayComps; ++c) {
        if (c == mainComp) {
          continue;
        }
        const std::vector<int> otherNodes =
            sampleNodes(compNodes[static_cast<std::size_t>(c)], 128);
        int bestU = -1;
        int bestV = -1;
        double bestD = 1e100;
        for (int u : mainNodes) {
          for (int v : otherNodes) {
            const double d =
                haversineM(nodeLat[static_cast<std::size_t>(u)], nodeLon[static_cast<std::size_t>(u)],
                           nodeLat[static_cast<std::size_t>(v)], nodeLon[static_cast<std::size_t>(v)]);
            if (d < bestD) {
              bestD = d;
              bestU = u;
              bestV = v;
            }
          }
        }
        if (bestU >= 0 && bestV >= 0) {
          syntheticBridges.push_back(
              {bestU, bestV, static_cast<float>(std::max(100.0, bestD))});
          std::cerr << "[hwy_csr] synthetic_bridge comp" << mainComp << "->" << c
                    << " dist_m=" << bestD << "\n";
        }
      }
    }
  }

  std::vector<uint32_t> degree(static_cast<std::size_t>(nodeCount), 0);
  in.seekg(static_cast<std::streamoff>(edgeStart));
  std::cerr << "[hwy_csr] pass2 connector degree\n";
  uint64_t connectorArcs = 0;
  for (uint64_t i = 0; i < edgeCount; ++i) {
    int64_t id = 0;
    int64_t fr = 0;
    int64_t to = 0;
    int32_t type = 0;
    double length = 0.0;
    double speed = 0.0;
    if (!in.read(reinterpret_cast<char*>(&id), 8) ||
        !in.read(reinterpret_cast<char*>(&fr), 8) || !in.read(reinterpret_cast<char*>(&to), 8) ||
        !in.read(reinterpret_cast<char*>(&type), 4) ||
        !in.read(reinterpret_cast<char*>(&length), 8) ||
        !in.read(reinterpret_cast<char*>(&speed), 8)) {
      break;
    }
    const int frRow = nodeRowOf(nodes, fr);
    const int toRow = nodeRowOf(nodes, to);
    if (!includeInOverlay(type, speed, length, frRow, toRow, active, primaryDegree)) {
      continue;
    }
    if (!isPrimaryHighwayEdge(type, speed, length)) {
      connectorArcs += (frRow >= 0) + (toRow >= 0);
    }
    if (frRow >= 0) {
      ++degree[static_cast<std::size_t>(frRow)];
    }
    if (toRow >= 0) {
      ++degree[static_cast<std::size_t>(toRow)];
    }
  }
  for (const SynBridge& sb : syntheticBridges) {
    ++degree[static_cast<std::size_t>(sb.a)];
    ++degree[static_cast<std::size_t>(sb.b)];
    connectorArcs += 2;
  }

  uint64_t arcCount = 0;
  for (uint32_t d : degree) {
    arcCount += d;
  }
  std::cerr << "[hwy_csr] arcs=" << arcCount << " connector_half_arcs=" << connectorArcs << "\n";

  const std::size_t kHeader = 28;
  const std::size_t rowBytes = (static_cast<std::size_t>(nodeCount) + 1) * sizeof(uint64_t);
  const std::size_t arcBytes = static_cast<std::size_t>(arcCount) * sizeof(CsrArcRec);
  const std::size_t totalSize = kHeader + rowBytes + arcBytes;

  std::vector<char> file(totalSize, 0);
  std::memcpy(file.data(), kCsrMagic, 8);
  const uint32_t csrVer = 1;
  std::memcpy(file.data() + 8, &csrVer, 4);
  std::memcpy(file.data() + 12, &nodeCount, 8);
  std::memcpy(file.data() + 20, &arcCount, 8);

  auto* rowPtr = reinterpret_cast<uint64_t*>(file.data() + kHeader);
  auto* arcs = reinterpret_cast<CsrArcRec*>(file.data() + kHeader + rowBytes);
  rowPtr[0] = 0;
  for (uint64_t i = 0; i < nodeCount; ++i) {
    rowPtr[i + 1] = rowPtr[i] + degree[static_cast<std::size_t>(i)];
  }
  std::vector<uint64_t> next(static_cast<std::size_t>(nodeCount));
  for (uint64_t i = 0; i < nodeCount; ++i) {
    next[static_cast<std::size_t>(i)] = rowPtr[i];
  }

  in.seekg(static_cast<std::streamoff>(edgeStart));
  std::cerr << "[hwy_csr] pass3 fill\n";
  for (uint64_t i = 0; i < edgeCount; ++i) {
    int64_t id = 0;
    int64_t fr = 0;
    int64_t to = 0;
    int32_t type = 0;
    double length = 0.0;
    double speed = 0.0;
    if (!in.read(reinterpret_cast<char*>(&id), 8) ||
        !in.read(reinterpret_cast<char*>(&fr), 8) || !in.read(reinterpret_cast<char*>(&to), 8) ||
        !in.read(reinterpret_cast<char*>(&type), 4) ||
        !in.read(reinterpret_cast<char*>(&length), 8) ||
        !in.read(reinterpret_cast<char*>(&speed), 8)) {
      break;
    }
    const int frRow = nodeRowOf(nodes, fr);
    const int toRow = nodeRowOf(nodes, to);
    if (!includeInOverlay(type, speed, length, frRow, toRow, active, primaryDegree)) {
      continue;
    }
    CsrArcRec outArc;
    outArc.edgeId = id;
    outArc.edgeType = type;
    outArc.length = static_cast<float>(length);
    outArc.speedLimit = static_cast<float>(speed);
    if (frRow >= 0) {
      const std::size_t slot = static_cast<std::size_t>(next[static_cast<std::size_t>(frRow)]++);
      outArc.toNodeId = to;
      arcs[slot] = outArc;
    }
    if (toRow >= 0) {
      const std::size_t slot = static_cast<std::size_t>(next[static_cast<std::size_t>(toRow)]++);
      outArc.toNodeId = fr;
      arcs[slot] = outArc;
    }
  }
  for (const SynBridge& sb : syntheticBridges) {
    CsrArcRec outArc;
    outArc.edgeId = -1;
    outArc.edgeType = 0;
    outArc.length = sb.len;
    outArc.speedLimit = 60.0f;
    {
      const std::size_t slot = static_cast<std::size_t>(next[static_cast<std::size_t>(sb.a)]++);
      outArc.toNodeId = nodes[static_cast<std::size_t>(sb.b)].id;
      arcs[slot] = outArc;
    }
    {
      const std::size_t slot = static_cast<std::size_t>(next[static_cast<std::size_t>(sb.b)]++);
      outArc.toNodeId = nodes[static_cast<std::size_t>(sb.a)].id;
      arcs[slot] = outArc;
    }
  }

  {
    std::vector<int> overlayComp(static_cast<std::size_t>(nodeCount), -1);
    int overlayComps = 0;
    for (uint64_t i = 0; i < nodeCount; ++i) {
      if (degree[static_cast<std::size_t>(i)] == 0 || overlayComp[static_cast<std::size_t>(i)] >= 0) {
        continue;
      }
      std::deque<int> q;
      q.push_back(static_cast<int>(i));
      overlayComp[static_cast<std::size_t>(i)] = overlayComps;
      while (!q.empty()) {
        const int u = q.front();
        q.pop_front();
        const uint64_t begin = rowPtr[static_cast<std::size_t>(u)];
        const uint64_t end = rowPtr[static_cast<std::size_t>(u) + 1];
        for (uint64_t slot = begin; slot < end; ++slot) {
          const int64_t v = arcs[slot].toNodeId;
          const int vRow = nodeRowOf(nodes, v);
          if (vRow < 0 || overlayComp[static_cast<std::size_t>(vRow)] >= 0) {
            continue;
          }
          overlayComp[static_cast<std::size_t>(vRow)] = overlayComps;
          q.push_back(vRow);
        }
      }
      ++overlayComps;
    }
    std::cerr << "[hwy_csr] overlay_components=" << overlayComps << "\n";
    if (overlayComps > 1) {
      std::cerr << "[hwy_csr] WARNING: overlay graph is disconnected (" << overlayComps
                << " components)\n";
    }
  }

  std::ofstream out(csrPath, std::ios::binary);
  out.write(file.data(), static_cast<std::streamsize>(file.size()));
  std::cerr << "[hwy_csr] wrote " << csrPath << " (" << (totalSize / 1e6) << " MB)\n";
  return 0;
}
