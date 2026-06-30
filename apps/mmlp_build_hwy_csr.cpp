// Build highway (arterial) CSR overlay: china.mmlp.hwy.csr
// Filter: ROAD edges with speedLimit >= 60 km/h.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr char kBinMagic[] = "MMLPGRPH";
constexpr char kCsrMagic[8] = {'M', 'M', 'L', 'P', 'C', 'S', 'R', '\0'};
constexpr std::size_t kNodeRecord = 28;
constexpr std::size_t kEdgeRecord = 44;
constexpr double kMinSpeedKmh = 40.0;

struct NodeRow {
  int64_t id = 0;
  uint64_t offset = 0;
};

struct IdOff {
  int64_t id = 0;
  uint64_t off = 0;
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

bool isHighwayEdge(int32_t type, double speedLimit, double length) {
  if (type != 0) {
    return false;
  }
  if (speedLimit >= kMinSpeedKmh) {
    return true;
  }
  // speedLimit==0 means "use default" in routing; keep longer arterials for connectivity.
  return speedLimit <= 0.0 && length >= 400.0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: mmlp_build_hwy_csr <china.mmlp.bin>\n";
    return 1;
  }
  const std::string binPath = argv[1];
  const std::string base = binPath.substr(0, binPath.size() - 4);
  const std::string nidxPath = base + ".nidx";
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
    nodes[i].offset = 28 + i * kNodeRecord;
  }
  std::sort(nodes.begin(), nodes.end(),
            [](const NodeRow& a, const NodeRow& b) { return a.id < b.id; });

  const auto edgeStart = 28 + nodeCount * kNodeRecord;
  in.seekg(static_cast<std::streamoff>(edgeStart));

  std::vector<uint32_t> degree(static_cast<std::size_t>(nodeCount), 0);
  std::cerr << "[hwy_csr] pass1 degree\n";
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
    if (!isHighwayEdge(type, speed, length)) {
      continue;
    }
    const int frRow = nodeRowOf(nodes, fr);
    const int toRow = nodeRowOf(nodes, to);
    if (frRow >= 0) {
      ++degree[static_cast<std::size_t>(frRow)];
    }
    if (toRow >= 0) {
      ++degree[static_cast<std::size_t>(toRow)];
    }
  }

  uint64_t arcCount = 0;
  for (uint32_t d : degree) {
    arcCount += d;
  }
  std::cerr << "[hwy_csr] arcs=" << arcCount << "\n";

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
  std::cerr << "[hwy_csr] pass2 fill\n";
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
    if (!isHighwayEdge(type, speed, length)) {
      continue;
    }
    const int frRow = nodeRowOf(nodes, fr);
    const int toRow = nodeRowOf(nodes, to);
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

  std::ofstream out(csrPath, std::ios::binary);
  out.write(file.data(), static_cast<std::streamsize>(file.size()));
  std::cerr << "[hwy_csr] wrote " << csrPath << " (" << (totalSize / 1e6) << " MB)\n";
  (void)nidxPath;
  return 0;
}
