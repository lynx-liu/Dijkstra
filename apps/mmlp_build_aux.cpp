#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr char kBinMagic[] = "MMLPGRPH";
constexpr char kNdxMagic[8] = {'M', 'M', 'L', 'P', 'N', 'D', 'X', '\0'};
constexpr char kSidxMagic[8] = {'M', 'M', 'L', 'P', 'S', 'I', 'D', 'X'};
constexpr std::size_t kNodeRecord = 28;
constexpr std::size_t kEdgeRecord = 44;
constexpr double kCellSize = 0.02;

struct NodeRow {
  int64_t id = 0;
  uint64_t offset = 0;
  double lat = 0.0;
  double lon = 0.0;
};

struct IdOff {
  int64_t id = 0;
  uint64_t off = 0;
};

#pragma pack(push, 1)
struct CellRec {
  int32_t gx = 0;
  int32_t gy = 0;
  int64_t eid = 0;
};
#pragma pack(pop)
static_assert(sizeof(CellRec) == 16, "CellRec layout");

int cellGx(double lat) { return static_cast<int>(std::floor(lat / kCellSize)); }
int cellGy(double lon) { return static_cast<int>(std::floor(lon / kCellSize)); }

const NodeRow* findNode(const std::vector<NodeRow>& nodes, int64_t id) {
  auto it = std::lower_bound(nodes.begin(), nodes.end(), id,
                             [](const NodeRow& r, int64_t k) { return r.id < k; });
  if (it == nodes.end() || it->id != id) {
    return nullptr;
  }
  return &*it;
}

void writeNdx(const std::string& path, const std::vector<IdOff>& rows) {
  std::ofstream out(path, std::ios::binary);
  out.write(kNdxMagic, 8);
  const uint32_t ver = 1;
  const uint64_t n = rows.size();
  out.write(reinterpret_cast<const char*>(&ver), 4);
  out.write(reinterpret_cast<const char*>(&n), 8);
  for (const auto& r : rows) {
    out.write(reinterpret_cast<const char*>(&r.id), 8);
    out.write(reinterpret_cast<const char*>(&r.off), 8);
  }
}

void writeSidxFromSortedCells(const std::string& path, const std::vector<CellRec>& sorted) {
  std::ofstream out(path, std::ios::binary);
  out.write(kSidxMagic, 8);
  const uint32_t ver = 1;
  const double cellSize = kCellSize;
  uint32_t cellCount = 0;
  const auto pos0 = out.tellp();
  out.write(reinterpret_cast<const char*>(&ver), 4);
  out.write(reinterpret_cast<const char*>(&cellSize), 8);
  out.write(reinterpret_cast<const char*>(&cellCount), 4);

  std::size_t i = 0;
  while (i < sorted.size()) {
    const int32_t gx = sorted[i].gx;
    const int32_t gy = sorted[i].gy;
    std::size_t j = i + 1;
    while (j < sorted.size() && sorted[j].gx == gx && sorted[j].gy == gy) {
      ++j;
    }
    const uint64_t n = j - i;
    out.write(reinterpret_cast<const char*>(&gx), 4);
    out.write(reinterpret_cast<const char*>(&gy), 4);
    out.write(reinterpret_cast<const char*>(&n), 8);
    for (std::size_t k = i; k < j; ++k) {
      out.write(reinterpret_cast<const char*>(&sorted[k].eid), 8);
    }
    ++cellCount;
    i = j;
  }

  out.seekp(pos0 + static_cast<std::streamoff>(sizeof(ver) + sizeof(cellSize)));
  out.write(reinterpret_cast<const char*>(&cellCount), 4);
}

int shardOf(int gx, int gy) {
  const uint32_t ux = static_cast<uint32_t>(gx);
  const uint32_t uy = static_cast<uint32_t>(gy);
  return static_cast<int>((ux * 1315423911u ^ uy * 2654435761u) % 256);
}

bool buildSidxFromShards(const std::string& basePath, const std::string& sidxPath) {
  constexpr int kShards = 256;
  std::vector<std::string> shardPaths;
  shardPaths.reserve(kShards);
  for (int s = 0; s < kShards; ++s) {
    shardPaths.push_back(basePath + ".shard" + std::to_string(s));
  }

  std::ofstream out(sidxPath, std::ios::binary);
  out.write(kSidxMagic, 8);
  const uint32_t ver = 1;
  const double cellSize = kCellSize;
  uint32_t totalCells = 0;
  const auto countPos = out.tellp();
  out.write(reinterpret_cast<const char*>(&ver), 4);
  out.write(reinterpret_cast<const char*>(&cellSize), 8);
  out.write(reinterpret_cast<const char*>(&totalCells), 4);

  std::vector<CellRec> recs;
  for (int s = 0; s < kShards; ++s) {
    std::ifstream in(shardPaths[static_cast<std::size_t>(s)], std::ios::binary | std::ios::ate);
    if (!in) {
      continue;
    }
    const auto bytes = in.tellg();
    if (bytes <= 0) {
      std::remove(shardPaths[static_cast<std::size_t>(s)].c_str());
      continue;
    }
    in.seekg(0);
    const std::size_t n = static_cast<std::size_t>(bytes) / sizeof(CellRec);
    recs.resize(n);
    in.read(reinterpret_cast<char*>(recs.data()),
            static_cast<std::streamsize>(n * sizeof(CellRec)));
    std::remove(shardPaths[static_cast<std::size_t>(s)].c_str());
    if (recs.empty()) {
      continue;
    }
    std::sort(recs.begin(), recs.end(), [](const CellRec& a, const CellRec& b) {
      if (a.gx != b.gx) {
        return a.gx < b.gx;
      }
      return a.gy < b.gy;
    });
    std::size_t i = 0;
    while (i < recs.size()) {
      const int32_t gx = recs[i].gx;
      const int32_t gy = recs[i].gy;
      std::size_t j = i + 1;
      while (j < recs.size() && recs[j].gx == gx && recs[j].gy == gy) {
        ++j;
      }
      const uint64_t cnt = j - i;
      out.write(reinterpret_cast<const char*>(&gx), 4);
      out.write(reinterpret_cast<const char*>(&gy), 4);
      out.write(reinterpret_cast<const char*>(&cnt), 8);
      for (std::size_t k = i; k < j; ++k) {
        out.write(reinterpret_cast<const char*>(&recs[k].eid), 8);
      }
      ++totalCells;
      i = j;
    }
    if (s % 32 == 0) {
      std::cerr << "[build_aux] sidx shard " << s << "/" << kShards << "\n";
    }
  }

  out.seekp(countPos + static_cast<std::streamoff>(sizeof(ver) + sizeof(cellSize)));
  out.write(reinterpret_cast<const char*>(&totalCells), 4);
  return static_cast<bool>(out);
}

}  // namespace

int main(int argc, char** argv) {
  const std::string binPath = (argc > 1) ? argv[1] : "data/graph/china.mmlp.bin";
  bool skipNidx = false;
  bool nidxOnly = false;
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--skip-nidx") {
      skipNidx = true;
    } else if (arg == "--nidx-only") {
      nidxOnly = true;
      skipNidx = false;
    }
  }
  const auto dot = binPath.rfind('.');
  const std::string base = (dot == std::string::npos) ? binPath : binPath.substr(0, dot);
  const std::string nidxPath = base + ".nidx";
  const std::string eidxPath = base + ".eidx";
  const std::string sidxPath = base + ".sidx";

  std::ifstream in(binPath, std::ios::binary);
  if (!in) {
    std::cerr << "cannot open " << binPath << "\n";
    return 1;
  }

  char magic[8];
  uint32_t version = 0;
  uint64_t nodeCount = 0;
  uint64_t edgeCount = 0;
  if (!in.read(magic, 8) || std::string(magic, 8) != kBinMagic ||
      !in.read(reinterpret_cast<char*>(&version), 4) ||
      !in.read(reinterpret_cast<char*>(&nodeCount), 8) ||
      !in.read(reinterpret_cast<char*>(&edgeCount), 8) || version != 1) {
    std::cerr << "invalid graph\n";
    return 1;
  }

  std::cerr << "[build_aux] nodes=" << nodeCount << " edges=" << edgeCount << "\n";

  std::vector<NodeRow> nodes;
  nodes.reserve(static_cast<std::size_t>(nodeCount));
  for (uint64_t i = 0; i < nodeCount; ++i) {
    const auto off = static_cast<uint64_t>(in.tellg());
    int64_t id = 0;
    double lat = 0.0;
    double lon = 0.0;
    int32_t kind = 0;
    if (!in.read(reinterpret_cast<char*>(&id), 8) || !in.read(reinterpret_cast<char*>(&lat), 8) ||
        !in.read(reinterpret_cast<char*>(&lon), 8) ||
        !in.read(reinterpret_cast<char*>(&kind), 4)) {
      return 1;
    }
    nodes.push_back({id, off, lat, lon});
    if (i > 0 && i % 10000000 == 0) {
      std::cerr << "[build_aux] nodes " << (100 * i / nodeCount) << "%\n";
    }
  }

  if (!skipNidx) {
    std::cerr << "[build_aux] writing nidx\n";
    std::sort(nodes.begin(), nodes.end(),
              [](const NodeRow& a, const NodeRow& b) { return a.id < b.id; });
    std::vector<IdOff> nidx;
    nidx.reserve(nodes.size());
    for (const auto& n : nodes) {
      nidx.push_back({n.id, n.offset});
    }
    writeNdx(nidxPath, nidx);
  } else {
    std::cerr << "[build_aux] skip nidx (existing)\n";
    std::sort(nodes.begin(), nodes.end(),
              [](const NodeRow& a, const NodeRow& b) { return a.id < b.id; });
  }

  if (nidxOnly) {
    std::cerr << "[build_aux] nidx-only done\n";
    return 0;
  }

  constexpr int kShards = 256;
  std::vector<std::ofstream> shards(static_cast<std::size_t>(kShards));
  for (int s = 0; s < kShards; ++s) {
    shards[static_cast<std::size_t>(s)].open(base + ".shard" + std::to_string(s),
                                              std::ios::binary | std::ios::trunc);
  }

  std::vector<IdOff> eidx;
  eidx.reserve(static_cast<std::size_t>(edgeCount));

  std::cerr << "[build_aux] scanning edges -> shards + eidx\n";
  for (uint64_t i = 0; i < edgeCount; ++i) {
    const auto off = static_cast<uint64_t>(in.tellg());
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
      return 1;
    }
    eidx.push_back({id, off});

    const NodeRow* a = findNode(nodes, fr);
    const NodeRow* b = findNode(nodes, to);
    if (a != nullptr && b != nullptr) {
      CellRec rec;
      rec.eid = id;
      rec.gx = cellGx(a->lat);
      rec.gy = cellGy(a->lon);
      shards[static_cast<std::size_t>(shardOf(rec.gx, rec.gy))].write(
          reinterpret_cast<const char*>(&rec), sizeof(rec));
      rec.gx = cellGx(b->lat);
      rec.gy = cellGy(b->lon);
      shards[static_cast<std::size_t>(shardOf(rec.gx, rec.gy))].write(
          reinterpret_cast<const char*>(&rec), sizeof(rec));
      rec.gx = cellGx(0.5 * (a->lat + b->lat));
      rec.gy = cellGy(0.5 * (a->lon + b->lon));
      shards[static_cast<std::size_t>(shardOf(rec.gx, rec.gy))].write(
          reinterpret_cast<const char*>(&rec), sizeof(rec));
    }

    if (i > 0 && i % 5000000 == 0) {
      std::cerr << "[build_aux] edges " << (100 * i / edgeCount) << "%\n";
    }
  }
  for (auto& sh : shards) {
    sh.close();
  }
  nodes.clear();
  nodes.shrink_to_fit();

  std::cerr << "[build_aux] writing eidx\n";
  std::sort(eidx.begin(), eidx.end(),
            [](const IdOff& a, const IdOff& b) { return a.id < b.id; });
  writeNdx(eidxPath, eidx);
  eidx.clear();

  std::cerr << "[build_aux] building sidx from shards\n";
  if (!buildSidxFromShards(base, sidxPath)) {
    return 1;
  }

  std::cerr << "[build_aux] done\n";
  return 0;
}
