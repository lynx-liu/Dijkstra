// Build partition tile edge cache: china.mmlp.rtidx (+ embedded edge ids)
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr char kSidxMagic[8] = {'M', 'M', 'L', 'P', 'S', 'I', 'D', 'X'};
constexpr char kOutMagic[8] = {'M', 'M', 'L', 'P', 'R', 'T', 'I', 'X'};
constexpr double kTileSize = 0.25;

#pragma pack(push, 1)
struct TileEntry {
  int32_t tx = 0;
  int32_t ty = 0;
  uint64_t offset = 0;
  uint32_t count = 0;
};
#pragma pack(pop)

int tileOf(int cellIndex, double cellSizeDeg) {
  return static_cast<int>(std::floor((static_cast<double>(cellIndex) * cellSizeDeg) / kTileSize));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: mmlp_build_rtiles <china.mmlp.bin>\n";
    return 1;
  }
  const std::string base = std::string(argv[1]).substr(0, std::string(argv[1]).size() - 4);
  const std::string sidxPath = base + ".sidx";
  const std::string outPath = base + ".rtidx";

  std::ifstream in(sidxPath, std::ios::binary);
  if (!in) {
    std::cerr << "cannot open " << sidxPath << "\n";
    return 1;
  }
  char magic[8] = {};
  uint32_t version = 0;
  double cellSize = 0.02;
  uint32_t cellCount = 0;
  if (!in.read(magic, 8) || std::memcmp(magic, kSidxMagic, 8) != 0 ||
      !in.read(reinterpret_cast<char*>(&version), 4) ||
      !in.read(reinterpret_cast<char*>(&cellSize), 8) ||
      !in.read(reinterpret_cast<char*>(&cellCount), 4)) {
    std::cerr << "invalid sidx header\n";
    return 1;
  }
  if (version != 1 || cellSize <= 0.0) {
    std::cerr << "unsupported sidx version/cell size\n";
    return 1;
  }

  std::unordered_map<int64_t, std::unordered_set<int64_t>> tiles;
  tiles.reserve(80000);
  std::cerr << "[rtiles] scan " << cellCount << " cells (cellSize=" << cellSize << ")\n";

  for (uint32_t c = 0; c < cellCount; ++c) {
    int32_t gx = 0;
    int32_t gy = 0;
    uint64_t n = 0;
    if (!in.read(reinterpret_cast<char*>(&gx), 4) ||
        !in.read(reinterpret_cast<char*>(&gy), 4) ||
        !in.read(reinterpret_cast<char*>(&n), 8)) {
      std::cerr << "truncated sidx at cell " << c << "\n";
      return 1;
    }
    const int tx = tileOf(gx, cellSize);
    const int ty = tileOf(gy, cellSize);
    const int64_t key =
        (static_cast<int64_t>(tx) << 32) | static_cast<uint32_t>(ty);
    auto& bucket = tiles[key];
    for (uint64_t i = 0; i < n; ++i) {
      int64_t eid = 0;
      if (!in.read(reinterpret_cast<char*>(&eid), 8)) {
        std::cerr << "truncated sidx edge list at cell " << c << "\n";
        return 1;
      }
      bucket.insert(eid);
    }
    if (c > 0 && c % 200000 == 0) {
      std::cerr << "[rtiles] " << (100 * c / cellCount) << "%\n";
    }
  }

  std::vector<std::pair<int64_t, std::unordered_set<int64_t>>> ordered;
  ordered.reserve(tiles.size());
  for (auto& kv : tiles) {
    ordered.push_back(std::move(kv));
  }
  std::sort(ordered.begin(), ordered.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  std::vector<TileEntry> entries;
  std::vector<int64_t> edgeData;
  entries.reserve(ordered.size());
  for (const auto& kv : ordered) {
    TileEntry e;
    e.tx = static_cast<int32_t>(kv.first >> 32);
    e.ty = static_cast<int32_t>(kv.first & 0xffffffffu);
    e.offset = edgeData.size();
    for (int64_t eid : kv.second) {
      edgeData.push_back(eid);
    }
    e.count = static_cast<uint32_t>(edgeData.size() - e.offset);
    entries.push_back(e);
  }

  const uint32_t ver = 1;
  const uint64_t nTiles = entries.size();
  std::vector<char> file;
  file.resize(20 + entries.size() * sizeof(TileEntry) + edgeData.size() * sizeof(int64_t));
  std::memcpy(file.data(), kOutMagic, 8);
  std::memcpy(file.data() + 8, &ver, 4);
  std::memcpy(file.data() + 12, &nTiles, 8);
  std::memcpy(file.data() + 20, entries.data(), entries.size() * sizeof(TileEntry));
  std::memcpy(file.data() + 20 + entries.size() * sizeof(TileEntry), edgeData.data(),
              edgeData.size() * sizeof(int64_t));

  std::ofstream out(outPath, std::ios::binary);
  out.write(file.data(), static_cast<std::streamsize>(file.size()));
  std::cerr << "[rtiles] wrote " << outPath << " tiles=" << nTiles
            << " edges=" << edgeData.size() << " (" << (file.size() / 1e6) << " MB)\n";
  return 0;
}
