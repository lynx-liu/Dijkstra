#include "mmlp/rtile_index.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mmlp {

namespace {

constexpr char kMagic[8] = {'M', 'M', 'L', 'P', 'R', 'T', 'I', 'X'};
constexpr double kTileSize = 0.25;

#pragma pack(push, 1)
struct TileEntry {
  int32_t tx = 0;
  int32_t ty = 0;
  uint64_t offset = 0;
  uint32_t count = 0;
};
#pragma pack(pop)

int tileX(double lat) { return static_cast<int>(std::floor(lat / kTileSize)); }
int tileY(double lon) { return static_cast<int>(std::floor(lon / kTileSize)); }

int64_t tileKey(int32_t tx, int32_t ty) {
  return (static_cast<int64_t>(tx) << 32) | static_cast<uint32_t>(ty);
}

}  // namespace

RtileIndex::~RtileIndex() {
  if (data_ != nullptr && size_ > 0) {
    ::munmap(data_, size_);
  }
  data_ = nullptr;
}

bool RtileIndex::open(const std::string& basePath, std::string* error) {
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

  const std::string path = basePath + ".rtidx";
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
  if (size_ < 20) {
    return fail("truncated rtidx");
  }
  if (std::memcmp(data_, kMagic, 8) != 0) {
    return fail("invalid rtidx magic");
  }

  uint32_t version = 0;
  uint64_t count = 0;
  const char* base = static_cast<const char*>(data_);
  std::memcpy(&version, base + 8, 4);
  std::memcpy(&count, base + 12, 8);
  if (version != 1) {
    return fail("unsupported rtidx version");
  }
  tileCount_ = static_cast<std::size_t>(count);
  entries_ = base + 20;
  const std::size_t tableBytes = tileCount_ * sizeof(TileEntry);
  if (size_ < 20 + tableBytes) {
    return fail("truncated rtidx table");
  }
  edgeData_ = reinterpret_cast<const int64_t*>(base + 20 + tableBytes);
  return true;
}

void RtileIndex::collectEdgesInBBox(const GeoBBox& box, std::unordered_set<int64_t>& edgeIds) const {
  if (!isOpen()) {
    return;
  }
  const int x0 = tileX(box.minLat);
  const int x1 = tileX(box.maxLat);
  const int y0 = tileY(box.minLon);
  const int y1 = tileY(box.maxLon);
  const auto* entries = static_cast<const TileEntry*>(entries_);
  for (int tx = x0; tx <= x1; ++tx) {
    const int64_t loKey = tileKey(tx, y0);
    const int64_t hiKey = tileKey(tx, y1);
    const TileEntry* begin = entries;
    const TileEntry* end = entries + tileCount_;
    const TileEntry* it = std::lower_bound(
        begin, end, loKey,
        [](const TileEntry& entry, int64_t key) {
          return tileKey(entry.tx, entry.ty) < key;
        });
    for (; it != end; ++it) {
      const int64_t key = tileKey(it->tx, it->ty);
      if (key > hiKey) {
        break;
      }
      for (uint32_t j = 0; j < it->count; ++j) {
        edgeIds.insert(edgeData_[it->offset + j]);
      }
    }
  }
}

}  // namespace mmlp
