#pragma once

#include "mmlp/graph.hpp"

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace mmlp {

// mmap-backed nationwide graph accessor (index mode). Loaded once at startup.
class GraphFileStore {
 public:
  GraphFileStore() = default;
  ~GraphFileStore();

  GraphFileStore(const GraphFileStore&) = delete;
  GraphFileStore& operator=(const GraphFileStore&) = delete;

  bool open(const std::string& binPath, std::string* error = nullptr);
  bool isOpen() const { return bin_.data != nullptr; }

  bool nodeLatLon(int64_t nodeId, double& lat, double& lon) const;
  bool edgeEndpoints(int64_t edgeId, int64_t& from, int64_t& to) const;

  bool loadGraphSubset(const std::unordered_set<int64_t>& edgeIds, MultimodalGraph& graph,
                       std::string* error = nullptr) const;

 private:
  struct IdOffset {
    int64_t id = 0;
    uint64_t offset = 0;
  };

  struct MmapFile {
    void* data = nullptr;
    std::size_t size = 0;
    bool map(const std::string& path, std::string* error);
    void unmap();
  };

  static const IdOffset* findOffset(const IdOffset* table, std::size_t count, int64_t id);

  std::string binPath_;
  MmapFile bin_;
  MmapFile nidx_;
  MmapFile eidx_;
  const IdOffset* nodeIndex_ = nullptr;
  std::size_t nodeCount_ = 0;
  const IdOffset* edgeIndex_ = nullptr;
  std::size_t edgeCount_ = 0;
  uint64_t edgeRegionOffset_ = 0;
};

}  // namespace mmlp
