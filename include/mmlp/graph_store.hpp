#pragma once

#include "mmlp/graph.hpp"
#include "mmlp/geo.hpp"

#include "mmlp/ch_graph.hpp"
#include "mmlp/csr_graph.hpp"
#include "mmlp/full_ch_graph.hpp"

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace mmlp {

struct CorridorSegment {
  double aLat = 0.0;
  double aLon = 0.0;
  double bLat = 0.0;
  double bLon = 0.0;
  double widthM = 0.0;
};

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
  // Endpoint lat/lon from mmap .egeo (O(1) after eidx lookup). Falls back to nodeLatLon.
  bool edgeEndpointLatLon(int64_t edgeId, double& flat, double& flon, double& tlat,
                        double& tlon) const;
  bool hasEdgeGeo() const { return egeo_.data != nullptr; }
  bool hasCsr() const { return csr_.isOpen(); }
  const CsrGraph& csr() const { return csr_; }
  bool hasHwyCsr() const { return hwyCsr_.isOpen(); }
  const CsrGraph& hwyCsr() const { return hwyCsr_; }
  bool hasCh() const { return ch_.isOpen(); }
  const ChGraph& ch() const { return ch_; }
  bool hasFullCh() const { return fullCh_.isOpen(); }
  const FullChGraph& fullCh() const { return fullCh_; }

  // Prefault CSR mmap into page cache (async, uses thread pool).
  void warmMappedRoutingFilesAsync() const;
  // Blocking page-in of CSR / full.ch (use at startup preload so first query is hot).
  void warmMappedRoutingFiles() const;

  // Row in nidx / CSR tables for a node id (-1 if missing).
  int nodeRowIndex(int64_t nodeId) const;

  bool readEdge(int64_t edgeId, EdgeType& type, double& length, double& speedLimit) const;

  bool loadGraphSubset(const std::unordered_set<int64_t>& edgeIds, MultimodalGraph& graph,
                       std::string* error = nullptr) const;

  // One mmap pass: keep edges near vehicle→destination corridors (parallel filter+load).
  bool loadGraphSubsetNearCorridors(const std::unordered_set<int64_t>& candidates,
                                    const std::vector<CorridorSegment>& corridors,
                                    const std::vector<LatLon>& anchorPoints,
                                    double anchorRadiusM, MultimodalGraph& graph,
                                    std::string* error = nullptr) const;

  bool readEdgeRecord(int64_t edgeId, Edge& edge) const;
  bool materializeGraphFromEdges(std::vector<Edge>&& edges, MultimodalGraph& graph,
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
  MmapFile egeo_;
  const float* edgeGeo_ = nullptr;
  std::size_t edgeGeoCount_ = 0;
  CsrGraph csr_;
  CsrGraph hwyCsr_;
  ChGraph ch_;
  FullChGraph fullCh_;
  const IdOffset* nodeIndex_ = nullptr;
  std::size_t nodeCount_ = 0;
  const IdOffset* edgeIndex_ = nullptr;
  std::size_t edgeCount_ = 0;
  uint64_t edgeRegionOffset_ = 0;
};

}  // namespace mmlp
