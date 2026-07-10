#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mmlp {

// mmap full-graph Contraction Hierarchies (road network of a regional graph).
// Built offline by mmlp_build_full_ch; file format MMLPCH03. Unlike the hwy
// overlay CH this covers every road node, and shortcuts carry child arcs so a
// query unpacks into the exact original edge sequence (for polylines and exact
// vehicle-speed ETA). Queries use sparse hash maps: no O(n) allocation per call.
class FullChGraph {
 public:
  struct Seed {
    int64_t nodeId = 0;
    double costSec = 0.0;  // initial offset (e.g. partial edge from snap point)
  };

  struct PathArc {
    int64_t edgeId = 0;
    float lengthM = 0.0f;
    float speedLimitKmh = 0.0f;
    int64_t fromNodeId = 0;
    int64_t toNodeId = 0;
  };

  struct PathResult {
    bool found = false;
    bool capped = false;  // settle cap hit before search finished: result unknown
    double profileSec = 0.0;  // build-profile weight incl. seed offsets
    int64_t startNodeId = 0;
    int64_t endNodeId = 0;
    std::size_t settledNodes = 0;
    std::vector<PathArc> arcs;  // ordered start -> end, original edges only
  };

  FullChGraph() = default;
  ~FullChGraph();

  FullChGraph(const FullChGraph&) = delete;
  FullChGraph& operator=(const FullChGraph&) = delete;

  bool open(const std::string& path, std::string* error = nullptr);
  bool isOpen() const { return data_ != nullptr; }

  const void* mappedData() const { return data_; }
  std::size_t mappedSize() const { return size_; }

  std::size_t nodeCount() const { return nodeCount_; }
  std::size_t arcCount() const { return arcCount_; }
  double profileKmh() const { return profileKmh_; }

  // Global node id -> compact index (-1 if not a road node of this graph).
  int nodeIndex(int64_t nodeId) const;

  // Multi-source/multi-target bidirectional CH search + full unpack.
  // settleCap bounds worst-case latency (isolated islands would otherwise scan
  // the whole up-graph); reachable queries settle far fewer nodes than the cap.
  // maxWallMs (0 = off) aborts the search early for interactive corridor hops.
  PathResult route(const std::vector<Seed>& from, const std::vector<Seed>& to, double maxSec,
                   std::size_t settleCap = 400000, double maxWallMs = 0.0) const;

 private:
  void* data_ = nullptr;
  std::size_t size_ = 0;
  std::size_t nodeCount_ = 0;
  std::size_t arcCount_ = 0;
  float profileKmh_ = 80.0f;
  const int64_t* nodeIds_ = nullptr;
  const uint32_t* ranks_ = nullptr;
  const uint64_t* upRow_ = nullptr;
  const void* arcs_ = nullptr;
};

}  // namespace mmlp
