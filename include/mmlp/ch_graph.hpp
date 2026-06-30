#pragma once

#include "mmlp/csr_graph.hpp"
#include "mmlp/types.hpp"

#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

namespace mmlp {

class GraphFileStore;
struct RouteToGoal;
struct PredictParam;

// mmap Contraction Hierarchies overlay on the highway (arterial) subgraph.
// Built from china.mmlp.hwy.csr by mmlp_build_hwy_ch.
class ChGraph {
 public:
  ChGraph() = default;
  ~ChGraph();

  ChGraph(const ChGraph&) = delete;
  ChGraph& operator=(const ChGraph&) = delete;

  bool open(const std::string& path, std::string* error = nullptr);
  bool isOpen() const { return data_ != nullptr; }

  std::size_t nodeCount() const { return nodeCount_; }
  std::size_t upArcCount() const { return upArcCount_; }
  double profileKmh() const { return profileKmh_; }

  // Global node id -> compact CH index (-1 if not in overlay).
  int nodeIndex(int64_t nodeId) const;

  // Bidirectional CH shortest path on fixed profile (build-time speed).
  RouteToGoal query(const GraphFileStore& store, const CsrGraph& hwyCsr, int64_t fromNodeId,
                    int64_t toNodeId, VehicleType type, const PredictParam& param,
                    double maxTime) const;

 private:
  void* data_ = nullptr;
  std::size_t size_ = 0;
  std::size_t nodeCount_ = 0;
  std::size_t upArcCount_ = 0;
  float profileKmh_ = 80.0f;
  const int64_t* nodeIds_ = nullptr;
  const uint32_t* ranks_ = nullptr;
  const uint64_t* upRow_ = nullptr;
  const void* upArcs_ = nullptr;
  mutable std::vector<std::vector<std::tuple<int, int64_t, float>>> reverseDown_;
};

}  // namespace mmlp
