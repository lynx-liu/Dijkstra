#pragma once

#include "mmlp/types.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_set>

namespace mmlp {

class GraphFileStore;

// One directed half-edge in the mmap CSR (undirected roads stored as two arcs).
struct CsrArc {
  int64_t toNodeId = 0;
  int64_t edgeId = 0;
  EdgeType type = EdgeType::ROAD;
  float length = 0.0f;
  float speedLimit = 0.0f;
};

// mmap CSR adjacency aligned with .nidx row order (row i = nodeIndex_[i].id).
class CsrGraph {
 public:
  CsrGraph() = default;
  ~CsrGraph();

  CsrGraph(const CsrGraph&) = delete;
  CsrGraph& operator=(const CsrGraph&) = delete;

  bool open(const std::string& csrPath, std::string* error = nullptr);
  bool isOpen() const { return data_ != nullptr; }

  std::size_t nodeCount() const { return nodeCount_; }
  std::size_t arcCount() const { return arcCount_; }

  // Row index for node id via sorted nidx (same as GraphFileStore node table).
  int nodeRow(const GraphFileStore& store, int64_t nodeId) const;

  using NeighborFn = std::function<void(const CsrArc&)>;

  void forEachNeighbor(const GraphFileStore& store, int64_t nodeId,
                       const std::unordered_set<int64_t>* allowedEdgeIds,
                       const NeighborFn& fn) const;

 private:
  void* data_ = nullptr;
  std::size_t size_ = 0;
  std::size_t nodeCount_ = 0;
  std::size_t arcCount_ = 0;
  const uint64_t* rowPtr_ = nullptr;
  const void* arcs_ = nullptr;
};

}  // namespace mmlp
