#pragma once

#include "mmlp/types.hpp"

#include <unordered_map>
#include <vector>

namespace mmlp {

struct AdjacencyEdge {
  int64_t edgeId = 0;
  int64_t to = 0;
  EdgeType type = EdgeType::ROAD;
  double length = 0.0;
  double speedLimit = 0.0;
};

// In-memory multimodal graph built from OSM or test fixtures.
class MultimodalGraph {
 public:
  void addNode(Node node);
  void addEdge(Edge edge);

  void reserveGraph(std::size_t nodeCount, std::size_t edgeCount);
  void addNodeBulk(Node node);
  void addEdgeBulk(Edge edge);

  const std::vector<Node>& nodes() const { return nodes_; }
  const std::vector<Edge>& edges() const { return edges_; }
  const std::vector<AdjacencyEdge>& neighbors(int64_t nodeId) const;

  const Node* findNode(int64_t id) const;
  const Edge* findEdge(int64_t id) const;

  void clear();

 private:
  std::vector<Node> nodes_;
  std::vector<Edge> edges_;
  std::unordered_map<int64_t, std::size_t> nodeIndex_;
  std::unordered_map<int64_t, std::size_t> edgeIndex_;
  std::unordered_map<int64_t, std::vector<AdjacencyEdge>> adjacency_;
  static const std::vector<AdjacencyEdge> kEmptyNeighbors;
};

}  // namespace mmlp
