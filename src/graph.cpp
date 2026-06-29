#include "mmlp/graph.hpp"

namespace mmlp {

const std::vector<AdjacencyEdge> MultimodalGraph::kEmptyNeighbors{};

void MultimodalGraph::clear() {
  nodes_.clear();
  edges_.clear();
  nodeIndex_.clear();
  edgeIndex_.clear();
  adjacency_.clear();
  denseAdjacency_.clear();
  useDenseAdjacency_ = false;
}

void MultimodalGraph::reserveGraph(std::size_t nodeCount, std::size_t edgeCount) {
  nodes_.reserve(nodeCount);
  nodeIndex_.reserve(nodeCount);
  edges_.reserve(edgeCount);
  edgeIndex_.reserve(edgeCount);
  adjacency_.reserve(nodeCount / 4 + 1);
}

void MultimodalGraph::addNodeBulk(Node node) {
  nodeIndex_[node.id] = nodes_.size();
  nodes_.push_back(std::move(node));
}

void MultimodalGraph::addEdgeBulk(Edge edge) {
  const auto fromIt = nodeIndex_.find(edge.from);
  const auto toIt = nodeIndex_.find(edge.to);
  if (fromIt == nodeIndex_.end() || toIt == nodeIndex_.end()) {
    return;
  }
  edgeIndex_[edge.id] = edges_.size();
  edges_.push_back(edge);

  AdjacencyEdge out;
  out.edgeId = edge.id;
  out.to = edge.to;
  out.type = edge.type;
  out.length = edge.length;
  out.speedLimit = edge.speedLimit;
  adjacency_[edge.from].push_back(out);

  AdjacencyEdge back;
  back.edgeId = edge.id;
  back.to = edge.from;
  back.type = edge.type;
  back.length = edge.length;
  back.speedLimit = edge.speedLimit;
  adjacency_[edge.to].push_back(back);
}

void MultimodalGraph::buildFromSubset(std::vector<Node>&& nodes, std::vector<Edge>&& edges) {
  clear();
  useDenseAdjacency_ = true;
  nodes_.reserve(nodes.size());
  nodeIndex_.reserve(nodes.size() * 2);
  for (auto& node : nodes) {
    nodeIndex_[node.id] = nodes_.size();
    nodes_.push_back(std::move(node));
  }
  edges_.reserve(edges.size());
  edgeIndex_.reserve(edges.size() * 2);
  for (auto& edge : edges) {
    edgeIndex_[edge.id] = edges_.size();
    edges_.push_back(edge);
  }

  denseAdjacency_.assign(nodes_.size(), {});
  std::vector<std::size_t> degree(nodes_.size(), 0);
  for (const Edge& edge : edges_) {
    const auto fromIt = nodeIndex_.find(edge.from);
    const auto toIt = nodeIndex_.find(edge.to);
    if (fromIt == nodeIndex_.end() || toIt == nodeIndex_.end()) {
      continue;
    }
    degree[fromIt->second]++;
    degree[toIt->second]++;
  }
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    denseAdjacency_[i].reserve(degree[i]);
  }

  for (const Edge& edge : edges_) {
    const auto fromIt = nodeIndex_.find(edge.from);
    const auto toIt = nodeIndex_.find(edge.to);
    if (fromIt == nodeIndex_.end() || toIt == nodeIndex_.end()) {
      continue;
    }
    AdjacencyEdge out;
    out.edgeId = edge.id;
    out.to = edge.to;
    out.type = edge.type;
    out.length = edge.length;
    out.speedLimit = edge.speedLimit;
    denseAdjacency_[fromIt->second].push_back(out);

    AdjacencyEdge back;
    back.edgeId = edge.id;
    back.to = edge.from;
    back.type = edge.type;
    back.length = edge.length;
    back.speedLimit = edge.speedLimit;
    denseAdjacency_[toIt->second].push_back(back);
  }
}

void MultimodalGraph::addNode(Node node) {
  if (nodeIndex_.count(node.id) > 0) {
    return;
  }
  nodeIndex_[node.id] = nodes_.size();
  nodes_.push_back(std::move(node));
}

void MultimodalGraph::addEdge(Edge edge) {
  if (edgeIndex_.count(edge.id) > 0) {
    return;
  }
  if (nodeIndex_.count(edge.from) == 0 || nodeIndex_.count(edge.to) == 0) {
    return;
  }
  edgeIndex_[edge.id] = edges_.size();
  edges_.push_back(edge);

  AdjacencyEdge out;
  out.edgeId = edge.id;
  out.to = edge.to;
  out.type = edge.type;
  out.length = edge.length;
  out.speedLimit = edge.speedLimit;
  adjacency_[edge.from].push_back(out);

  AdjacencyEdge back;
  back.edgeId = edge.id;
  back.to = edge.from;
  back.type = edge.type;
  back.length = edge.length;
  back.speedLimit = edge.speedLimit;
  adjacency_[edge.to].push_back(back);
}

const std::vector<AdjacencyEdge>& MultimodalGraph::neighbors(int64_t nodeId) const {
  if (useDenseAdjacency_) {
    const auto it = nodeIndex_.find(nodeId);
    if (it == nodeIndex_.end()) {
      return kEmptyNeighbors;
    }
    return denseAdjacency_[it->second];
  }
  const auto it = adjacency_.find(nodeId);
  if (it == adjacency_.end()) {
    return kEmptyNeighbors;
  }
  return it->second;
}

const Node* MultimodalGraph::findNode(int64_t id) const {
  const auto it = nodeIndex_.find(id);
  if (it == nodeIndex_.end()) {
    return nullptr;
  }
  return &nodes_[it->second];
}

const Edge* MultimodalGraph::findEdge(int64_t id) const {
  const auto it = edgeIndex_.find(id);
  if (it == edgeIndex_.end()) {
    return nullptr;
  }
  return &edges_[it->second];
}

}  // namespace mmlp
