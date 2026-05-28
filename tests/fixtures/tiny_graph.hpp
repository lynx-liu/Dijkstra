#pragma once

#include "mmlp/graph.hpp"

namespace mmlp::test {

// Straight corridor: N1 —1000m— N2 —1000m— N3 (road only).
// Manual meeting check (phase 4+):
//   Truck A at N1, Truck B at N3, same speed 72 km/h => meet at N2, T_meet = 50s each leg.
inline MultimodalGraph makeTinyRoadGraph() {
  MultimodalGraph g;

  g.addNode(Node{1, 43.80, 87.50, NodeKind::ROAD_JUNCTION});
  g.addNode(Node{2, 43.81, 87.51, NodeKind::ROAD_JUNCTION});
  g.addNode(Node{3, 43.82, 87.52, NodeKind::ROAD_JUNCTION});

  Edge e12;
  e12.id = 101;
  e12.from = 1;
  e12.to = 2;
  e12.type = EdgeType::ROAD;
  e12.length = 1000.0;
  e12.speedLimit = 72.0;
  g.addEdge(e12);

  Edge e23 = e12;
  e23.id = 102;
  e23.from = 2;
  e23.to = 3;
  g.addEdge(e23);

  return g;
}

}  // namespace mmlp::test
