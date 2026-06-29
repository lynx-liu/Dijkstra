#pragma once

#include "mmlp/csr_graph.hpp"
#include "mmlp/geo.hpp"
#include "mmlp/graph.hpp"
#include "mmlp/graph_store.hpp"
#include "mmlp/types.hpp"

#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mmlp {

constexpr double kInfTime = std::numeric_limits<double>::infinity();

struct GraphPosition {
  bool valid = false;
  int64_t nodeId = 0;   // set when snapped exactly to a node
  int64_t edgeId = 0;   // set when on an edge (nodeId = edge.from)
  double alongMeters = 0.0;  // distance from edge.from along edge
};

struct TimeField {
  std::unordered_map<int64_t, double> atNode;
};

// Shortest-path tree from a goal (one search serves N vehicle lookups).
struct RoutedTimeField {
  std::unordered_map<int64_t, double> atNode;
  std::unordered_map<int64_t, int64_t> parentTowardGoal;
};

TimeField computeTimeField(const MultimodalGraph& graph, const GraphPosition& start,
                           double speedMs, VehicleType type, const PredictParam& param,
                           double maxTime, const LatLon* goal = nullptr);

// Advance along fastest routes for `deltaSeconds` from `start`.
GraphPosition advancePosition(const MultimodalGraph& graph, const GraphPosition& start,
                              double speedMs, VehicleType type, const PredictParam& param,
                              double deltaSeconds);

double timeAtPosition(const MultimodalGraph& graph, const TimeField& field,
                      const GraphPosition& pos, double speedMs, VehicleType type,
                      const PredictParam& param);

GraphPosition positionAtTime(const MultimodalGraph& graph, const GraphPosition& start,
                           double speedMs, VehicleType type, const PredictParam& param,
                           double timeSeconds);

LatLon positionLatLon(const MultimodalGraph& graph, const GraphPosition& pos);

struct RouteToGoal {
  double travelTimeSec = kInfTime;
  RoutePolyline polyline;
};

// Single A* search: travel time + map polyline (used by destination arrival).
RouteToGoal computeRouteToGoal(const MultimodalGraph& graph, const GraphPosition& start,
                               const GraphPosition& goal, double speedMs, VehicleType type,
                               const PredictParam& param, double maxTime);

RoutedTimeField computeRoutedTimeFieldFromGoal(const MultimodalGraph& graph,
                                               const GraphPosition& goal, double speedMs,
                                               VehicleType type, const PredictParam& param,
                                               double maxTime,
                                               const std::unordered_set<int64_t>* targetNodes =
                                                   nullptr);

RouteToGoal routeFromRoutedField(const MultimodalGraph& graph, const RoutedTimeField& field,
                                 const GraphPosition& start, const GraphPosition& goal,
                                 double speedMs, VehicleType type, const PredictParam& param);

RoutedTimeField computeRoutedTimeFieldFromGoalCsr(
    const GraphFileStore& store, const CsrGraph& csr, const GraphPosition& goal, double speedMs,
    VehicleType type, const PredictParam& param, double maxTime,
    const std::unordered_set<int64_t>* allowedEdgeIds);

RouteToGoal routeFromRoutedFieldCsr(const GraphFileStore& store, const CsrGraph& csr,
                                    const RoutedTimeField& field, const GraphPosition& start,
                                    const GraphPosition& goal, double speedMs, VehicleType type,
                                    const PredictParam& param);

// Downsample for API/map (full junction trace can be 10k+ points on long haul).
void simplifyRoutePolyline(RoutePolyline& route, std::size_t maxPoints = 120);

// Shortest-time route polyline from start to goal on the graph (for map display).
RoutePolyline computeRoutePolyline(const MultimodalGraph& graph, const GraphPosition& start,
                                   const GraphPosition& goal, double speedMs, VehicleType type,
                                   const PredictParam& param, double maxTime);

}  // namespace mmlp
