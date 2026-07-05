// CH snap + query smoke test using arrival::predictVehicleToDestinationCh.
#include "mmlp/arrival.hpp"
#include "mmlp/graph_store.hpp"
#include "mmlp/region_loader.hpp"

#include <chrono>
#include <deque>
#include <iostream>
#include <string>
#include <unordered_set>

using namespace mmlp;

static bool hwyReachable(const GraphFileStore& store, int64_t from, int64_t to) {
  const CsrGraph& hwy = store.hwyCsr();
  std::deque<int64_t> q;
  std::unordered_set<int64_t> seen;
  q.push_back(from);
  seen.insert(from);
  while (!q.empty()) {
    const int64_t u = q.front();
    q.pop_front();
    if (u == to) {
      return true;
    }
    hwy.forEachNeighbor(store, u, nullptr, [&](const CsrArc& arc) {
      if (seen.count(arc.toNodeId) == 0) {
        seen.insert(arc.toNodeId);
        q.push_back(arc.toNodeId);
      }
    });
  }
  return false;
}

int main(int argc, char** argv) {
  const std::string bin =
      argc > 1 ? argv[1] : "data/graph/china_prd.mmlp.bin";

  GraphFileStore store;
  std::string err;
  if (!store.open(bin, &err)) {
    std::cerr << "open failed: " << err << "\n";
    return 1;
  }

  GraphContext ctx;
  if (!loadGraphContextIndexOnly(bin, ctx, &err)) {
    std::cerr << "index failed: " << err << "\n";
    return 1;
  }

  GraphFileStore national;
  const GraphFileStore* chStore = &store;
  if (argc >= 3) {
    std::string nerr;
    if (national.open(argv[2], &nerr) && national.hasCh() && national.hasHwyCsr()) {
      chStore = &national;
      std::cerr << "using national ch overlay: " << argv[2] << "\n";
    }
  }
  if (!chStore->hasCh() || !chStore->hasHwyCsr()) {
    std::cerr << "no ch overlay\n";
    return 1;
  }

  const auto t0 = std::chrono::steady_clock::now();
  chStore->ch().warmReverseDown();
  const auto t1 = std::chrono::steady_clock::now();
  std::cerr << "warm_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "\n";

  if (argc >= 5) {
    const int64_t from = std::stoll(argv[3]);
    const int64_t to = std::stoll(argv[4]);
    std::cerr << "from=" << from << " chIdx=" << chStore->ch().nodeIndex(from) << "\n";
    std::cerr << "to=" << to << " chIdx=" << chStore->ch().nodeIndex(to) << "\n";
    std::cerr << "hwy_reachable=" << (hwyReachable(*chStore, from, to) ? 1 : 0) << "\n";
    PredictParam param;
    const RouteToGoal path = chStore->ch().query(*chStore, chStore->hwyCsr(), from, to,
                                                 VehicleType::TRUCK, param, 86400.0);
    std::cerr << "ch_travel=" << path.travelTimeSec << " pts=" << path.polyline.points.size()
              << "\n";
    return path.travelTimeSec < 1e100 ? 0 : 2;
  }

  VehicleInfo veh;
  veh.id = "test";
  veh.lat = 22.539045;
  veh.lon = 113.944015;
  veh.type = VehicleType::TRUCK;
  veh.speed = 60;
  veh.timestamp = 1719731373;

  DestinationQuery dest;
  dest.lat = 23.137133;
  dest.lon = 113.276772;
  dest.type = VehicleType::TRUCK;
  dest.arriveByUnix = veh.timestamp + 86400;

  PredictParam param;
  param.maxTime = 86400.0;

  const auto t2 = std::chrono::steady_clock::now();
  const auto row =
      predictVehicleToDestinationCh(store, ctx.index, *chStore, veh, nullptr, dest, param);
  const auto t3 = std::chrono::steady_clock::now();

  std::cerr << "query_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << "\n";
  if (row) {
    std::cerr << "hit travel=" << row->travelDurationSec
              << " route_pts=" << row->route.points.size() << "\n";
    return 0;
  }
  std::cerr << "miss\n";
  return 2;
}
