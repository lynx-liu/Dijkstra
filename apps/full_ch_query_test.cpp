// Validate full CH queries against ground-truth CSR Dijkstra on random pairs.
#include "mmlp/csr_graph.hpp"
#include "mmlp/full_ch_graph.hpp"
#include "mmlp/graph_store.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <unordered_map>

using namespace mmlp;

namespace {

constexpr double kProfileMs = 80.0 / 3.6;

double arcW(const CsrArc& arc) {
  double eff = kProfileMs;
  if (arc.speedLimit > 0.0f) {
    eff = std::min(eff, static_cast<double>(arc.speedLimit) / 3.6);
  }
  return static_cast<double>(arc.length) / eff;
}

double dijkstra(const GraphFileStore& store, const CsrGraph& csr, int64_t from, int64_t to,
                double cap) {
  std::unordered_map<int64_t, double> dist;
  using QI = std::pair<double, int64_t>;
  std::priority_queue<QI, std::vector<QI>, std::greater<QI>> pq;
  dist[from] = 0.0;
  pq.push({0.0, from});
  while (!pq.empty()) {
    const auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[u] + 1e-9 || d > cap) {
      continue;
    }
    if (u == to) {
      return d;
    }
    csr.forEachNeighbor(store, u, nullptr, [&](const CsrArc& arc) {
      if (arc.type != EdgeType::ROAD) {
        return;
      }
      const double nd = d + arcW(arc);
      auto it = dist.find(arc.toNodeId);
      if (it == dist.end() || nd < it->second - 1e-9) {
        dist[arc.toNodeId] = nd;
        pq.push({nd, arc.toNodeId});
      }
    });
  }
  return std::numeric_limits<double>::infinity();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: full_ch_query_test <region.mmlp.bin> [pairs] [fromId toId]\n";
    return 1;
  }
  const int pairs = argc > 2 ? std::atoi(argv[2]) : 30;
  if (argc > 4) {
    GraphFileStore store;
    std::string err;
    if (!store.open(argv[1], &err)) {
      std::cerr << "open failed: " << err << "\n";
      return 1;
    }
    const int64_t a = std::atoll(argv[3]);
    const int64_t b = std::atoll(argv[4]);
    for (int i = 0; i < pairs; ++i) {
      const auto t0 = std::chrono::steady_clock::now();
      const FullChGraph::PathResult res =
          store.fullCh().route({{a, 0.0}}, {{b, 0.0}}, 6.0 * 86400.0);
      const auto t1 = std::chrono::steady_clock::now();
      std::cout << "single pair found=" << res.found << " sec=" << res.profileSec
                << " arcs=" << res.arcs.size() << " us="
                << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()
                << "\n";
    }
    return 0;
  }
  GraphFileStore store;
  std::string err;
  if (!store.open(argv[1], &err)) {
    std::cerr << "open failed: " << err << "\n";
    return 1;
  }
  if (!store.hasFullCh() || !store.hasCsr()) {
    std::cerr << "need .full.ch and .csr\n";
    return 1;
  }
  const FullChGraph& ch = store.fullCh();
  const CsrGraph& csr = store.csr();

  std::mt19937_64 rng(42);
  int ok = 0;
  int mismatch = 0;
  int noPathBoth = 0;
  double totalChUs = 0.0;
  double maxChUs = 0.0;
  for (int t = 0; t < pairs; ++t) {
    // random CH nodes (guaranteed road nodes)
    const std::size_t n = ch.nodeCount();
    int64_t a = 0;
    int64_t b = 0;
    {
      std::uniform_int_distribution<std::size_t> pick(0, n - 1);
      // read ids via nodeIndex trick: sample until valid
      // nodeIds are private; use csr rows -> global ids via store? use nidx order:
      // Instead sample rows from store node table by probing random ids is hard;
      // easiest: sample two indices and recover ids through binary search bounds.
      // FullChGraph lacks an id accessor, so scan: pick index, then binary search
      // over int64 range is silly. Add-on: reuse matchless approach - walk csr row.
      (void)pick;
    }
    // Sample global node ids from the store via random row of CSR: row r maps to
    // nidx row r; we don't have direct accessor either. Fallback: rejection-sample
    // int64 ids from the CH by binary searching random indices via nodeIndex is
    // impossible without accessor -> instead do BFS from a fixed known id.
    // Simpler: pick random rows and use csr.forEachNeighbor on ids discovered by
    // expanding from previously found ids. Seed with the first arc target found.
    static std::vector<int64_t> poolIds;
    if (poolIds.empty()) {
      // harvest ids: iterate arcs of random known node ids starting from any id in
      // the store: use brute force scan over a few million ids is not viable;
      // instead pull ids from the store's node index through nodeLatLon probing of
      // sequential rows: GraphFileStore lacks that accessor too. So harvest from
      // CSR arcs reachable from an id found by scanning raw nidx file.
      FILE* f = std::fopen((std::string(argv[1]).substr(0, std::string(argv[1]).size() - 4) +
                            ".nidx").c_str(),
                           "rb");
      if (f == nullptr) {
        std::cerr << "cannot open nidx\n";
        return 1;
      }
      std::fseek(f, 20, SEEK_SET);
      struct Rec {
        int64_t id;
        uint64_t off;
      } rec;
      std::vector<int64_t> all;
      while (std::fread(&rec, sizeof(rec), 1, f) == 1) {
        all.push_back(rec.id);
      }
      std::fclose(f);
      std::shuffle(all.begin(), all.end(), rng);
      for (int64_t id : all) {
        if (ch.nodeIndex(id) >= 0) {
          poolIds.push_back(id);
          if (poolIds.size() >= 4096) {
            break;
          }
        }
      }
    }
    std::uniform_int_distribution<std::size_t> pick(0, poolIds.size() - 1);
    a = poolIds[pick(rng)];
    b = poolIds[pick(rng)];
    if (a == b) {
      continue;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const FullChGraph::PathResult res = ch.route({{a, 0.0}}, {{b, 0.0}}, 6.0 * 86400.0);
    const auto t1 = std::chrono::steady_clock::now();
    const double us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    totalChUs += us;
    maxChUs = std::max(maxChUs, us);
    std::cerr << "pair a=" << a << " b=" << b << " found=" << res.found << " us=" << us
              << " arcs=" << res.arcs.size() << " settled=" << res.settledNodes << "\n";

    const double truth = dijkstra(store, csr, a, b, 6.0 * 86400.0);
    const bool chFound = res.found;
    const bool djFound = truth < std::numeric_limits<double>::infinity();
    if (!chFound && !djFound) {
      ++noPathBoth;
      continue;
    }
    if (chFound != djFound) {
      ++mismatch;
      std::cerr << "MISMATCH reach a=" << a << " b=" << b << " ch=" << chFound
                << " dj=" << djFound << "\n";
      continue;
    }
    // verify unpacked arc lengths reproduce the weight
    double sumW = 0.0;
    for (const auto& arc : res.arcs) {
      double eff = kProfileMs;
      if (arc.speedLimitKmh > 0.0f) {
        eff = std::min(eff, static_cast<double>(arc.speedLimitKmh) / 3.6);
      }
      sumW += static_cast<double>(arc.lengthM) / eff;
    }
    const double relErr = std::abs(res.profileSec - truth) / std::max(truth, 1.0);
    const double unpackErr = std::abs(sumW - res.profileSec) / std::max(res.profileSec, 1.0);
    if (relErr > 0.001 || unpackErr > 0.005) {
      ++mismatch;
      std::cerr << "MISMATCH cost a=" << a << " b=" << b << " ch=" << res.profileSec
                << " dj=" << truth << " unpackSum=" << sumW << " arcs=" << res.arcs.size()
                << "\n";
    } else {
      ++ok;
    }
  }
  std::cout << "pairs_ok=" << ok << " mismatch=" << mismatch << " nopath_both=" << noPathBoth
            << " avg_ch_us=" << (totalChUs / std::max(1, ok + mismatch))
            << " max_ch_us=" << maxChUs << "\n";
  return mismatch == 0 ? 0 : 2;
}
