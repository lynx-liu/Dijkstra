#include "mmlp/fleet_service.hpp"
#include "mmlp/graph_io.hpp"
#include "mmlp/json_util.hpp"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

namespace {

enum class LoadMode { Index, Full, Region };

void printUsage() {
  std::cerr << "Usage: mmlp_service --graph <china.mmlp.bin> [--padding-m M] "
               "[--load-mode index|full|region]\n"
            << "  index: load .sidx only (~seconds, needs build_graph_auxiliary.py)\n"
            << "  full:  load entire graph (~5min, ~27GB RAM)\n"
            << "  region: load bbox on first request\n"
            << "  One JSON object per line on stdin.\n";
}

LoadMode envLoadMode() {
  const char* mode = std::getenv("MMLP_LOAD_MODE");
  if (mode == nullptr || mode[0] == '\0') {
    return LoadMode::Index;
  }
  if (std::strcmp(mode, "full") == 0) {
    return LoadMode::Full;
  }
  if (std::strcmp(mode, "region") == 0) {
    return LoadMode::Region;
  }
  return LoadMode::Index;
}

LoadMode parseLoadMode(const std::string& mode) {
  if (mode == "full") {
    return LoadMode::Full;
  }
  if (mode == "region") {
    return LoadMode::Region;
  }
  return LoadMode::Index;
}

}  // namespace

int main(int argc, char** argv) {
  std::string graphPath = "data/graph/china.mmlp.bin";
  double padding = 150000.0;
  LoadMode loadMode = envLoadMode();

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--graph" && i + 1 < argc) {
      graphPath = argv[++i];
    } else if (arg == "--padding-m" && i + 1 < argc) {
      padding = std::stod(argv[++i]);
    } else if (arg == "--load-mode" && i + 1 < argc) {
      loadMode = parseLoadMode(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      printUsage();
      return 0;
    } else {
      printUsage();
      return 1;
    }
  }

  if (const char* env = std::getenv("MMLP_GRAPH_PATH")) {
    graphPath = env;
  }

  mmlp::FleetMeetingService service(graphPath, padding);

  if (loadMode == LoadMode::Full) {
    std::cerr << "[mmlp_service] loading FULL graph at startup (needs ~27GB RAM, ~5min)...\n";
    std::string loadErr;
    if (!service.preloadFullGraph(&loadErr)) {
      std::cerr << "[mmlp_service] FATAL: " << loadErr << "\n";
      return 1;
    }
  } else if (loadMode == LoadMode::Index) {
    if (!mmlp::graphAuxiliaryReady(graphPath)) {
      std::cerr << "[mmlp_service] FATAL: index files missing for " << graphPath << "\n"
                << "  Run once: python3 tools/build_graph_auxiliary.py " << graphPath << "\n"
                << "  Or use MMLP_LOAD_MODE=full (slow) or region\n";
      return 1;
    }
    std::cerr << "[mmlp_service] loading spatial index only (fast startup)...\n";
    std::string loadErr;
    if (!service.preloadIndexOnly(&loadErr)) {
      std::cerr << "[mmlp_service] FATAL: " << loadErr << "\n";
      return 1;
    }
  } else {
    std::cerr << "[mmlp_service] region mode (graph loads on first vehicle bbox)\n";
  }

  std::cerr << "[mmlp_service] ready graph=" << graphPath << " padding_m=" << padding
            << " full=" << (service.isFullGraphLoaded() ? 1 : 0)
            << " index=" << (service.isIndexOnlyMode() ? 1 : 0)
            << " (default maxTime=2d, override with MMLP_MAX_TIME_SEC)\n";
  std::cout << mmlp::formatOkJson("ready") << "\n" << std::flush;

  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {
      continue;
    }

    if (mmlp::isDestinationArrivalCommand(line)) {
      mmlp::DestinationQuery dest;
      std::vector<mmlp::VehicleInfo> vehicles;
      std::vector<mmlp::VehicleHistory> histories;
      std::string parseErr;
      if (!mmlp::parseDestinationArrivalJson(line, dest, &vehicles, &histories, &parseErr)) {
        std::cout << mmlp::formatErrorJson(parseErr) << "\n" << std::flush;
        continue;
      }
      const std::vector<mmlp::VehicleInfo>* vehPtr =
          vehicles.empty() ? nullptr : &vehicles;
      const std::vector<mmlp::VehicleHistory>* histPtr =
          histories.empty() ? nullptr : &histories;
      std::string runErr;
      const auto summary = service.vehiclesReachDestinationBy(
          dest.lat, dest.lon, dest.arriveByUnix, dest.type, dest.sortBy, vehPtr, histPtr, &runErr);
      if (!runErr.empty() && summary.vehicles.empty()) {
        std::cout << mmlp::formatErrorJson(runErr) << "\n" << std::flush;
        continue;
      }
      std::cout << mmlp::formatDestinationArrivalJson(summary) << "\n" << std::flush;
      continue;
    }

    if (mmlp::isMeetWithLeadCommand(line)) {
      std::vector<mmlp::VehicleInfo> vehicles;
      std::vector<mmlp::VehicleHistory> histories;
      std::string parseErr;
      if (!mmlp::parseVehiclesJson(line, vehicles, histories, &parseErr)) {
        std::cout << mmlp::formatErrorJson(parseErr) << "\n" << std::flush;
        continue;
      }
      std::string runErr;
      const auto results = service.meetingsWithLead(vehicles, histories, &runErr);
      if (!runErr.empty() && results.empty()) {
        std::cout << mmlp::formatErrorJson(runErr) << "\n" << std::flush;
        continue;
      }
      std::cout << mmlp::formatMeetingsWithLeadJson(results) << "\n" << std::flush;
      continue;
    }

    if (mmlp::isPreloadCommand(line)) {
      if (line.find("\"full\"") != std::string::npos ||
          line.find("\"mode\":\"full\"") != std::string::npos) {
        std::string runErr;
        if (!service.preloadFullGraph(&runErr)) {
          std::cout << mmlp::formatErrorJson(runErr) << "\n" << std::flush;
          continue;
        }
        std::cout << mmlp::formatOkJson("full graph preloaded") << "\n" << std::flush;
        continue;
      }
      if (line.find("\"index\"") != std::string::npos ||
          line.find("\"mode\":\"index\"") != std::string::npos) {
        std::string runErr;
        if (!service.preloadIndexOnly(&runErr)) {
          std::cout << mmlp::formatErrorJson(runErr) << "\n" << std::flush;
          continue;
        }
        std::cout << mmlp::formatOkJson("index preloaded") << "\n" << std::flush;
        continue;
      }

      mmlp::GeoBBox bbox;
      std::string parseErr;
      if (!mmlp::parsePreloadJson(line, bbox, &parseErr)) {
        std::cout << mmlp::formatErrorJson(parseErr) << "\n" << std::flush;
        continue;
      }
      std::string runErr;
      if (!service.preloadRegion(bbox, &runErr)) {
        std::cout << mmlp::formatErrorJson(runErr) << "\n" << std::flush;
        continue;
      }
      std::cout << mmlp::formatOkJson("graph preloaded") << "\n" << std::flush;
      continue;
    }

    mmlp::VehicleInfo vehicle;
    mmlp::VehicleHistory history;
    std::string parseErr;
    if (!mmlp::parseVehicleJson(line, vehicle, &history, &parseErr)) {
      std::cout << mmlp::formatErrorJson(parseErr) << "\n" << std::flush;
      continue;
    }

    const mmlp::VehicleHistory* histPtr =
        history.speedSamples.empty() ? nullptr : &history;

    std::string runErr;
    const auto result = service.ingestVehicle(vehicle, histPtr, &runErr);
    if (!result.found && !runErr.empty()) {
      std::cout << mmlp::formatErrorJson(runErr) << "\n" << std::flush;
      continue;
    }

    std::cout << mmlp::formatFocalBestMeetingJson(result) << "\n" << std::flush;
  }

  return 0;
}
