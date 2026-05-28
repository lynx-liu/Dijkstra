#include "mmlp/predict.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

void usage() {
  std::cerr
      << "Usage: mmlp_predict --graph <china.mmlp.bin> --focal <id> \\\n"
      << "       --vehicle <id>,<lat>,<lon>,<speed_kmh>,<unix_ts> [...]\n"
      << "Optional: --padding-m <meters>  (default 150000)\n"
      << "          --all-pairs           (list all pairwise meetings)\n";
}

bool parseVehicle(const std::string& text, mmlp::VehicleInfo& v) {
  std::stringstream ss(text);
  std::string lat, lon, speed, ts;
  if (!std::getline(ss, v.id, ',') || !std::getline(ss, lat, ',') ||
      !std::getline(ss, lon, ',') || !std::getline(ss, speed, ',') ||
      !std::getline(ss, ts, ',')) {
    return false;
  }
  v.type = mmlp::VehicleType::TRUCK;
  v.lat = std::stod(lat);
  v.lon = std::stod(lon);
  v.speed = std::stod(speed);
  v.timestamp = std::stoll(ts);
  v.heading = 0.0;
  return true;
}

std::string jsonEscape(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  return out;
}

std::string formatMeetTimeUtc(double meetTimeUnix) {
  const auto sec = static_cast<std::time_t>(std::floor(meetTimeUnix));
  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &sec);
#else
  gmtime_r(&sec, &tm);
#endif
  char buf[32];
  if (std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm) == 0) {
    return {};
  }
  return buf;
}

void printMeetingJson(std::ostream& out, const mmlp::MeetingResult& m, bool prettyIndent) {
  const std::string prefix = prettyIndent ? "  " : "";
  const int64_t meetUnix = static_cast<int64_t>(std::llround(m.meetTime));
  const std::string meetUtc = formatMeetTimeUtc(m.meetTime);

  out << prefix << "{"
      << "\"vehicleA\":\"" << jsonEscape(m.vehicleA) << "\","
      << "\"vehicleB\":\"" << jsonEscape(m.vehicleB) << "\","
      << "\"meetTimeUnix\":" << meetUnix << ","
      << "\"meetTimeUtc\":\"" << meetUtc << "\","
      << "\"lat\":" << std::fixed << std::setprecision(6) << m.lat << ","
      << "\"lon\":" << m.lon << ","
      << "\"locationId\":\"" << jsonEscape(m.locationId) << "\""
      << "}";
  out << std::defaultfloat;
}

void printBest(const mmlp::FocalBestMeeting& b) {
  if (!b.found) {
    std::cout << "{\"found\":false,\"focal\":\"" << jsonEscape(b.focalVehicleId) << "\"}\n";
    return;
  }

  const int64_t meetUnix = static_cast<int64_t>(std::llround(b.meetTime));
  const std::string meetUtc = formatMeetTimeUtc(b.meetTime);

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "{"
            << "\"found\":true,"
            << "\"focal\":\"" << jsonEscape(b.focalVehicleId) << "\","
            << "\"partner\":\"" << jsonEscape(b.partnerVehicleId) << "\","
            << "\"meetTimeUnix\":" << meetUnix << ","
            << "\"meetTimeUtc\":\"" << meetUtc << "\","
            << "\"meetDurationSec\":" << std::setprecision(2) << b.meetDuration << ","
            << "\"lat\":" << std::setprecision(6) << b.lat << ","
            << "\"lon\":" << b.lon << ","
            << "\"locationId\":\"" << jsonEscape(b.locationId) << "\""
            << "}\n";
  std::cout << std::defaultfloat;
}

}  // namespace

int main(int argc, char** argv) {
  std::string graphPath = "data/graph/china.mmlp.bin";
  if (const char* env = std::getenv("MMLP_GRAPH_PATH")) {
    graphPath = env;
  }
  std::string focalId;
  std::vector<mmlp::VehicleInfo> fleet;
  double padding = 150000.0;
  bool allPairs = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--graph" && i + 1 < argc) {
      graphPath = argv[++i];
    } else if (arg == "--focal" && i + 1 < argc) {
      focalId = argv[++i];
    } else if (arg == "--vehicle" && i + 1 < argc) {
      mmlp::VehicleInfo v;
      if (!parseVehicle(argv[++i], v)) {
        std::cerr << "bad --vehicle format\n";
        return 1;
      }
      fleet.push_back(v);
    } else if (arg == "--padding-m" && i + 1 < argc) {
      padding = std::stod(argv[++i]);
    } else if (arg == "--all-pairs") {
      allPairs = true;
    } else if (arg == "--help" || arg == "-h") {
      usage();
      return 0;
    } else {
      usage();
      return 1;
    }
  }

  if (fleet.empty()) {
    usage();
    return 1;
  }
  if (focalId.empty()) {
    focalId = fleet.front().id;
  }

  std::string error;
  const mmlp::GeoBBox bbox = mmlp::bboxFromVehicles(fleet, padding);
  mmlp::GraphContext ctx;
  std::cerr << "[mmlp_predict] loading region lon[" << bbox.minLon << "," << bbox.maxLon
            << "] lat[" << bbox.minLat << "," << bbox.maxLat << "] ...\n";
  if (!mmlp::loadGraphContextRegion(graphPath, bbox, ctx, &error)) {
    std::cerr << "load failed: " << error << "\n";
    return 2;
  }
  std::cerr << "[mmlp_predict] region nodes=" << ctx.graph.nodes().size()
            << " edges=" << ctx.graph.edges().size() << "\n";

  const std::vector<mmlp::VehicleHistory> histories;
  const mmlp::PredictParam param;

  if (allPairs) {
    const auto meetings = mmlp::predictMeetings(fleet, histories, ctx, param);
    std::cout << "[\n";
    for (std::size_t i = 0; i < meetings.size(); ++i) {
      printMeetingJson(std::cout, meetings[i], true);
      if (i + 1 < meetings.size()) {
        std::cout << ",";
      }
      std::cout << "\n";
    }
    std::cout << "]\n";
    return 0;
  }

  const auto best = mmlp::predictBestMeetingFor(focalId, fleet, histories, ctx, param);
  printBest(best);
  return best.found ? 0 : 3;
}
