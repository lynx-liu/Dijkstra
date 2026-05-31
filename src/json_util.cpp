#include "mmlp/json_util.hpp"

#include "mmlp/geo.hpp"

#include <cctype>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace mmlp {

namespace {

std::string extractString(const std::string& json, const std::string& key) {
  const std::string pat = "\"" + key + "\"";
  const auto pos = json.find(pat);
  if (pos == std::string::npos) {
    return {};
  }
  const auto colon = json.find(':', pos + pat.size());
  if (colon == std::string::npos) {
    return {};
  }
  auto start = json.find('"', colon + 1);
  if (start == std::string::npos) {
    return {};
  }
  ++start;
  const auto end = json.find('"', start);
  if (end == std::string::npos) {
    return {};
  }
  return json.substr(start, end - start);
}

bool extractNumber(const std::string& json, const std::string& key, double& out) {
  const std::string pat = "\"" + key + "\"";
  const auto pos = json.find(pat);
  if (pos == std::string::npos) {
    return false;
  }
  const auto colon = json.find(':', pos + pat.size());
  if (colon == std::string::npos) {
    return false;
  }
  std::size_t i = colon + 1;
  while (i < json.size() && std::isspace(static_cast<unsigned char>(json[i]))) {
    ++i;
  }
  std::size_t j = i;
  while (j < json.size() &&
         (std::isdigit(static_cast<unsigned char>(json[j])) || json[j] == '.' || json[j] == '-' ||
          json[j] == '+' || json[j] == 'e' || json[j] == 'E')) {
    ++j;
  }
  if (j == i) {
    return false;
  }
  out = std::stod(json.substr(i, j - i));
  return true;
}

std::string formatUtcFromUnix(double unixSec) {
  const auto sec = static_cast<std::time_t>(std::floor(unixSec));
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

std::string escapeJson(const std::string& s) {
  std::string out;
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  return out;
}

}  // namespace

bool parseVehicleJson(const std::string& json, VehicleInfo& vehicle, VehicleHistory* history,
                      std::string* error) {
  vehicle.id = extractString(json, "id");
  if (vehicle.id.empty()) {
    if (error) {
      *error = "missing id";
    }
    return false;
  }

  double v = 0.0;
  if (!extractNumber(json, "lat", v)) {
    if (error) {
      *error = "missing lat";
    }
    return false;
  }
  vehicle.lat = v;

  if (!extractNumber(json, "lon", v)) {
    if (error) {
      *error = "missing lon";
    }
    return false;
  }
  vehicle.lon = v;

  if (!extractNumber(json, "speed", v)) {
    if (error) {
      *error = "missing speed";
    }
    return false;
  }
  vehicle.speed = v;

  if (!extractNumber(json, "timestamp", v)) {
    if (error) {
      *error = "missing timestamp";
    }
    return false;
  }
  vehicle.timestamp = static_cast<int64_t>(std::llround(v));

  const std::string type = extractString(json, "type");
  vehicle.type = (type == "train" || type == "TRAIN") ? VehicleType::TRAIN : VehicleType::TRUCK;
  vehicle.heading = 0.0;

  if (history != nullptr) {
    history->id = vehicle.id;
    history->speedSamples.clear();
    const std::string histKey = "\"history\"";
    const auto hpos = json.find(histKey);
    if (hpos != std::string::npos) {
      const auto lb = json.find('[', hpos);
      const auto rb = json.find(']', lb);
      if (lb != std::string::npos && rb != std::string::npos && rb > lb) {
        std::string inner = json.substr(lb + 1, rb - lb - 1);
        std::stringstream ss(inner);
        std::string token;
        while (std::getline(ss, token, ',')) {
          if (!token.empty()) {
            history->speedSamples.push_back(std::stod(token));
          }
        }
      }
    }
  }

  return true;
}

namespace {

void appendRouteJson(std::ostringstream& os, const RoutePolyline& route) {
  os << "[";
  for (std::size_t i = 0; i < route.points.size(); ++i) {
    if (i > 0) {
      os << ',';
    }
    os << std::fixed << std::setprecision(6) << '[' << route.points[i].lat << ','
       << route.points[i].lon << ']';
  }
  os << "]";
}

double polylineLengthMeters(const RoutePolyline& route) {
  double sum = 0.0;
  for (std::size_t i = 1; i < route.points.size(); ++i) {
    sum += haversineMeters(route.points[i - 1], route.points[i]);
  }
  return sum;
}

void appendRouteMetricsJson(std::ostringstream& os, const RoutePolyline& routeSelf,
                            const RoutePolyline& routePartner) {
  os << std::fixed << std::setprecision(1);
  os << ",\"routeDistanceSelfM\":" << polylineLengthMeters(routeSelf)
     << ",\"routeDistancePartnerM\":" << polylineLengthMeters(routePartner);
}

}  // namespace

std::string formatFocalBestMeetingJson(const FocalBestMeeting& b) {
  if (!b.found) {
    std::ostringstream os;
    os << "{\"found\":false,\"focal\":\"" << escapeJson(b.focalVehicleId) << "\"";
    if (!b.partnerVehicleId.empty()) {
      os << ",\"partner\":\"\"";
    }
    os << "}";
    return os.str();
  }

  const int64_t meetUnix = static_cast<int64_t>(std::llround(b.meetTime));
  const std::string meetUtc = formatUtcFromUnix(b.meetTime);

  std::ostringstream os;
  os << std::fixed;
  os << "{\"found\":true"
     << ",\"focal\":\"" << escapeJson(b.focalVehicleId) << "\""
     << ",\"partner\":\"" << escapeJson(b.partnerVehicleId) << "\""
     << ",\"meetTimeUnix\":" << meetUnix << ",\"meetTimeUtc\":\"" << meetUtc << "\""
     << ",\"meetDurationSec\":" << std::setprecision(2) << b.meetDuration << ",\"lat\":"
     << std::setprecision(6) << b.lat << ",\"lon\":" << b.lon << ",\"locationId\":\""
     << escapeJson(b.locationId) << "\",\"routeSelf\":";
  appendRouteJson(os, b.routeSelf);
  os << ",\"routePartner\":";
  appendRouteJson(os, b.routePartner);
  appendRouteMetricsJson(os, b.routeSelf, b.routePartner);
  os << "}";
  return os.str();
}

bool isPreloadCommand(const std::string& json) {
  return json.find("\"action\"") != std::string::npos &&
         json.find("preload") != std::string::npos;
}

bool isMeetWithLeadCommand(const std::string& json) {
  if (json.find("\"meet_with_lead\"") != std::string::npos) {
    return true;
  }
  if (json.find("\"meetWithLead\"") != std::string::npos) {
    return true;
  }
  return json.find("\"vehicles\"") != std::string::npos && json.find('[') != std::string::npos &&
         json.find("\"id\"") != std::string::npos && json.find("\"lat\"") != std::string::npos;
}

namespace {

std::vector<std::string> splitTopLevelObjects(const std::string& arrayContent) {
  std::vector<std::string> objects;
  int arrDepth = 0;
  int objDepth = 0;
  std::size_t start = std::string::npos;
  for (std::size_t i = 0; i < arrayContent.size(); ++i) {
    const char c = arrayContent[i];
    if (c == '[') {
      ++arrDepth;
      continue;
    }
    if (c == ']') {
      --arrDepth;
      continue;
    }
    if (c == '{') {
      if (arrDepth == 1 && objDepth == 0) {
        start = i;
      }
      ++objDepth;
      continue;
    }
    if (c == '}') {
      --objDepth;
      if (arrDepth == 1 && objDepth == 0 && start != std::string::npos) {
        objects.push_back(arrayContent.substr(start, i - start + 1));
        start = std::string::npos;
      }
    }
  }
  return objects;
}

}  // namespace

bool parseVehiclesJson(const std::string& json, std::vector<VehicleInfo>& vehicles,
                       std::vector<VehicleHistory>& histories, std::string* error) {
  const std::string key = "\"vehicles\"";
  const auto keyPos = json.find(key);
  if (keyPos == std::string::npos) {
    if (error) {
      *error = "missing vehicles array";
    }
    return false;
  }
  const auto arrStart = json.find('[', keyPos);
  if (arrStart == std::string::npos) {
    if (error) {
      *error = "malformed vehicles array";
    }
    return false;
  }

  int depth = 0;
  std::size_t arrEnd = std::string::npos;
  for (std::size_t i = arrStart; i < json.size(); ++i) {
    if (json[i] == '[') {
      ++depth;
    } else if (json[i] == ']') {
      --depth;
      if (depth == 0) {
        arrEnd = i;
        break;
      }
    }
  }
  if (arrEnd == std::string::npos) {
    if (error) {
      *error = "unclosed vehicles array";
    }
    return false;
  }

  const std::string arrayBody = json.substr(arrStart, arrEnd - arrStart + 1);
  const auto objects = splitTopLevelObjects(arrayBody);
  if (objects.empty()) {
    if (error) {
      *error = "vehicles array is empty";
    }
    return false;
  }

  vehicles.clear();
  histories.clear();
  vehicles.reserve(objects.size());
  histories.reserve(objects.size());

  for (const auto& obj : objects) {
    VehicleInfo vehicle;
    VehicleHistory history;
    std::string parseErr;
    if (!parseVehicleJson(obj, vehicle, &history, &parseErr)) {
      if (error) {
        *error = "invalid vehicle: " + parseErr;
      }
      return false;
    }
    vehicles.push_back(vehicle);
    if (!history.speedSamples.empty()) {
      histories.push_back(history);
    }
  }
  return true;
}

std::string formatMeetingsWithLeadJson(const std::vector<FocalBestMeeting>& meetings) {
  std::ostringstream os;
  os << "{\"focal\":\"";
  if (!meetings.empty()) {
    os << escapeJson(meetings.front().focalVehicleId);
  }
  os << "\",\"meetings\":[";
  for (std::size_t i = 0; i < meetings.size(); ++i) {
    if (i > 0) {
      os << ',';
    }
    const auto& m = meetings[i];
    if (!m.found) {
      os << "{\"found\":false,\"partner\":\"" << escapeJson(m.partnerVehicleId) << "\"}";
      continue;
    }
    const int64_t meetUnix = static_cast<int64_t>(std::llround(m.meetTime));
    const std::string meetUtc = formatUtcFromUnix(m.meetTime);
    os << std::fixed << "{\"found\":true"
       << ",\"partner\":\"" << escapeJson(m.partnerVehicleId) << "\""
       << ",\"meetTimeUnix\":" << meetUnix << ",\"meetTimeUtc\":\"" << meetUtc << "\""
       << ",\"meetDurationSec\":" << std::setprecision(2) << m.meetDuration << ",\"lat\":"
       << std::setprecision(6) << m.lat << ",\"lon\":" << m.lon << ",\"locationId\":\""
       << escapeJson(m.locationId) << "\",\"routeSelf\":";
    appendRouteJson(os, m.routeSelf);
    os << ",\"routePartner\":";
    appendRouteJson(os, m.routePartner);
    appendRouteMetricsJson(os, m.routeSelf, m.routePartner);
    os << "}";
  }
  os << "]}";
  return os.str();
}

bool parsePreloadJson(const std::string& json, GeoBBox& bbox, std::string* error) {
  double v = 0.0;
  if (extractNumber(json, "minLon", v)) {
    bbox.minLon = v;
    if (!extractNumber(json, "minLat", bbox.minLat) || !extractNumber(json, "maxLon", bbox.maxLon) ||
        !extractNumber(json, "maxLat", bbox.maxLat)) {
      if (error) {
        *error = "preload needs minLon,minLat,maxLon,maxLat";
      }
      return false;
    }
    return true;
  }
  const std::string bboxStr = extractString(json, "bbox");
  if (bboxStr.empty()) {
    if (error) {
      *error = "preload needs bbox or minLon..maxLat";
    }
    return false;
  }
  std::stringstream ss(bboxStr);
  std::string a, b, c, d;
  if (!std::getline(ss, a, ',') || !std::getline(ss, b, ',') || !std::getline(ss, c, ',') ||
      !std::getline(ss, d, ',')) {
    if (error) {
      *error = "bbox format: minLon,minLat,maxLon,maxLat";
    }
    return false;
  }
  bbox.minLon = std::stod(a);
  bbox.minLat = std::stod(b);
  bbox.maxLon = std::stod(c);
  bbox.maxLat = std::stod(d);
  return true;
}

std::string formatOkJson(const std::string& message) {
  return "{\"ok\":true,\"message\":\"" + escapeJson(message) + "\"}";
}

std::string formatErrorJson(const std::string& message) {
  return "{\"error\":\"" + escapeJson(message) + "\"}";
}

}  // namespace mmlp
