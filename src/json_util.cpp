#include "mmlp/json_util.hpp"

#include "mmlp/geo.hpp"

#include <cctype>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace mmlp {

namespace {

std::string decodeJsonString(const std::string& raw) {
  std::string out;
  out.reserve(raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i) {
    if (raw[i] != '\\' || i + 1 >= raw.size()) {
      out.push_back(raw[i]);
      continue;
    }
    const char next = raw[i + 1];
    if (next == 'u' && i + 5 < raw.size()) {
      unsigned int code = 0;
      bool ok = true;
      for (int j = 0; j < 4; ++j) {
        const char h = raw[i + 2 + static_cast<std::size_t>(j)];
        code <<= 4;
        if (h >= '0' && h <= '9') {
          code |= static_cast<unsigned>(h - '0');
        } else if (h >= 'a' && h <= 'f') {
          code |= static_cast<unsigned>(h - 'a' + 10);
        } else if (h >= 'A' && h <= 'F') {
          code |= static_cast<unsigned>(h - 'A' + 10);
        } else {
          ok = false;
          break;
        }
      }
      if (ok && code <= 0x10FFFF) {
        if (code <= 0x7F) {
          out.push_back(static_cast<char>(code));
        } else if (code <= 0x7FF) {
          out.push_back(static_cast<char>(0xC0 | (code >> 6)));
          out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
        } else {
          out.push_back(static_cast<char>(0xE0 | (code >> 12)));
          out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
          out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
        }
        i += 5;
        continue;
      }
    }
    if (next == '"' || next == '\\' || next == '/') {
      out.push_back(next);
      i += 1;
      continue;
    }
    if (next == 'n') {
      out.push_back('\n');
      i += 1;
      continue;
    }
    if (next == 't') {
      out.push_back('\t');
      i += 1;
      continue;
    }
    out.push_back(raw[i]);
  }
  return out;
}

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
  return decodeJsonString(json.substr(start, end - start));
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

bool extractNumberOutsideArray(const std::string& json, const std::string& key,
                               const std::string& arrayKey, double& out) {
  const std::string apat = "\"" + arrayKey + "\"";
  std::size_t arrBegin = std::string::npos;
  std::size_t arrEnd = std::string::npos;
  const auto ak = json.find(apat);
  if (ak != std::string::npos) {
    arrBegin = json.find('[', ak);
    if (arrBegin != std::string::npos) {
      int depth = 0;
      for (std::size_t i = arrBegin; i < json.size(); ++i) {
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
    }
  }

  const std::string pat = "\"" + key + "\"";
  for (std::size_t search = 0; search < json.size();) {
    const auto pos = json.find(pat, search);
    if (pos == std::string::npos) {
      return false;
    }
    if (arrBegin != std::string::npos && pos > arrBegin && pos < arrEnd) {
      search = pos + pat.size();
      continue;
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
  return false;
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

namespace {

bool utcPartsToUnix(int year, int month, int day, int hour, int minute, int second,
                    int64_t& outUnix) {
  if (month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 ||
      minute > 59 || second < 0 || second > 59) {
    return false;
  }

  int y = year;
  int m = month;
  if (m <= 2) {
    y -= 1;
    m += 12;
  }
  const int era = y / 400;
  const int yoe = y - era * 400;
  const int doy = (153 * (m - 3) + 2) / 5 + day - 1;
  const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int64_t days = static_cast<int64_t>(era) * 146097 + doe - 719468;
  outUnix = days * 86400 + hour * 3600 + minute * 60 + second;
  return true;
}

}  // namespace

bool parseIso8601Utc(const std::string& text, int64_t& outUnix, std::string* error) {
  if (text.empty()) {
    if (error) {
      *error = "empty datetime";
    }
    return false;
  }

  std::string s = text;
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
    s.pop_back();
  }
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
    s.erase(s.begin());
  }

  // Strip trailing Z / timezone — we treat the wall clock as UTC.
  if (!s.empty() && (s.back() == 'Z' || s.back() == 'z')) {
    s.pop_back();
  }
  // Drop fractional seconds if present.
  const auto dot = s.find('.');
  if (dot != std::string::npos) {
    s.resize(dot);
  }

  std::tm tm{};
  auto tryParse = [&](const char* fmt) -> bool {
    std::istringstream iss(s);
    iss >> std::get_time(&tm, fmt);
    return !iss.fail();
  };
  bool ok = false;
  if (s.size() >= 16 && s[10] == 'T') {
    ok = tryParse("%Y-%m-%dT%H:%M:%S") || tryParse("%Y-%m-%dT%H:%M");
  } else if (s.size() >= 16 && s[10] == ' ') {
    ok = tryParse("%Y-%m-%d %H:%M:%S") || tryParse("%Y-%m-%d %H:%M");
  }
  if (!ok) {
    if (error) {
      *error = "arriveBy format: YYYY-MM-DDTHH:MM[:SS]Z or YYYY-MM-DD HH:MM[:SS]";
    }
    return false;
  }

  if (!utcPartsToUnix(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
                      tm.tm_sec, outUnix)) {
    if (error) {
      *error = "arriveBy out of range: " + text;
    }
    return false;
  }
  return true;
}

ArrivalSortBy parseArrivalSortBy(const std::string& text) {
  if (text == "eta" || text == "arrival" || text == "arrive") {
    return ArrivalSortBy::ETA;
  }
  if (text == "distance" || text == "route" || text == "routeDistance") {
    return ArrivalSortBy::DISTANCE;
  }
  return ArrivalSortBy::DURATION;
}

std::string arrivalSortByLabel(ArrivalSortBy sortBy) {
  switch (sortBy) {
    case ArrivalSortBy::ETA:
      return "eta";
    case ArrivalSortBy::DISTANCE:
      return "distance";
    default:
      return "duration";
  }
}

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

  std::string timeStr = extractString(json, "time");
  if (timeStr.empty()) {
    timeStr = extractString(json, "observedAt");
  }
  if (timeStr.empty()) {
    if (error) {
      *error = "missing time (ISO UTC, e.g. 2026-06-01T10:00:00Z)";
    }
    return false;
  }
  std::string timeErr;
  if (!parseIso8601Utc(timeStr, vehicle.timestamp, &timeErr)) {
    if (error) {
      *error = timeErr;
    }
    return false;
  }

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

  const std::string meetTime = formatUtcFromUnix(b.meetTime);

  std::ostringstream os;
  os << std::fixed;
  os << "{\"found\":true"
     << ",\"focal\":\"" << escapeJson(b.focalVehicleId) << "\""
     << ",\"partner\":\"" << escapeJson(b.partnerVehicleId) << "\""
     << ",\"meetTime\":\"" << meetTime << "\""
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

bool isDestinationArrivalCommand(const std::string& json) {
  if (json.find("destination_arrival") != std::string::npos) {
    return true;
  }
  if (json.find("destinationArrival") != std::string::npos) {
    return true;
  }
  if (json.find("\"arriveBy\"") != std::string::npos && json.find("\"lat\"") != std::string::npos) {
    return true;
  }
  return false;
}

bool isMeetWithLeadCommand(const std::string& json) {
  if (isDestinationArrivalCommand(json)) {
    return false;
  }
  if (json.find("\"meet_with_lead\"") != std::string::npos) {
    return true;
  }
  if (json.find("\"meetWithLead\"") != std::string::npos) {
    return true;
  }
  return json.find("\"vehicles\"") != std::string::npos && json.find('[') != std::string::npos &&
         json.find("\"id\"") != std::string::npos && json.find("\"lat\"") != std::string::npos;
}

bool parseDestinationArrivalJson(const std::string& json, DestinationQuery& dest,
                                 std::vector<VehicleInfo>* vehicles,
                                 std::vector<VehicleHistory>* histories, std::string* error) {
  double v = 0.0;
  if (!extractNumberOutsideArray(json, "lat", "vehicles", v)) {
    if (error) {
      *error = "missing lat";
    }
    return false;
  }
  dest.lat = v;

  if (!extractNumberOutsideArray(json, "lon", "vehicles", v)) {
    if (error) {
      *error = "missing lon";
    }
    return false;
  }
  dest.lon = v;

  const std::string arriveBy = extractString(json, "arriveBy");
  if (arriveBy.empty()) {
    if (error) {
      *error = "missing arriveBy (ISO UTC, e.g. 2026-06-01T12:00:00Z)";
    }
    return false;
  }
  std::string timeErr;
  if (!parseIso8601Utc(arriveBy, dest.arriveByUnix, &timeErr)) {
    if (error) {
      *error = timeErr;
    }
    return false;
  }
  if (dest.arriveByUnix <= 0) {
    if (error) {
      *error = "arriveBy must be a valid future/past UTC time";
    }
    return false;
  }

  const std::string type = extractString(json, "type");
  dest.type = (type == "train" || type == "TRAIN") ? VehicleType::TRAIN : VehicleType::TRUCK;

  const std::string sortBy = extractString(json, "sortBy");
  if (!sortBy.empty()) {
    dest.sortBy = parseArrivalSortBy(sortBy);
  }

  if (vehicles != nullptr && json.find("\"vehicles\"") != std::string::npos) {
    std::vector<VehicleHistory> parsedHist;
    if (!parseVehiclesJson(json, *vehicles, parsedHist, error)) {
      return false;
    }
    if (histories != nullptr) {
      *histories = std::move(parsedHist);
    }
  }

  return true;
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

std::string formatDestinationArrivalJson(const DestinationArrivalSummary& summary) {
  const std::string arriveBy = formatUtcFromUnix(static_cast<double>(summary.arriveByUnix));
  const std::string sortLabel = arrivalSortByLabel(summary.sortBy);
  std::ostringstream os;
  os << std::fixed << "{\"destination\":{"
     << "\"lat\":" << std::setprecision(6) << summary.lat << ",\"lon\":" << summary.lon
     << ",\"arriveBy\":\"" << arriveBy << "\",\"locationId\":\"" << escapeJson(summary.locationId)
     << "\"}"
     << ",\"sortBy\":\"" << sortLabel << "\",\"vehicles\":[";
  for (std::size_t i = 0; i < summary.vehicles.size(); ++i) {
    if (i > 0) {
      os << ',';
    }
    const auto& row = summary.vehicles[i];
    const std::string eta = formatUtcFromUnix(row.etaUnix);
    os << "{\"reachable\":" << (row.reachable ? "true" : "false")
       << ",\"vehicleId\":\"" << escapeJson(row.vehicleId) << "\""
       << ",\"eta\":\"" << eta << "\""
       << ",\"travelDurationSec\":" << std::setprecision(2) << row.travelDurationSec
       << ",\"route\":";
    appendRouteJson(os, row.route);
    os << ",\"routeDistanceM\":" << std::setprecision(1) << row.routeDistanceM << "}";
  }
  os << "]}";
  return os.str();
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
    const std::string meetTime = formatUtcFromUnix(m.meetTime);
    os << std::fixed << "{\"found\":true"
       << ",\"partner\":\"" << escapeJson(m.partnerVehicleId) << "\""
       << ",\"meetTime\":\"" << meetTime << "\""
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
