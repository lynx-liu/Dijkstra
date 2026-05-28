#pragma once

#include "mmlp/fleet_service.hpp"
#include "mmlp/types.hpp"

#include <string>

namespace mmlp {

// Minimal JSON parse/format for service I/O (no external JSON library).
bool parseVehicleJson(const std::string& json, VehicleInfo& vehicle, VehicleHistory* history,
                      std::string* error = nullptr);

bool parsePreloadJson(const std::string& json, GeoBBox& bbox, std::string* error = nullptr);

bool isPreloadCommand(const std::string& json);

bool isMeetWithLeadCommand(const std::string& json);

bool parseVehiclesJson(const std::string& json, std::vector<VehicleInfo>& vehicles,
                       std::vector<VehicleHistory>& histories, std::string* error = nullptr);

std::string formatFocalBestMeetingJson(const FocalBestMeeting& result);
std::string formatMeetingsWithLeadJson(const std::vector<FocalBestMeeting>& meetings);
std::string formatOkJson(const std::string& message);
std::string formatErrorJson(const std::string& message);

}  // namespace mmlp
