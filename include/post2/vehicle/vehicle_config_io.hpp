#pragma once

#include <string>

#include "post2/vehicle/vehicle.hpp"

namespace post2::vehicle {

VehicleConfig default_vehicle_config();

bool validate_vehicle_config(const VehicleConfig& config, std::string* error);
std::string vehicle_config_summary(const VehicleConfig& config);

std::string vehicle_config_to_text(const VehicleConfig& config);
bool vehicle_config_from_text(const std::string& text, VehicleConfig* config, std::string* error);

bool load_vehicle_config_file(const std::string& path, VehicleConfig* config, std::string* error);
bool save_vehicle_config_file(const std::string& path, const VehicleConfig& config, std::string* error);

} // namespace post2::vehicle
