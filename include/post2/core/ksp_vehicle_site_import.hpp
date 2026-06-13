#pragma once

#include <string>

#include "post2/core/types.hpp"

namespace post2::core {

struct KspVehicleSiteImport {
    post2::vehicle::VehicleConfig vehicle;
    LaunchSiteConfig launch_site;
    bool has_launch_site = false;

    double earth_radius_m = kEarthRadiusM;
    double earth_mu_m3s2 = kEarthMuM3S2;
    double earth_rotation_rad_per_s = kEarthRotationRadPerS;
    bool has_earth_radius_m = false;
    bool has_earth_mu_m3s2 = false;
    bool has_earth_rotation_rad_per_s = false;

    std::string source_format;
};

bool ksp_vehicle_site_import_from_json(
    const std::string& text,
    const post2::vehicle::AeroConfig& preserved_aero,
    KspVehicleSiteImport* result,
    std::string* error);

bool load_ksp_vehicle_site_import_file(
    const std::string& path,
    const post2::vehicle::AeroConfig& preserved_aero,
    KspVehicleSiteImport* result,
    std::string* error);

void apply_ksp_vehicle_site_import(CaseConfig* config, const KspVehicleSiteImport& imported);
void apply_ksp_vehicle_site_import(SimulationConfig* config, const KspVehicleSiteImport& imported);

} // namespace post2::core
