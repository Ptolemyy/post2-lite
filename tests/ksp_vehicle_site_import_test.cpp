#include "post2/core/ksp_vehicle_site_import.hpp"
#include "post2/vehicle/runtime_state.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool close(double lhs, double rhs, double tolerance = 1.0e-9)
{
    return std::abs(lhs - rhs) <= tolerance;
}

bool expect_close(const char* label, double actual, double expected, double tolerance = 1.0e-9)
{
    if (!close(actual, expected, tolerance)) {
        std::cerr << label << " expected " << expected << " got " << actual << '\n';
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "fixture path argument required\n";
        return 2;
    }

    post2::vehicle::AeroConfig preserved_aero;
    preserved_aero.enabled = true;
    preserved_aero.reference_area_m2 = 42.0;
    preserved_aero.cd = 0.7;
    preserved_aero.cl = 0.1;
    preserved_aero.aero_table_path = "preserved.csv";

    post2::core::KspVehicleSiteImport imported;
    std::string error;
    if (!post2::core::load_ksp_vehicle_site_import_file(argv[1], preserved_aero, &imported, &error)) {
        std::cerr << "import failed: " << error << '\n';
        return 1;
    }

    const auto& vehicle = imported.vehicle;
    if (vehicle.name != "Wrapped Import Test" ||
        vehicle.stages.size() != 3 ||
        vehicle.stages[0].name != "booster" ||
        vehicle.stages[1].name != "upper_stack" ||
        vehicle.stages[2].name != "payload") {
        std::cerr << "unexpected imported vehicle/stage names\n";
        return 1;
    }

    if (!vehicle.stages[0].active || vehicle.stages[1].active || vehicle.stages[2].active) {
        std::cerr << "unexpected imported stage active flags\n";
        return 1;
    }

    if (!expect_close("booster dry mass", vehicle.stages[0].dry_mass_kg, 210.0) ||
        !expect_close("booster propellant", vehicle.stages[0].tanks.front().initial_kg, 1000.0) ||
        !expect_close("booster thrust", vehicle.stages[0].engine.thrust_vac_n, 5000.0) ||
        !expect_close("upper dry mass", vehicle.stages[1].dry_mass_kg, 100.0) ||
        !expect_close("upper thrust", vehicle.stages[1].engine.thrust_vac_n, 1000.0) ||
        !expect_close("payload gross mass", vehicle.stages[2].dry_mass_kg, 65.0) ||
        !expect_close("payload metric", post2::vehicle::payload_stage_dry_mass_kg(vehicle), 65.0)) {
        return 1;
    }

    if (!vehicle.aero.enabled ||
        !expect_close("aero reference area", vehicle.aero.reference_area_m2, 42.0) ||
        !expect_close("aero cd", vehicle.aero.cd, 0.7) ||
        !expect_close("aero cl", vehicle.aero.cl, 0.1) ||
        vehicle.aero.aero_table_path != "preserved.csv") {
        std::cerr << "aero settings were not preserved\n";
        return 1;
    }

    constexpr double kPi = 3.141592653589793238462643383279502884;
    if (!imported.has_launch_site ||
        !imported.has_earth_radius_m ||
        !imported.has_earth_mu_m3s2 ||
        !imported.has_earth_rotation_rad_per_s ||
        !expect_close("launch lat", imported.launch_site.latitude_deg, 12.5) ||
        !expect_close("launch lon", imported.launch_site.longitude_deg, -45.25) ||
        !expect_close("launch alt", imported.launch_site.altitude_m, 123.0) ||
        !expect_close("earth radius", imported.earth_radius_m, 6378137.0) ||
        !expect_close("earth mu", imported.earth_mu_m3s2, 398600441800000.0) ||
        !expect_close("earth rotation", imported.earth_rotation_rad_per_s, 2.0 * kPi / 86164.0905)) {
        return 1;
    }

    return 0;
}
