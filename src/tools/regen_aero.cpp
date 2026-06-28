// post2_regen_aero -- headless equivalent of the GUI "Vehicle -> Import KSP"
// action: re-import a KSP vehicle/site into one or more case files and
// regenerate their per-staging aerodynamic tables (writing the table CSVs next
// to each case), then save the cases. Shares post2::core::generate_case_aero_tables
// with the GUI so the output is identical.
//
//   post2_regen_aero <vehicle_launchsite.json> <case.json> [case2.json ...]
//
// Pass "--geometry-only" (or "-") as the source to skip the vehicle re-import
// and regenerate the tables from each case's EXISTING aero geometry only. This
// preserves edits to the vehicle (engine_count, ignition_count_options, masses)
// while upgrading old cases to the current per-staging table set (incl. the
// single-stage booster-alone [i,i] tables that older imports never wrote).

#include <filesystem>
#include <iostream>
#include <string>

#include "post2/core/aero_table_build.hpp"
#include "post2/core/case_config_io.hpp"
#include "post2/core/ksp_vehicle_site_import.hpp"

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "usage: post2_regen_aero <vehicle_launchsite.json|--geometry-only> "
                     "<case.json> [case2.json ...]\n";
        return 2;
    }

    const std::string source = argv[1];
    const bool geometry_only = (source == "--geometry-only" || source == "-");
    int failures = 0;

    for (int i = 2; i < argc; ++i) {
        const std::string case_path = argv[i];
        post2::core::CaseConfig config;
        std::string error;

        if (!post2::core::load_case_config_file(case_path, &config, &error)) {
            std::cerr << case_path << ": load failed: " << error << "\n";
            ++failures;
            continue;
        }

        // Geometry-only: leave the vehicle untouched and regenerate tables from
        // the case's existing aero geometry (empty import). Otherwise re-import
        // the vehicle + launch site from `source` first.
        post2::core::KspVehicleSiteImport imported;
        if (!geometry_only) {
            if (!post2::core::load_ksp_vehicle_site_import_file(
                    source, config.vehicle.aero, &imported, &error)) {
                std::cerr << source << ": import failed: " << error << "\n";
                return 1;
            }
            post2::core::apply_ksp_vehicle_site_import(&config, imported);
        }

        // Write the table CSVs next to the case (absolute path so the simulator
        // finds them regardless of working directory), matching the GUI.
        std::filesystem::path out_dir =
            std::filesystem::absolute(case_path).parent_path();
        std::string output_dir = out_dir.string();
        if (!output_dir.empty() && output_dir.back() != '/' && output_dir.back() != '\\') {
            output_dir += static_cast<char>(std::filesystem::path::preferred_separator);
        }

        if (!post2::core::generate_case_aero_tables(&config, imported, output_dir, &error)) {
            std::cerr << case_path << ": table generation failed: " << error << "\n";
            ++failures;
            continue;
        }

        if (!post2::core::save_case_config_file(case_path, config, &error)) {
            std::cerr << case_path << ": save failed: " << error << "\n";
            ++failures;
            continue;
        }

        std::cout << case_path << ": " << config.vehicle.aero.stage_tables.size()
                  << " aero table(s)";
        for (const auto& t : config.vehicle.aero.stage_tables) {
            std::cout << "  [" << t.activate_at_min_attached_stage << "..";
            if (t.max_attached_stage < 0) {
                std::cout << "top";  // open-top upper-stack / full table
            } else {
                std::cout << t.max_attached_stage;  // bounded single-stage / sub-stack
            }
            std::cout << " S=" << t.reference_area_m2 << " m2]";
        }
        std::cout << "\n";
    }

    return failures == 0 ? 0 : 1;
}
