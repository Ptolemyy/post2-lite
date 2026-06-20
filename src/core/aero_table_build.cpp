#include "post2/core/aero_table_build.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "post2/aero/aero_model.hpp"
#include "post2/aero/aero_table.hpp"

namespace post2::core {

bool generate_case_aero_tables(CaseConfig* config,
                               const KspVehicleSiteImport& imported,
                               const std::string& output_dir,
                               std::string* error)
{
    if (!config) {
        if (error) {
            *error = "null case config";
        }
        return false;
    }
    post2::vehicle::AeroConfig& aero = config->vehicle.aero;

    const double full_length = aero.body_length_m > 0.0
        ? aero.body_length_m
        : (imported.has_vehicle_length ? imported.vehicle_length_m : 70.0);
    const double ref_diameter = aero.ref_diameter_m > 0.0 ? aero.ref_diameter_m : 5.2;   // fairing
    const double base_diameter = aero.base_diameter_m > 0.0 ? aero.base_diameter_m : 3.66;  // core

    std::string local_error;
    // Builds + writes one configuration table and returns its descriptor.
    const auto make_table = [&](int activate_at, const char* file, double length,
                                post2::vehicle::AeroStageTable* out) -> bool {
        post2::aero::AeroGeometry geom;
        geom.ref_diameter_m = ref_diameter;
        geom.total_length_m = length;
        geom.nose_length_m = std::min(0.18 * full_length, length);  // fairing/nose region
        geom.base_diameter_m = base_diameter;
        geom.power_on = true;
        post2::aero::finalize_geometry(&geom);
        const post2::aero::AeroTable table = post2::aero::generate_aero_table(geom);
        const std::string path = output_dir + file;
        if (!post2::aero::write_aero_table_csv(path, table, &local_error)) {
            return false;
        }
        out->activate_at_min_attached_stage = activate_at;
        out->table_path = path;
        out->reference_area_m2 = geom.ref_area_m2;
        out->ref_diameter_m = geom.ref_diameter_m;
        out->body_length_m = geom.total_length_m;
        out->nose_length_m = geom.nose_length_m;
        out->base_diameter_m = geom.base_diameter_m;
        return true;
    };

    std::vector<post2::vehicle::AeroStageTable> tables;

    // Level 0: full stack (all stages attached).
    post2::vehicle::AeroStageTable full;
    if (!make_table(0, "aero_table_full.csv", full_length, &full)) {
        if (error) {
            *error = local_error;
        }
        return false;
    }
    tables.push_back(full);

    // Level 1: upper stack after the booster separates. Generated whenever the
    // vehicle has at least two propulsive stages -- using the measured upper-stack
    // length when the import provided one, else a slenderness estimate.
    int propulsive_stages = 0;
    for (const auto& stage : config->vehicle.stages) {
        if (stage.engine.enabled && stage.engine.thrust_vac_n > 0.0) {
            ++propulsive_stages;
        }
    }
    if (propulsive_stages >= 2) {
        double upper_length = full_length;
        if (imported.has_upper_length && imported.upper_length_m > 0.0 &&
            imported.upper_length_m < full_length - 1.0) {
            upper_length = imported.upper_length_m;  // measured from parts
        } else {
            // Estimate: the upper stack (with fairing) is roughly 40% of the full
            // stack length for a typical two-stage launcher.
            upper_length = std::min(0.4 * full_length, full_length - 1.0);
        }
        post2::vehicle::AeroStageTable upper;
        if (!make_table(1, "aero_table_stage2.csv", upper_length, &upper)) {
            if (error) {
                *error = local_error;
            }
            return false;
        }
        tables.push_back(upper);
    }

    aero.enabled = true;
    aero.use_table = true;
    aero.stage_tables = tables;
    aero.aero_table_path = full.table_path;  // legacy/primary pointer
    aero.reference_area_m2 = full.reference_area_m2;
    aero.ref_diameter_m = full.ref_diameter_m;
    aero.body_length_m = full.body_length_m;
    aero.nose_length_m = full.nose_length_m;
    aero.base_diameter_m = full.base_diameter_m;

    // Enable real aerodynamics and the realistic atmosphere on every phase.
    for (auto& phase : config->phases) {
        phase.force_models.aerodynamic = true;
        phase.force_models.atmosphere_model.type = "us_standard_1976";
    }
    return true;
}

} // namespace post2::core
