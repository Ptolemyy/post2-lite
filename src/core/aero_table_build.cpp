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
    // Builds + writes one configuration table for the attached stage range
    // [lo, hi] (hi < 0 == open to the top of the stack) and returns its
    // descriptor.
    const auto make_table = [&](int lo, int hi, const std::string& file, double length,
                                double ref_d, double base_d,
                                post2::vehicle::AeroStageTable* out,
                                const post2::aero::GridFinSpec& fins = {}) -> bool {
        post2::aero::AeroGeometry geom;
        geom.ref_diameter_m = ref_d;
        geom.total_length_m = std::max(length, 1.0);
        geom.nose_length_m = std::min(0.18 * full_length, geom.total_length_m);  // fairing/nose region
        geom.base_diameter_m = base_d;
        geom.power_on = true;
        post2::aero::finalize_geometry(&geom);
        const post2::aero::AeroTable table =
            post2::aero::generate_aero_table(geom, {}, {}, fins);
        const std::string path = output_dir + file;
        if (!post2::aero::write_aero_table_csv(path, table, &local_error)) {
            return false;
        }
        out->activate_at_min_attached_stage = lo;
        out->max_attached_stage = hi;
        out->table_path = path;
        out->reference_area_m2 = geom.ref_area_m2;
        out->ref_diameter_m = geom.ref_diameter_m;
        out->body_length_m = geom.total_length_m;
        out->nose_length_m = geom.nose_length_m;
        out->base_diameter_m = geom.base_diameter_m;
        out->nose_radius_m = 0.5 * geom.base_diameter_m;
        return true;
    };

    const auto& stages = config->vehicle.stages;
    const int stage_count = static_cast<int>(stages.size());

    // Per-stage wet mass, used to distribute the body length across stages when
    // the import only measured the full and upper-stack lengths.
    const auto stage_wet_mass = [](const post2::vehicle::StageConfig& s) -> double {
        double m = s.dry_mass_kg;
        for (const auto& tank : s.tanks) {
            m += tank.capacity_kg > 0.0 ? tank.capacity_kg : tank.initial_kg;
        }
        return std::max(m, 0.0);
    };

    std::vector<post2::vehicle::AeroStageTable> tables;

    // Full stack: every stage attached, [0, top]. Fairing diameter sets S_ref.
    post2::vehicle::AeroStageTable full;
    if (!make_table(0, -1, "aero_table_full.csv", full_length, ref_diameter, base_diameter,
                    &full)) {
        if (error) {
            *error = local_error;
        }
        return false;
    }
    tables.push_back(full);

    post2::vehicle::AeroStageTable first_stage;
    bool has_first_stage_table = false;

    // Multi-stage vehicle: build the whole staging ladder.
    //   * upper-stack tables [lo, top] for lo = 1..top  (after stage lo-1 drops),
    //   * single-stage tables [i, i] for i = 0..top-1   (a lower stage flying
    //     alone, e.g. booster recovery).
    // The terminal upper-stack table [top, top] is the last stage on its own, so
    // single-stage tables only cover the lower stages (0..top-1).
    if (stage_count >= 2) {
        const int top = stage_count - 1;

        // Measured upper-stack (stages 1..top) length when the import provides it,
        // else a slenderness estimate (~40% of the full stack).
        double upper_length = std::min(0.4 * full_length, full_length - 1.0);
        if (imported.has_upper_length && imported.upper_length_m > 0.0 &&
            imported.upper_length_m < full_length - 1.0) {
            upper_length = imported.upper_length_m;
        }

        // Estimate each stage's own body length. The booster (stage 0) takes the
        // measured remainder; the upper stack length is split across stages
        // 1..top by wet-mass fraction. This keeps the [0,top] and [1,top] tables
        // consistent with the measured full / upper lengths.
        std::vector<double> stage_length(static_cast<std::size_t>(stage_count), 0.0);
        stage_length[0] = std::max(full_length - upper_length, 1.0);
        double upper_mass = 0.0;
        for (int k = 1; k < stage_count; ++k) {
            upper_mass += stage_wet_mass(stages[static_cast<std::size_t>(k)]);
        }
        for (int k = 1; k < stage_count; ++k) {
            const double frac = upper_mass > 0.0
                ? stage_wet_mass(stages[static_cast<std::size_t>(k)]) / upper_mass
                : 1.0 / static_cast<double>(stage_count - 1);
            stage_length[static_cast<std::size_t>(k)] = std::max(upper_length * frac, 1.0);
        }

        // Upper-stack tables [lo, top]: the configurations the ascending vehicle
        // flies through. File name keyed by the leading (lowest attached) stage,
        // 1-based, so [1,top] -> aero_table_stage2.csv, matching prior output.
        for (int lo = 1; lo < stage_count; ++lo) {
            double sub_length = 0.0;
            for (int k = lo; k < stage_count; ++k) {
                sub_length += stage_length[static_cast<std::size_t>(k)];
            }
            const std::string file = "aero_table_stage" + std::to_string(lo + 1) + ".csv";
            post2::vehicle::AeroStageTable upper;
            if (!make_table(lo, -1, file, sub_length, ref_diameter, base_diameter, &upper)) {
                if (error) {
                    *error = local_error;
                }
                return false;
            }
            tables.push_back(upper);
        }

        // Single-stage tables [i, i] for the lower stages flying on their own.
        // No fairing, so the core diameter sets both the reference and base. The
        // booster (stage 0) carries the deployed grid fins through its recovery,
        // so its solo table is generated grid-fin-inclusive.
        for (int i = 0; i < top; ++i) {
            const std::string file =
                "aero_table_stage" + std::to_string(i + 1) + "_only.csv";
            post2::aero::GridFinSpec fins;
            if (i == 0) {
                fins.count = aero.grid_fins.count;
                fins.area_per_fin_m2 = aero.grid_fins.area_per_fin_m2;
            }
            post2::vehicle::AeroStageTable alone;
            if (!make_table(i, i, file, stage_length[static_cast<std::size_t>(i)],
                            base_diameter, base_diameter, &alone, fins)) {
                if (error) {
                    *error = local_error;
                }
                return false;
            }
            tables.push_back(alone);
            if (i == 0) {
                first_stage = alone;  // mirror into the legacy first_stage_table
                has_first_stage_table = true;
            }
        }
    }

    aero.enabled = true;
    aero.use_table = true;
    aero.stage_tables = tables;
    aero.first_stage_table =
        has_first_stage_table ? first_stage : post2::vehicle::AeroStageTable{};
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
