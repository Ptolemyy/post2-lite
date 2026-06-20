#pragma once

#include <string>
#include <vector>

namespace post2::player {

// In-memory form of the sectioned guidance-script CSV written by
// post2::core::guidance_script_to_csv. The C++ kRPC player (post2_player)
// re-flies the ascent from these per-phase parameters: it evaluates the
// polynomials itself and runs the real UPFG (post2/core/upfg.hpp) for UPFG
// phases. See include/post2/core/io.hpp for the writer.

struct ScriptMeta {
    std::string name;
    double mu_m3s2 = 0.0;
    double radius_m = 0.0;
    double rotation_rad_s = 0.0;
    double rotation_at_epoch_rad = 0.0;
    double launch_lat_deg = 0.0;
    double launch_lon_deg = 0.0;
    double launch_alt_m = 0.0;
};

// Propulsion of one configured stage, in burn order. Live total mass comes from
// KSP; the player rebuilds UpfgStages from these (mirrors build_upfg_stages).
struct ScriptStage {
    int index = 0;
    std::string name;
    double thrust_n = 0.0;
    double exhaust_velocity_mps = 0.0;
    double mdot_kgps = 0.0;
    double propellant_kg = 0.0;
    double dry_mass_kg = 0.0;
};

struct ScriptAction {
    int phase_index = 0;
    double time_s = 0.0;
    std::string type;
    bool value = false;
    int stage_index = -1;
    std::string stage_name;
};

struct ScriptSegment {
    double start_time_s = 0.0;
    int order = 1;
    std::vector<double> azimuth_coefficients;
    std::vector<double> elevation_coefficients;
};

struct ScriptPhase {
    int index = 0;
    std::string name;
    std::string steering_type = "generic_poly";

    // Vehicle is held on the pad by the hold-down clamp while this phase runs
    // (the ignition phase). The player ignites and waits for thrust to build,
    // then releases the launch clamp when the phase ends. Absent in old scripts
    // (defaults false).
    bool hold_down_clamp_initial_active = false;

    std::string termination_type = "time";
    std::string termination_comparison = ">=";
    double termination_value = 0.0;

    std::string throttle_type = "poly";
    double throttle_c0 = 1.0;
    double throttle_c1 = 0.0;
    double throttle_c2 = 0.0;

    // generic_poly / rpy_poly / tangent azimuth: az[0..2], el[0..2], roll[0..2].
    double azimuth[3] = {0.0, 0.0, 0.0};
    double elevation[3] = {0.0, 0.0, 0.0};
    double roll[3] = {0.0, 0.0, 0.0};

    // segmented_poly.
    std::vector<ScriptSegment> segments;

    // linear/bilinear tangent (elevation law); azimuth still from azimuth[].
    bool has_tangent = false;
    bool tangent_bilinear = false;
    double tangent_a = 0.0;
    double tangent_a_dot = 0.0;
    double tangent_b = 0.0;
    double tangent_b_dot = 0.0;
    double tangent_t_offset_s = 0.0;

    // upfg: the only steering data a UPFG phase needs.
    bool is_upfg = false;
    double upfg_periapsis_km = 0.0;
    double upfg_apoapsis_km = 0.0;
    double upfg_inclination_deg = 0.0;

    std::vector<ScriptAction> actions;
};

struct GuidanceScript {
    ScriptMeta meta;
    std::vector<ScriptStage> stages;
    std::vector<ScriptPhase> phases;
};

// Parse the sectioned CSV text. Returns false and sets *error on a malformed
// file (missing META, unparsable numbers, no phases).
bool parse_guidance_script(const std::string& text, GuidanceScript* out, std::string* error);

} // namespace post2::player
