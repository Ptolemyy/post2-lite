// Roundtrip: post2::core::guidance_script_to_csv -> parse_guidance_script.
// Verifies the writer (post2_core) and the player's parser agree, covering a
// poly first-stage phase, a staging action, and a UPFG upper phase.

#include "post2/core/io.hpp"
#include "post2/core/types.hpp"
#include "post2/player/guidance_script.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool close(double a, double b, double eps = 1.0e-9)
{
    return std::abs(a - b) <= eps;
}

} // namespace

int main()
{
    post2::core::CaseConfig cc;
    cc.name = "rt_case";
    cc.earth_mu_m3s2 = 3.5316e12;     // Kerbin-like
    cc.earth_radius_m = 600000.0;
    cc.launch_site.latitude_deg = 0.0;
    cc.launch_site.longitude_deg = -74.0;

    // Two stages.
    post2::vehicle::StageConfig booster;
    booster.name = "booster";
    booster.dry_mass_kg = 5000.0;
    booster.engine.thrust_vac_n = 1.0e6;
    booster.engine.isp_vac_s = 300.0;
    booster.engine.engine_count = 1;
    booster.tanks = {post2::vehicle::TankConfig{}};
    booster.tanks[0].initial_kg = 40000.0;

    post2::vehicle::StageConfig upper;
    upper.name = "upper";
    upper.dry_mass_kg = 2000.0;
    upper.engine.thrust_vac_n = 2.0e5;
    upper.engine.isp_vac_s = 340.0;
    upper.engine.engine_count = 1;
    upper.tanks = {post2::vehicle::TankConfig{}};
    upper.tanks[0].initial_kg = 18000.0;

    cc.vehicle.stages = {booster, upper};

    // Phase 0: poly ascent + poly throttle, ends on time, separates the booster.
    post2::core::PhaseConfig p0;
    p0.name = "ascent";
    p0.hold_down_clamp_initial_active = true;  // pad-hold ignition flag must round-trip
    p0.steering_model.type = "generic_poly";
    p0.steering_model.azimuth_deg.c0 = 90.0;
    p0.steering_model.elevation_deg.c0 = 85.0;
    p0.steering_model.elevation_deg.c1 = -0.5;
    p0.throttle_model.type = "poly";
    p0.throttle_model.c0 = 1.0;
    p0.termination = {"time", ">=", 120.0};
    post2::core::PhaseAction sep;
    sep.type = "set_stage_attached";
    sep.value = false;
    sep.stage_index = 0;
    sep.stage_name = "booster";
    p0.actions = {sep};

    // Phase 1: UPFG upper stage to a circular target.
    post2::core::PhaseConfig p1;
    p1.name = "upfg";
    p1.steering_model.type = "upfg";
    p1.steering_model.upfg.periapsis_km = 100.0;
    p1.steering_model.upfg.apoapsis_km = 100.0;
    p1.steering_model.upfg.inclination_deg = 0.0;
    p1.termination = {"altitude_m", ">=", 90000.0};

    cc.phases = {p0, p1};

    const std::string csv = post2::core::guidance_script_to_csv(cc);

    post2::player::GuidanceScript script;
    std::string error;
    if (!post2::player::parse_guidance_script(csv, &script, &error)) {
        std::cerr << "parse failed: " << error << "\n";
        return 1;
    }

    if (!close(script.meta.mu_m3s2, cc.earth_mu_m3s2, 1.0) ||
        !close(script.meta.radius_m, cc.earth_radius_m, 1.0e-3)) {
        std::cerr << "META mismatch\n";
        return 1;
    }

    if (script.stages.size() != 2) {
        std::cerr << "expected 2 stages, got " << script.stages.size() << "\n";
        return 1;
    }
    // STAGE 0: thrust = count*thrust_vac, ve = isp*g0, mdot = thrust/ve.
    const double g0 = 9.80665;
    if (!close(script.stages[0].thrust_n, 1.0e6, 1.0) ||
        !close(script.stages[0].exhaust_velocity_mps, 300.0 * g0, 1.0e-3) ||
        !close(script.stages[0].mdot_kgps, 1.0e6 / (300.0 * g0), 1.0e-6) ||
        !close(script.stages[0].propellant_kg, 40000.0, 1.0e-6) ||
        !close(script.stages[0].dry_mass_kg, 5000.0, 1.0e-6)) {
        std::cerr << "STAGE 0 propulsion mismatch\n";
        return 1;
    }

    if (script.phases.size() != 2) {
        std::cerr << "expected 2 phases\n";
        return 1;
    }
    const auto& sp0 = script.phases[0];
    if (sp0.steering_type != "generic_poly" ||
        !close(sp0.azimuth[0], 90.0) ||
        !close(sp0.elevation[0], 85.0) ||
        !close(sp0.elevation[1], -0.5) ||
        sp0.throttle_type != "poly" ||
        !close(sp0.throttle_c0, 1.0) ||
        sp0.termination_type != "time" ||
        !close(sp0.termination_value, 120.0) ||
        !sp0.hold_down_clamp_initial_active ||
        sp0.actions.size() != 1 ||
        sp0.actions[0].type != "set_stage_attached" ||
        sp0.actions[0].value != false ||
        sp0.actions[0].stage_index != 0) {
        std::cerr << "phase 0 mismatch\n";
        return 1;
    }

    const auto& sp1 = script.phases[1];
    if (sp1.hold_down_clamp_initial_active ||
        !sp1.is_upfg ||
        sp1.steering_type != "upfg" ||
        !close(sp1.upfg_periapsis_km, 100.0) ||
        !close(sp1.upfg_apoapsis_km, 100.0) ||
        !close(sp1.upfg_inclination_deg, 0.0) ||
        sp1.termination_type != "altitude_m" ||
        !close(sp1.termination_value, 90000.0)) {
        std::cerr << "phase 1 (UPFG) mismatch\n";
        return 1;
    }

    std::cout << "guidance_script roundtrip OK\n";
    return 0;
}
