// Tests for the trigger-driven phase termination and case-level
// non-sequential events introduced alongside the phase action editor refactor.
// Covers:
//   * TriggerCondition / EventConfig JSON round-trip
//   * legacy duration_s ↔ termination synthesis on load
//   * altitude_m phase termination ends the phase near the threshold
//   * a mission event with a time trigger fires and applies its actions

#include "post2/core/case_config_io.hpp"
#include "post2/core/trajectory_service.hpp"
#include "post2/vehicle/vehicle.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

post2::vehicle::VehicleConfig make_simple_vehicle()
{
    post2::vehicle::VehicleConfig vehicle;
    vehicle.name = "test vehicle";
    vehicle.dry_mass_kg = 1000.0;

    post2::vehicle::StageConfig stage;
    stage.name = "stage1";
    stage.active = true;
    stage.attached = true;
    stage.dry_mass_kg = 800.0;
    stage.engine.enabled = true;
    // T/W well above 1 so the rocket lifts off from the launch site under
    // normal force. Total dry+payload+propellant ≈ 6000 kg → T/W ≈ 2.5.
    stage.engine.thrust_vac_n = 150000.0;
    stage.engine.isp_vac_s = 280.0;
    stage.engine.engine_count = 1;
    stage.engine.direction_body = {1.0, 0.0, 0.0};
    post2::vehicle::TankConfig tank;
    tank.name = "main";
    tank.propellant = "rp1";
    tank.capacity_kg = 5000.0;
    tank.initial_kg = 5000.0;
    stage.tanks = {tank};
    stage.engine.feed_tanks = {{"stage1", "main"}};

    post2::vehicle::StageConfig payload;
    payload.name = "payload";
    payload.active = false;
    payload.attached = true;
    payload.dry_mass_kg = 200.0;
    payload.engine.enabled = false;
    payload.tanks = {};

    vehicle.stages = {stage, payload};
    vehicle.dry_mass_kg = stage.dry_mass_kg + payload.dry_mass_kg;
    vehicle.tanks = stage.tanks;
    vehicle.engine = stage.engine;
    return vehicle;
}

post2::core::CaseConfig make_base_case()
{
    post2::core::CaseConfig config;
    config.name = "test";
    config.step_s = 1.0;
    config.launch_site.latitude_deg = 28.5;
    config.launch_site.longitude_deg = -80.6;
    config.launch_site.altitude_m = 100.0;
    config.vehicle = make_simple_vehicle();
    return config;
}

post2::core::PhaseConfig make_default_phase(const post2::core::CaseConfig& config)
{
    post2::core::PhaseConfig phase;
    phase.name = "phase";
    phase.termination = {"time", ">=", 60.0};
    phase.inherit_initial_state = false;
    phase.hold_down_clamp_initial_active = false;
    phase.optimize_enabled = false;
    phase.integrator = "dopri5";
    phase.force_models.gravity = true;
    phase.force_models.gravity_model.j2 = 0.0;
    phase.force_models.gravity_model.type = "j2";
    phase.force_models.thrust = true;
    phase.force_models.normal_force = false;
    phase.force_models.aerodynamic = false;
    phase.force_models.atmosphere_model.type = "none";
    phase.throttle_model.type = "poly";
    phase.throttle_model.c0 = 1.0;
    phase.steering_model.type = "generic_poly";
    // Straight up.
    phase.steering_model.azimuth_deg.c0 = 90.0;
    phase.steering_model.elevation_deg.c0 = 90.0;
    return phase;
    (void)config;
}

int test_trigger_and_event_json_roundtrip()
{
    post2::core::CaseConfig config = make_base_case();
    auto phase = make_default_phase(config);
    phase.termination = {"altitude_m", ">=", 12345.6};
    config.phases.push_back(phase);

    post2::core::EventConfig event;
    event.name = "test_event";
    event.enabled = true;
    event.trigger = {"velocity_mps", "<=", 250.0};
    post2::core::PhaseAction action;
    action.type = "set_stage_active";
    action.value = false;
    action.stage_index = 0;
    action.stage_name = "stage1";
    event.actions.push_back(action);
    config.events.push_back(event);

    const std::string json = post2::core::case_config_to_json(config);
    post2::core::CaseConfig reloaded;
    std::string error;
    if (!post2::core::case_config_from_json(json, &reloaded, &error)) {
        std::cerr << "trigger json reload failed: " << error << '\n';
        return 1;
    }
    if (reloaded.phases.size() != 1 ||
        reloaded.phases[0].termination.type != "altitude_m" ||
        reloaded.phases[0].termination.comparison != ">=" ||
        std::abs(reloaded.phases[0].termination.value - 12345.6) > 1.0e-9) {
        std::cerr << "phase termination did not survive roundtrip\n";
        return 1;
    }
    if (reloaded.events.size() != 1 ||
        reloaded.events[0].name != "test_event" ||
        !reloaded.events[0].enabled ||
        reloaded.events[0].trigger.type != "velocity_mps" ||
        reloaded.events[0].trigger.comparison != "<=" ||
        std::abs(reloaded.events[0].trigger.value - 250.0) > 1.0e-9 ||
        reloaded.events[0].actions.size() != 1 ||
        reloaded.events[0].actions[0].type != "set_stage_active" ||
        reloaded.events[0].actions[0].stage_name != "stage1") {
        std::cerr << "mission event did not survive roundtrip\n";
        return 1;
    }
    return 0;
}

int test_legacy_duration_s_synthesises_termination()
{
    // Hand-write a phase block with only duration_s (no termination block).
    const std::string legacy_json = R"({
        "name": "legacy",
        "step_s": 1.0,
        "launch_site": {"latitude_deg": 28.5, "longitude_deg": -80.6, "altitude_m": 100.0},
        "phases": [
            {"name": "p", "duration_s": 7.5,
             "force_models": {"thrust": false, "normal_force": false, "aerodynamic": false},
             "throttle_model": {"type": "poly", "c0": 1.0},
             "steering_model": {"type": "generic_poly"}}
        ]
    })";
    post2::core::CaseConfig parsed;
    std::string error;
    if (!post2::core::case_config_from_json(legacy_json, &parsed, &error)) {
        std::cerr << "legacy duration_s case failed to parse: " << error << '\n';
        return 1;
    }
    if (parsed.phases.size() != 1 ||
        parsed.phases[0].termination.type != "time" ||
        parsed.phases[0].termination.comparison != ">=" ||
        std::abs(parsed.phases[0].termination.value - 7.5) > 1.0e-9 ||
        std::abs(parsed.phases[0].termination.value - 7.5) > 1.0e-9) {
        std::cerr << "legacy duration_s did not synthesise termination\n";
        return 1;
    }
    return 0;
}

int test_altitude_termination()
{
    post2::core::CaseConfig config = make_base_case();
    auto phase = make_default_phase(config);
    // Start at launch site under hold-down, release at t=0 so the rocket
    // begins climbing immediately.
    phase.hold_down_clamp_initial_active = true;
    phase.force_models.normal_force = true;
    post2::core::PhaseAction release;
    release.time_s = 0.0;
    release.type = "set_hold_down_clamp_active";
    release.value = false;
    phase.actions.push_back(release);
    // Phase should terminate when the vehicle climbs through 500 m above
    // the launch site (launch altitude is 100 m → absolute 600 m).
    phase.termination = {"altitude_m", ">=", 600.0};
    // termination.value 600.0 is set later by `phase.termination = {...}`; no
    // separate duration safety cap is needed (kMaxPhaseTimeS handles it).
    config.phases.push_back(phase);

    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(config);
    if (!result.ok) {
        std::cerr << "altitude termination simulation failed: " << result.error << '\n';
        return 1;
    }
    if (result.state_log.empty()) {
        std::cerr << "altitude termination produced empty state log\n";
        return 1;
    }
    const double final_altitude = result.state_log.back().altitude_m;
    if (final_altitude < 595.0 || final_altitude > 700.0) {
        std::cerr << "altitude termination ended at unexpected altitude: "
                  << final_altitude << '\n';
        return 1;
    }
    if (result.state_log.back().time_s > 200.0) {
        std::cerr << "altitude termination did not end before the safety cap: t="
                  << result.state_log.back().time_s << '\n';
        return 1;
    }
    return 0;
}

int test_initially_satisfied_phase_termination()
{
    post2::core::CaseConfig config = make_base_case();
    auto phase = make_default_phase(config);
    phase.inherit_initial_state = false;
    phase.force_models.thrust = false;
    phase.force_models.normal_force = false;
    phase.force_models.aerodynamic = false;

    const double orbit_altitude_m = 200000.0;
    const double radius_m = config.earth_radius_m + orbit_altitude_m;
    const double circular_speed_mps = std::sqrt(config.earth_mu_m3s2 / radius_m);
    phase.initial_state_eci = post2::core::State{
        {radius_m, 0.0, 0.0},
        {0.0, circular_speed_mps, 0.0},
    };
    phase.termination = {"periapsis_altitude_m", ">=", 170000.0};
    config.phases.push_back(phase);

    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(config);
    if (!result.ok) {
        std::cerr << "initially satisfied termination simulation failed: "
                  << result.error << '\n';
        return 1;
    }
    if (result.state_log.size() != 1) {
        std::cerr << "initially satisfied termination should stop at the initial entry, got "
                  << result.state_log.size() << " entries\n";
        return 1;
    }
    if (std::abs(result.state_log.back().time_s) > 1.0e-9) {
        std::cerr << "initially satisfied termination advanced to t="
                  << result.state_log.back().time_s << '\n';
        return 1;
    }
    return 0;
}

int test_mission_event_fires_and_deactivates_stage()
{
    post2::core::CaseConfig config = make_base_case();
    auto phase = make_default_phase(config);
    phase.hold_down_clamp_initial_active = true;
    phase.force_models.normal_force = true;
    post2::core::PhaseAction release;
    release.time_s = 0.0;
    release.type = "set_hold_down_clamp_active";
    release.value = false;
    phase.actions.push_back(release);
    phase.termination = {"time", ">=", 30.0};
    config.phases.push_back(phase);

    post2::core::EventConfig event;
    event.name = "deactivate stage1 at t=10";
    event.enabled = true;
    event.trigger = {"time", ">=", 10.0};
    post2::core::PhaseAction deactivate;
    deactivate.type = "set_stage_active";
    deactivate.value = false;
    deactivate.stage_index = 0;
    deactivate.stage_name = "stage1";
    event.actions.push_back(deactivate);
    config.events.push_back(event);

    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(config);
    if (!result.ok) {
        std::cerr << "mission event simulation failed: " << result.error << '\n';
        return 1;
    }

    // Find a state entry strictly after t=10 and assert stage 1 is inactive
    // (no engine thrust) by checking propellant mass has stopped decreasing.
    double mass_at_10 = -1.0;
    double mass_at_end = -1.0;
    for (const auto& entry : result.state_log.entries()) {
        if (entry.time_s < 9.9 || entry.time_s > 10.1) {
            // skip
        } else if (mass_at_10 < 0.0) {
            mass_at_10 = entry.runtime.vehicle.propellant_mass_kg;
        }
        mass_at_end = entry.runtime.vehicle.propellant_mass_kg;
    }
    if (mass_at_10 < 0.0 || mass_at_end < 0.0) {
        std::cerr << "did not find expected log entries for mission event test\n";
        return 1;
    }
    // After deactivation, propellant should not decrease further (or decrease
    // by less than the per-step burn would have produced).
    if (mass_at_end + 1.0 < mass_at_10) {
        std::cerr << "stage1 kept burning after event fired: mass_at_10=" << mass_at_10
                  << " mass_at_end=" << mass_at_end << '\n';
        return 1;
    }
    return 0;
}

int test_continuity_flags_json_roundtrip()
{
    post2::core::CaseConfig config = make_base_case();
    auto phase = make_default_phase(config);
    phase.throttle_model.continuity = true;
    phase.steering_model.azimuth_deg.continuity = true;
    phase.steering_model.elevation_deg.continuity = false;
    phase.steering_model.pitch_deg.continuity = true;
    config.phases.push_back(phase);

    const std::string json = post2::core::case_config_to_json(config);
    post2::core::CaseConfig reloaded;
    std::string error;
    if (!post2::core::case_config_from_json(json, &reloaded, &error)) {
        std::cerr << "continuity json reload failed: " << error << '\n';
        return 1;
    }
    if (reloaded.phases.size() != 1) {
        std::cerr << "continuity roundtrip: wrong phase count\n";
        return 1;
    }
    const auto& p = reloaded.phases[0];
    if (!p.throttle_model.continuity ||
        !p.steering_model.azimuth_deg.continuity ||
        p.steering_model.elevation_deg.continuity ||
        !p.steering_model.pitch_deg.continuity) {
        std::cerr << "continuity flags did not survive roundtrip\n";
        return 1;
    }
    return 0;
}

// Finds the last phase-0 entry (the phase boundary) and the first phase-1 entry
// (which reflects the phase-2 throttle/steering command — the inherited initial
// entry is deduped because it shares the boundary's timestamp).
bool find_boundary_and_next(
    const post2::core::StateLog& log,
    post2::core::LaunchVehicleStateLogEntry* boundary,
    post2::core::LaunchVehicleStateLogEntry* next)
{
    bool have_boundary = false;
    for (const auto& entry : log.entries()) {
        if (entry.phase_index == 0) {
            *boundary = entry;
            have_boundary = true;
        }
    }
    if (!have_boundary) {
        return false;
    }
    for (const auto& entry : log.entries()) {
        if (entry.phase_index == 1) {
            *next = entry;
            return true;
        }
    }
    return false;
}

post2::core::CaseConfig make_two_phase_continuity_case(bool continuity, const std::string& p2_throttle_type)
{
    post2::core::CaseConfig config = make_base_case();

    // Phase 1: a free-flight burn with a decreasing poly throttle and a fixed
    // azimuth/elevation, so the boundary throttle and direction are non-trivial.
    auto p1 = make_default_phase(config);
    p1.name = "p1";
    p1.inherit_initial_state = false;  // phase 0 uses the default LEO state
    p1.termination = {"time", ">=", 10.0};
    post2::core::PhaseAction enable_engine;
    enable_engine.time_s = 0.0;
    enable_engine.type = "set_engine_enabled";
    enable_engine.value = true;
    p1.actions.push_back(enable_engine);
    p1.throttle_model.type = "poly";
    p1.throttle_model.c0 = 1.0;
    p1.throttle_model.c1 = -0.02;
    p1.steering_model.type = "generic_poly";
    p1.steering_model.azimuth_deg.c0 = 90.0;
    p1.steering_model.elevation_deg.c0 = 80.0;
    config.phases.push_back(p1);

    // Phase 2: deliberately different configured constants. With continuity on
    // they must be re-anchored to the phase-1 boundary; with it off they stand.
    auto p2 = make_default_phase(config);
    p2.name = "p2";
    p2.inherit_initial_state = true;
    p2.termination = {"time", ">=", 5.0};
    p2.throttle_model.type = p2_throttle_type;
    p2.throttle_model.c0 = 0.3;
    p2.throttle_model.target_t2w = 0.1;
    p2.throttle_model.continuity = continuity;
    p2.steering_model.type = "generic_poly";
    p2.steering_model.azimuth_deg.c0 = 0.0;
    p2.steering_model.elevation_deg.c0 = 0.0;
    p2.steering_model.azimuth_deg.continuity = continuity;
    p2.steering_model.elevation_deg.continuity = continuity;
    config.phases.push_back(p2);
    return config;
}

int test_phase_continuity_reanchors_throttle_and_steering()
{
    post2::core::LocalTrajectoryService service;

    // Continuity ON: phase 2 opens at the phase-1 boundary throttle/direction,
    // for both the poly and T/W throttle models.
    for (const std::string& type : {std::string("poly"), std::string("t2w")}) {
        const auto config = make_two_phase_continuity_case(true, type);
        const auto result = service.simulate(config);
        if (!result.ok) {
            std::cerr << "continuity(" << type << ") sim failed: " << result.error << '\n';
            return 1;
        }
        post2::core::LaunchVehicleStateLogEntry boundary;
        post2::core::LaunchVehicleStateLogEntry next;
        if (!find_boundary_and_next(result.state_log, &boundary, &next)) {
            std::cerr << "continuity(" << type << "): missing boundary/next entries\n";
            return 1;
        }
        if (std::abs(next.throttle - boundary.throttle) > 2.0e-3) {
            std::cerr << "continuity(" << type << "): throttle not held: boundary="
                      << boundary.throttle << " next=" << next.throttle << '\n';
            return 1;
        }
        const double dir_dot =
            boundary.engine_direction_eci.x * next.engine_direction_eci.x +
            boundary.engine_direction_eci.y * next.engine_direction_eci.y +
            boundary.engine_direction_eci.z * next.engine_direction_eci.z;
        if (dir_dot < 0.9999) {
            std::cerr << "continuity(" << type << "): direction not held: dot=" << dir_dot << '\n';
            return 1;
        }
    }

    // Continuity OFF: phase 2 opens at its own configured throttle (0.3), which
    // differs from the boundary value.
    {
        const auto config = make_two_phase_continuity_case(false, "poly");
        const auto result = service.simulate(config);
        if (!result.ok) {
            std::cerr << "no-continuity sim failed: " << result.error << '\n';
            return 1;
        }
        post2::core::LaunchVehicleStateLogEntry boundary;
        post2::core::LaunchVehicleStateLogEntry next;
        if (!find_boundary_and_next(result.state_log, &boundary, &next)) {
            std::cerr << "no-continuity: missing boundary/next entries\n";
            return 1;
        }
        if (std::abs(next.throttle - boundary.throttle) < 0.05) {
            std::cerr << "no-continuity: throttle unexpectedly matched boundary (boundary="
                      << boundary.throttle << " next=" << next.throttle << ")\n";
            return 1;
        }
    }

    return 0;
}

} // namespace

// The engine spool transient (ignition dead-time + first-order spool) and the
// "thrust_fraction" termination must survive a JSON round-trip.
int test_spool_and_thrust_fraction_json_roundtrip()
{
    post2::core::CaseConfig config = make_base_case();
    config.vehicle.stages[0].engine.ignition_delay_s = 1.0;
    config.vehicle.stages[0].engine.spool_up_rate_per_s = 5.0;
    config.vehicle.stages[0].engine.spool_down_rate_per_s = 4.0;
    config.vehicle.engine = config.vehicle.stages[0].engine;
    auto phase = make_default_phase(config);
    phase.termination = {"thrust_fraction", ">=", 0.95};
    config.phases.push_back(phase);

    const std::string json = post2::core::case_config_to_json(config);
    post2::core::CaseConfig reloaded;
    std::string error;
    if (!post2::core::case_config_from_json(json, &reloaded, &error)) {
        std::cerr << "spool/thrust_fraction reload failed: " << error << '\n';
        return 1;
    }
    if (reloaded.phases.size() != 1 ||
        reloaded.phases[0].termination.type != "thrust_fraction" ||
        std::abs(reloaded.phases[0].termination.value - 0.95) > 1.0e-9) {
        std::cerr << "thrust_fraction termination did not survive roundtrip\n";
        return 1;
    }
    const auto& e = reloaded.vehicle.stages[0].engine;
    if (std::abs(e.ignition_delay_s - 1.0) > 1.0e-9 ||
        std::abs(e.spool_up_rate_per_s - 5.0) > 1.0e-9 ||
        std::abs(e.spool_down_rate_per_s - 4.0) > 1.0e-9) {
        std::cerr << "engine spool fields did not survive roundtrip\n";
        return 1;
    }
    return 0;
}

// A clamped ignition phase: thrust must be zero through the ignition dead-time,
// spool up, and the "thrust_fraction" termination must release the phase once
// actual thrust reaches 95% of the commanded steady thrust. With
// ignition_delay = 1.0 s and spool_up_rate = 5.0 /s, the 0.95 crossing is at
// t = 1.0 + ln(1/0.05)/5 ≈ 1.6 s.
int test_thrust_fraction_termination_ends_on_spool()
{
    post2::core::CaseConfig config = make_base_case();
    config.step_s = 0.1;  // resolve the ignition dead-time with sub-second samples
    config.vehicle.stages[0].engine.ignition_delay_s = 1.0;
    config.vehicle.stages[0].engine.spool_up_rate_per_s = 5.0;
    config.vehicle.stages[0].engine.spool_down_rate_per_s = 5.0;
    config.vehicle.engine = config.vehicle.stages[0].engine;

    auto phase = make_default_phase(config);
    phase.hold_down_clamp_initial_active = true;
    phase.force_models.normal_force = true;
    phase.force_models.thrust = true;
    phase.throttle_model.type = "poly";
    phase.throttle_model.c0 = 1.0;
    post2::core::PhaseAction ignite;
    ignite.time_s = 0.0;
    ignite.type = "set_engine_enabled";
    ignite.value = true;
    phase.actions.push_back(ignite);
    phase.termination = {"thrust_fraction", ">=", 0.95};
    config.phases.push_back(phase);

    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(config);
    if (!result.ok) {
        std::cerr << "thrust_fraction simulation failed: " << result.error << '\n';
        return 1;
    }
    if (result.state_log.empty()) {
        std::cerr << "thrust_fraction produced empty state log\n";
        return 1;
    }

    // During the ignition dead-time (t < 1.0 s) there is no thrust and the
    // vehicle stays clamped at the pad.
    bool saw_deadtime = false;
    for (const auto& entry : result.state_log.entries()) {
        if (entry.time_s > 0.2 && entry.time_s < 0.9) {
            saw_deadtime = true;
            if (entry.runtime.engine.actual_thrust_n > 1.0) {
                std::cerr << "thrust during ignition dead-time: "
                          << entry.runtime.engine.actual_thrust_n << " at t=" << entry.time_s << '\n';
                return 1;
            }
            if (!entry.hold_down_clamp_active) {
                std::cerr << "clamp released during dead-time at t=" << entry.time_s << '\n';
                return 1;
            }
        }
    }
    if (!saw_deadtime) {
        std::cerr << "no log entries inside the ignition dead-time window\n";
        return 1;
    }

    // The phase ends when thrust is established, near t ≈ 1.6 s and well before
    // any runaway, with actual thrust at ~95% of the commanded steady thrust.
    const auto& last = result.state_log.back();
    if (last.time_s < 1.4 || last.time_s > 2.0) {
        std::cerr << "thrust established at unexpected time t=" << last.time_s << '\n';
        return 1;
    }
    const double commanded = last.runtime.engine.commanded_thrust_n;
    const double actual = last.runtime.engine.actual_thrust_n;
    if (commanded <= 0.0 || actual / commanded < 0.95 || actual / commanded > 0.99) {
        std::cerr << "thrust fraction at termination out of range: "
                  << (commanded > 0.0 ? actual / commanded : 0.0) << '\n';
        return 1;
    }
    return 0;
}

int main()
{
    int failures = 0;
    failures += test_trigger_and_event_json_roundtrip();
    failures += test_spool_and_thrust_fraction_json_roundtrip();
    failures += test_thrust_fraction_termination_ends_on_spool();
    failures += test_legacy_duration_s_synthesises_termination();
    failures += test_altitude_termination();
    failures += test_initially_satisfied_phase_termination();
    failures += test_mission_event_fires_and_deactivates_stage();
    failures += test_continuity_flags_json_roundtrip();
    failures += test_phase_continuity_reanchors_throttle_and_steering();
    if (failures != 0) {
        std::cerr << "phase_termination_and_events_test: " << failures << " case(s) failed\n";
        return 1;
    }
    return 0;
}
