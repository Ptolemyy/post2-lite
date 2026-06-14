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

} // namespace

int main()
{
    int failures = 0;
    failures += test_trigger_and_event_json_roundtrip();
    failures += test_legacy_duration_s_synthesises_termination();
    failures += test_altitude_termination();
    failures += test_mission_event_fires_and_deactivates_stage();
    if (failures != 0) {
        std::cerr << "phase_termination_and_events_test: " << failures << " case(s) failed\n";
        return 1;
    }
    return 0;
}
