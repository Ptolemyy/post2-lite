#include "post2/core/case_config_io.hpp"
#include "post2/core/control_models.hpp"
#include "post2/core/io.hpp"
#include "post2/core/optimization.hpp"
#include "post2/core/projection.hpp"
#include "post2/core/trajectory_service.hpp"
#include "post2/environment/atmosphere.hpp"
#include "post2/propagation/force_model.hpp"
#include "post2/propagation/force_models.hpp"
#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle_config_io.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

int main()
{
    post2::core::SimulationConfig config;
    config.duration_s = 5400.0;
    config.step_s = 10.0;

    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(config);
    if (!result.ok) {
        std::cerr << result.error << '\n';
        return 1;
    }

    if (result.state_log.size() < 500) {
        std::cerr << "too few StateLog entries\n";
        return 1;
    }

    double min_altitude = result.state_log.front().altitude_m;
    double max_altitude = result.state_log.front().altitude_m;
    for (const auto& point : result.state_log.entries()) {
        min_altitude = std::min(min_altitude, point.altitude_m);
        max_altitude = std::max(max_altitude, point.altitude_m);
    }

    if (min_altitude < 180000.0 || max_altitude > 210000.0) {
        std::cerr << "unexpected LEO altitude drift: min=" << min_altitude
                  << " max=" << max_altitude << '\n';
        return 1;
    }

    const auto projector = post2::core::make_orbit_plane_projector(result.state_log);
    double min_projected_radius = result.state_log.front().radius_m;
    for (const auto& point : result.state_log.entries()) {
        const auto projected = projector.project(point.state.position_m);
        const double projected_radius = std::sqrt(projected.x_m * projected.x_m + projected.y_m * projected.y_m);
        min_projected_radius = std::min(min_projected_radius, projected_radius);
    }

    if (min_projected_radius <= post2::core::kEarthRadiusM) {
        std::cerr << "projected orbit intersects Earth: min_projected_radius="
                  << min_projected_radius << '\n';
        return 1;
    }

    const auto csv = post2::core::trajectory_to_csv(result.state_log);
    const auto parsed = post2::core::trajectory_from_csv(csv);
    if (!parsed.ok || parsed.state_log.size() != result.state_log.size()) {
        std::cerr << "CSV roundtrip failed\n";
        return 1;
    }

    {
        post2::environment::ExponentialAtmosphereModel atmosphere(post2::core::kEarthRadiusM);
        const auto sea_level = atmosphere.sample(
            0.0,
            {post2::core::kEarthRadiusM, 0.0, 0.0},
            {});
        const auto scale_height = atmosphere.sample(
            0.0,
            {post2::core::kEarthRadiusM + 7200.0, 0.0, 0.0},
            {});
        if (std::abs(sea_level.density_kgpm3 - 1.225) > 1.0e-12 ||
            std::abs(scale_height.density_kgpm3 - 1.225 / std::exp(1.0)) > 1.0e-12 ||
            sea_level.pressure_pa <= 0.0 ||
            sea_level.temperature_k <= 0.0 ||
            sea_level.speed_of_sound_mps <= 0.0 ||
            !std::isfinite(sea_level.pressure_pa) ||
            !std::isfinite(sea_level.speed_of_sound_mps)) {
            std::cerr << "exponential atmosphere sample was not physically sane\n";
            return 1;
        }

        post2::core::CaseConfig gravity_case;
        gravity_case.earth_radius_m = post2::core::kEarthRadiusM;
        gravity_case.earth_mu_m3s2 = post2::core::kEarthMuM3S2;
        gravity_case.earth_j2 = post2::core::kEarthJ2;

        post2::core::PhaseConfig gravity_phase;
        gravity_phase.force_models.gravity_model.type = "point_mass";
        gravity_phase.force_models.gravity_model.j2 = post2::core::kEarthJ2;

        post2::integrators::ExtendedState gravity_state;
        gravity_state.motion.position_m = {post2::core::kEarthRadiusM + 400000.0, 0.0, 0.0};
        const post2::propagation::ForceModelContext gravity_context{
            &gravity_case,
            &gravity_phase,
            nullptr,
            nullptr,
            {},
        };

        const double radius_m = post2::vehicle::norm(gravity_state.motion.position_m);
        const double point_factor =
            -post2::core::kEarthMuM3S2 / (radius_m * radius_m * radius_m);
        const post2::core::Vec3 expected_point =
            gravity_state.motion.position_m * point_factor;

        post2::propagation::PointMassGravityModel point_model;
        const auto point_output = point_model.evaluate(gravity_context, gravity_state);
        if (std::abs(point_output.acceleration_eci_mps2.x - expected_point.x) > 1.0e-12 ||
            std::abs(point_output.acceleration_eci_mps2.y - expected_point.y) > 1.0e-12 ||
            std::abs(point_output.acceleration_eci_mps2.z - expected_point.z) > 1.0e-12) {
            std::cerr << "point-mass force model did not match two-body gravity\n";
            return 1;
        }

        gravity_phase.force_models.gravity_model.type = "j2";
        const post2::propagation::ForceModelContext j2_context{
            &gravity_case,
            &gravity_phase,
            nullptr,
            nullptr,
            {},
        };
        post2::propagation::J2GravityModel j2_model;
        const auto j2_output = j2_model.evaluate(j2_context, gravity_state);
        const post2::core::Vec3 j2_delta =
            j2_output.acceleration_eci_mps2 - point_output.acceleration_eci_mps2;
        if (!std::isfinite(j2_output.acceleration_eci_mps2.x) ||
            !std::isfinite(j2_output.acceleration_eci_mps2.y) ||
            !std::isfinite(j2_output.acceleration_eci_mps2.z) ||
            post2::vehicle::norm(j2_delta) <= 1.0e-6) {
            std::cerr << "J2 gravity perturbation was not finite/nonzero\n";
            return 1;
        }

        post2::core::SimulationConfig point_config;
        point_config.gravity_model.type = "point_mass";
        const auto legacy_point =
            post2::propagation::gravity_acceleration_mps2(
                point_config,
                gravity_state.motion.position_m);
        if (std::abs(legacy_point.x - expected_point.x) > 1.0e-12 ||
            std::abs(legacy_point.y - expected_point.y) > 1.0e-12 ||
            std::abs(legacy_point.z - expected_point.z) > 1.0e-12) {
            std::cerr << "legacy point-mass gravity helper changed formula\n";
            return 1;
        }
    }

    {
        post2::core::CaseConfig drag_case;
        drag_case.earth_rotation_rad_per_s = 0.0;
        drag_case.vehicle = post2::vehicle::default_vehicle_config();
        drag_case.vehicle.aero.enabled = true;
        drag_case.vehicle.aero.reference_area_m2 = 2.0;
        drag_case.vehicle.aero.cd = 1.5;

        post2::core::PhaseConfig drag_phase;
        drag_phase.force_models.aerodynamic = true;

        post2::vehicle::VehicleRuntimeState runtime;
        runtime.vehicle.total_mass_kg = 100.0;

        post2::propagation::EnvironmentState environment;
        environment.time_s = 0.0;
        environment.density_kgpm3 = 1.0;

        post2::integrators::ExtendedState drag_state;
        drag_state.motion.position_m = {post2::core::kEarthRadiusM + 1000.0, 0.0, 0.0};
        drag_state.motion.velocity_mps = {10.0, 0.0, 0.0};

        const post2::propagation::ForceModelContext drag_context{
            &drag_case,
            &drag_phase,
            &runtime,
            &environment,
            {},
        };
        post2::propagation::AtmosphericDragModel drag_model;
        const auto drag_output = drag_model.evaluate(drag_context, drag_state);
        if (std::abs(drag_output.acceleration_eci_mps2.x + 1.5) > 1.0e-12 ||
            std::abs(drag_output.acceleration_eci_mps2.y) > 1.0e-12 ||
            std::abs(drag_output.acceleration_eci_mps2.z) > 1.0e-12) {
            std::cerr << "atmospheric drag force model returned wrong acceleration\n";
            return 1;
        }
    }

    post2::vehicle::VehicleConfig vehicle_config = post2::vehicle::default_vehicle_config();
    vehicle_config.name = "smoke";
    vehicle_config.dry_mass_kg = 1200.0;
    vehicle_config.aero.enabled = true;
    vehicle_config.aero.reference_area_m2 = 12.5;
    vehicle_config.aero.cd = 0.6;
    vehicle_config.aero.cl = 0.05;
    vehicle_config.aero.aero_table_path = "aero.csv";
    vehicle_config.rigid_body.moment_of_inertia_kgm2 = 12345.0;
    vehicle_config.rigid_body.initial_attitude_rad = 0.12;
    vehicle_config.rigid_body.initial_angular_velocity_radps = -0.03;
    vehicle_config.rigid_body.engine_moment_arm_m = 4.5;
    vehicle_config.engine.enabled = true;
    vehicle_config.engine.thrust_vac_n = 4500.0;
    vehicle_config.engine.isp_vac_s = 310.0;
    vehicle_config.tanks.front().capacity_kg = 200.0;
    vehicle_config.tanks.front().initial_kg = 150.0;
    vehicle_config.stages = post2::vehicle::effective_stage_configs(vehicle_config);
    vehicle_config.stages.front().name = "core";
    vehicle_config.stages.front().dry_mass_kg = 900.0;
    vehicle_config.stages.front().engine.feed_tanks = {{"core", "main"}};
    post2::vehicle::StageConfig booster = vehicle_config.stages.front();
    booster.name = "booster";
    booster.dry_mass_kg = 300.0;
    booster.engine.thrust_vac_n = 2500.0;
    booster.engine.feed_tanks = {{"booster", "main"}};
    booster.tanks.front().capacity_kg = 100.0;
    booster.tanks.front().initial_kg = 80.0;
    vehicle_config.stages.push_back(booster);
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&vehicle_config);

    const auto text = post2::vehicle::vehicle_config_to_text(vehicle_config);
    post2::vehicle::VehicleConfig loaded_vehicle_config;
    std::string error;
    if (!post2::vehicle::vehicle_config_from_text(text, &loaded_vehicle_config, &error)) {
        std::cerr << "vehicle config roundtrip failed: " << error << '\n';
        return 1;
    }
    if (loaded_vehicle_config.name != "smoke" ||
        !loaded_vehicle_config.engine.enabled ||
        loaded_vehicle_config.engine.thrust_vac_n != 4500.0 ||
        loaded_vehicle_config.tanks.front().initial_kg != 150.0 ||
        loaded_vehicle_config.stages.size() != 2 ||
        loaded_vehicle_config.stages[0].name != "core" ||
        !loaded_vehicle_config.stages[0].attached ||
        loaded_vehicle_config.stages[0].dry_mass_kg != 900.0 ||
        loaded_vehicle_config.stages[1].name != "booster" ||
        loaded_vehicle_config.stages[1].dry_mass_kg != 300.0 ||
        loaded_vehicle_config.stages[1].tanks.front().initial_kg != 80.0 ||
        !loaded_vehicle_config.aero.enabled ||
        loaded_vehicle_config.aero.reference_area_m2 != 12.5 ||
        loaded_vehicle_config.aero.cd != 0.6 ||
        loaded_vehicle_config.aero.cl != 0.05 ||
        loaded_vehicle_config.aero.aero_table_path != "aero.csv" ||
        loaded_vehicle_config.rigid_body.moment_of_inertia_kgm2 != 12345.0 ||
        loaded_vehicle_config.rigid_body.initial_attitude_rad != 0.12 ||
        loaded_vehicle_config.rigid_body.initial_angular_velocity_radps != -0.03 ||
        loaded_vehicle_config.rigid_body.engine_moment_arm_m != 4.5) {
        std::cerr << "vehicle config roundtrip changed values\n";
        return 1;
    }

    post2::core::SimulationConfig burn_config;
    burn_config.duration_s = 20.0;
    burn_config.step_s = 10.0;
    burn_config.vehicle = loaded_vehicle_config;
    const auto burn_result = service.simulate(burn_config);
    if (!burn_result.ok) {
        std::cerr << "burn simulation failed: " << burn_result.error << '\n';
        return 1;
    }
    if (burn_result.state_log.back().propellant_mass_kg >= burn_result.state_log.front().propellant_mass_kg ||
        burn_result.state_log.back().engine_thrust_n <= 0.0) {
        std::cerr << "engine runtime state did not consume propellant\n";
        return 1;
    }

    post2::core::SimulationConfig hdc_config;
    hdc_config.duration_s = 120.0;
    hdc_config.step_s = 10.0;
    hdc_config.launch_site.latitude_deg = 28.5;
    hdc_config.launch_site.longitude_deg = -80.6;
    hdc_config.hold_down_clamp.enabled = true;
    hdc_config.hold_down_clamp.release_time_s = 60.0;
    const auto hdc_result = service.simulate(hdc_config);
    if (!hdc_result.ok) {
        std::cerr << "HDC simulation failed: " << hdc_result.error << '\n';
        return 1;
    }
    if (!hdc_result.state_log.front().hold_down_clamp_active ||
        hdc_result.state_log.back().hold_down_clamp_active) {
        std::cerr << "HDC active state was not recorded correctly\n";
        return 1;
    }
    if (std::abs(hdc_result.state_log.back().altitude_m) > 1.0) {
        std::cerr << "normal force did not keep released ground vehicle on surface: altitude="
                  << hdc_result.state_log.back().altitude_m << '\n';
        return 1;
    }

    const auto hdc_csv = post2::core::trajectory_to_csv(hdc_result.state_log);
    const auto parsed_hdc = post2::core::trajectory_from_csv(hdc_csv);
    if (!parsed_hdc.ok ||
        !parsed_hdc.state_log.front().hold_down_clamp_active ||
        parsed_hdc.state_log.back().hold_down_clamp_active) {
        std::cerr << "HDC CSV roundtrip failed\n";
        return 1;
    }

    post2::core::CaseConfig case_config;
    case_config.name = "roundtrip";
    case_config.step_s = 10.0;
    case_config.vehicle = loaded_vehicle_config;
    case_config.vehicle.aero.use_table = true;
    case_config.vehicle.aero.stage_tables = {
        post2::vehicle::AeroStageTable{0, "aero_full.csv", 21.2, 5.2, 70.0, 12.6, 3.66},
        post2::vehicle::AeroStageTable{1, "aero_stage2.csv", 21.2, 5.2, 15.0, 12.6, 3.66},
    };
    case_config.launch_site.latitude_deg = 28.5;
    case_config.launch_site.longitude_deg = -80.6;
    case_config.earth_j2 = 1.0827e-3;
    case_config.phases.clear();

    post2::core::PhaseConfig hdc_phase;
    hdc_phase.name = "hold";
    hdc_phase.controller_stage_index = 1;
    hdc_phase.controller_stage_name = "booster";
    hdc_phase.termination.value = 60.0;
    hdc_phase.optimize_enabled = true;
    hdc_phase.inherit_initial_state = false;
    hdc_phase.hold_down_clamp_initial_active = true;
    hdc_phase.throttle_model.type = "poly";
    hdc_phase.throttle_model.c0 = 0.5;
    hdc_phase.steering_model.type = "generic_poly";
    hdc_phase.steering_model.azimuth_deg.c0 = 90.0;
    hdc_phase.steering_model.elevation_deg.c0 = 5.0;
    hdc_phase.force_models.gravity_model.type = "j2";
    hdc_phase.force_models.gravity_model.j2 = case_config.earth_j2;
    hdc_phase.force_models.gravity_model.degree = 2;
    hdc_phase.force_models.gravity_model.order = 0;
    hdc_phase.force_models.aerodynamic = true;
    hdc_phase.force_models.atmosphere_model.type = "none";
    hdc_phase.actions.push_back({30.0, "set_hold_down_clamp_active", false});
    case_config.phases.push_back(hdc_phase);

    post2::core::PhaseConfig coast_phase;
    coast_phase.name = "coast";
    coast_phase.termination.value = 20.0;
    coast_phase.optimize_enabled = false;
    coast_phase.inherit_initial_state = true;
    coast_phase.force_models.thrust = false;
    coast_phase.force_models.gravity_model.type = "point_mass";
    coast_phase.throttle_model.type = "t2w";
    coast_phase.throttle_model.target_t2w = 1.1;
    post2::core::PhaseAction booster_inactive;
    booster_inactive.time_s = 0.0;
    booster_inactive.type = "set_stage_active";
    booster_inactive.value = false;
    booster_inactive.stage_index = 1;
    booster_inactive.stage_name = "booster";
    coast_phase.actions.push_back(booster_inactive);
    post2::core::PhaseAction booster_detached;
    booster_detached.time_s = 0.0;
    booster_detached.type = "set_stage_attached";
    booster_detached.value = false;
    booster_detached.stage_index = 1;
    booster_detached.stage_name = "booster";
    coast_phase.actions.push_back(booster_detached);
    case_config.phases.push_back(coast_phase);
    case_config.optimization.max_iterations = 12;
    case_config.optimization.tolerance = 1.0e-3;
    case_config.optimization.variables.push_back({"phases[0].termination.value", true, 30.0, 90.0});
    case_config.optimization.targets.push_back({"terminal_altitude_m", "equal", 0.0, 0.0, 0.0, 2.0});
    case_config.optimization.objective.enabled = true;
    case_config.optimization.objective.metric = "terminal_speed_mps";
    case_config.optimization.objective.direction = "minimize";
    case_config.optimization.objectives.push_back(case_config.optimization.objective);
    post2::core::OptimizationObjectiveConfig payload_objective;
    payload_objective.enabled = true;
    payload_objective.metric = "payload_mass_kg";
    payload_objective.direction = "maximize";
    payload_objective.weight = 0.25;
    case_config.optimization.objectives.push_back(payload_objective);

    const auto case_json = post2::core::case_config_to_json(case_config);
    post2::core::CaseConfig loaded_case;
    if (!post2::core::case_config_from_json(case_json, &loaded_case, &error)) {
        std::cerr << "case JSON roundtrip parse failed: " << error << '\n';
        return 1;
    }
    if (loaded_case.vehicle.name != loaded_vehicle_config.name ||
        loaded_case.vehicle.stages.size() != 2 ||
        loaded_case.vehicle.stages[1].name != "booster" ||
        loaded_case.phases.size() != 2 ||
        loaded_case.phases[0].controller_stage_index != 1 ||
        loaded_case.phases[0].controller_stage_name != "booster" ||
        loaded_case.earth_j2 != case_config.earth_j2 ||
        !loaded_case.phases[0].optimize_enabled ||
        loaded_case.phases[1].optimize_enabled ||
        loaded_case.phases[0].force_models.gravity_model.type != "j2" ||
        loaded_case.phases[0].force_models.gravity_model.j2 != case_config.earth_j2 ||
        loaded_case.phases[0].force_models.gravity_model.degree != 2 ||
        loaded_case.phases[0].force_models.gravity_model.order != 0 ||
        !loaded_case.phases[0].force_models.aerodynamic ||
        loaded_case.phases[0].force_models.atmosphere_model.type != "none" ||
        loaded_case.phases[1].force_models.gravity_model.type != "point_mass" ||
        !loaded_case.vehicle.aero.enabled ||
        loaded_case.vehicle.aero.reference_area_m2 != 12.5 ||
        loaded_case.vehicle.aero.cd != 0.6 ||
        loaded_case.vehicle.aero.cl != 0.05 ||
        loaded_case.vehicle.aero.aero_table_path != "aero.csv" ||
        !loaded_case.vehicle.aero.use_table ||
        loaded_case.vehicle.aero.stage_tables.size() != 2 ||
        loaded_case.vehicle.aero.stage_tables[1].activate_at_min_attached_stage != 1 ||
        loaded_case.vehicle.aero.stage_tables[1].table_path != "aero_stage2.csv" ||
        loaded_case.vehicle.aero.stage_tables[0].reference_area_m2 != 21.2 ||
        loaded_case.vehicle.rigid_body.moment_of_inertia_kgm2 != 12345.0 ||
        loaded_case.vehicle.rigid_body.initial_attitude_rad != 0.12 ||
        loaded_case.vehicle.rigid_body.initial_angular_velocity_radps != -0.03 ||
        loaded_case.vehicle.rigid_body.engine_moment_arm_m != 4.5 ||
        loaded_case.phases[0].actions.size() != 1 ||
        loaded_case.phases[0].steering_model.azimuth_deg.c0 != 90.0 ||
        loaded_case.phases[1].throttle_model.type != "t2w" ||
        loaded_case.phases[1].actions.size() != 2 ||
        loaded_case.phases[1].actions[0].type != "set_stage_active" ||
        loaded_case.phases[1].actions[0].value ||
        loaded_case.phases[1].actions[0].stage_index != 1 ||
        loaded_case.phases[1].actions[0].stage_name != "booster" ||
        loaded_case.phases[1].actions[1].type != "set_stage_attached" ||
        loaded_case.phases[1].actions[1].value ||
        loaded_case.phases[1].actions[1].stage_index != 1 ||
        loaded_case.phases[1].actions[1].stage_name != "booster" ||
        loaded_case.optimization.max_iterations != 12 ||
        loaded_case.optimization.variables.size() != 1 ||
        !loaded_case.optimization.variables[0].enabled ||
        loaded_case.optimization.variables[0].path != "phases[0].termination.value" ||
        loaded_case.optimization.targets.size() != 1 ||
        loaded_case.optimization.targets[0].weight != 2.0 ||
        !loaded_case.optimization.objective.enabled ||
        loaded_case.optimization.objective.metric != "terminal_speed_mps" ||
        loaded_case.optimization.objectives.size() != 2 ||
        loaded_case.optimization.objectives[1].metric != "payload_mass_kg") {
        std::cerr << "case JSON roundtrip changed values\n";
        return 1;
    }

    const std::string case_request = post2::core::make_remote_request(loaded_case);
    post2::core::CaseConfig parsed_case_request;
    if (!post2::core::parse_remote_request(case_request, &parsed_case_request, &error) ||
        parsed_case_request.phases.size() != 2 ||
        parsed_case_request.vehicle.stages.size() != 2 ||
        parsed_case_request.phases[0].name != "hold") {
        std::cerr << "CASEJSON remote request roundtrip failed: " << error << '\n';
        return 1;
    }

    post2::core::SimulationConfig sim_request_config;
    sim_request_config.vehicle = loaded_vehicle_config;
    const std::string sim_request = post2::core::make_remote_request(sim_request_config);
    post2::core::SimulationConfig parsed_sim_request;
    if (!post2::core::parse_remote_request(sim_request, &parsed_sim_request, &error) ||
        !parsed_sim_request.vehicle.aero.enabled ||
        parsed_sim_request.vehicle.aero.reference_area_m2 != 12.5 ||
        parsed_sim_request.vehicle.aero.cd != 0.6 ||
        parsed_sim_request.vehicle.aero.cl != 0.05 ||
        parsed_sim_request.vehicle.aero.aero_table_path != "aero.csv") {
        std::cerr << "SIMV4 remote request roundtrip failed: " << error << '\n';
        return 1;
    }
    post2::core::CaseConfig parsed_sim_case_request;
    if (!post2::core::parse_remote_request(sim_request, &parsed_sim_case_request, &error) ||
        parsed_sim_case_request.phases.empty() ||
        !parsed_sim_case_request.phases.front().force_models.aerodynamic) {
        std::cerr << "SIMV4 case conversion did not preserve aero enablement: " << error << '\n';
        return 1;
    }

    const auto case_result = service.simulate(loaded_case);
    if (!case_result.ok) {
        std::cerr << "case simulation failed: " << case_result.error << '\n';
        return 1;
    }
    if (!case_result.state_log.front().hold_down_clamp_active ||
        case_result.state_log.back().hold_down_clamp_active ||
        case_result.state_log.back().phase_name != "coast") {
        std::cerr << "case phase/HDC state was not recorded correctly\n";
        return 1;
    }
    if (case_result.state_log.back().runtime.stages.size() != 2 ||
        case_result.state_log.back().runtime.stages[1].active ||
        case_result.state_log.back().runtime.stages[1].attached) {
        std::cerr << "stage action did not deactivate and detach booster\n";
        return 1;
    }

    {
        const auto plain_case_csv = post2::core::trajectory_to_csv(case_result.state_log);
        if (plain_case_csv.find("kos_") != std::string::npos ||
            !post2::core::trajectory_from_csv(plain_case_csv).ok) {
            std::cerr << "normal trajectory CSV changed after adding kOS export\n";
            return 1;
        }

        const auto split_csv = [](const std::string& line) {
            std::vector<std::string> parts;
            std::istringstream input(line);
            std::string item;
            while (std::getline(input, item, ',')) {
                parts.push_back(item);
            }
            return parts;
        };
        const auto find_col = [](const std::vector<std::string>& header, const std::string& name) {
            const auto it = std::find(header.begin(), header.end(), name);
            return it == header.end() ? -1 : static_cast<int>(std::distance(header.begin(), it));
        };
        const auto value_at = [](const std::vector<std::string>& row, int index) {
            return index >= 0 && static_cast<std::size_t>(index) < row.size()
                ? std::stod(row[static_cast<std::size_t>(index)])
                : 0.0;
        };

        // Guidance script CSV: sectioned per-phase poly params consumed by the
        // standalone post2_player. Validate the record structure and that the
        // first powered phase's poly + throttle and the staging actions survive.
        (void)find_col;
        const auto guidance_csv = post2::core::guidance_script_to_csv(loaded_case);
        std::istringstream input(guidance_csv);
        std::string line;

        bool saw_meta = false;
        int stage_rows = 0;
        bool phase0_generic_poly = false;
        bool phase0_poly_throttle = false;
        bool phase0_poly_angles = false;
        bool phase1_non_poly_throttle = false;
        int phase1_action_rows = 0;
        bool phase1_stage_active_action = false;
        bool phase1_stage_attached_action = false;
        while (std::getline(input, line)) {
            if (line.empty()) {
                continue;
            }
            const auto row = split_csv(line);
            if (row.empty()) {
                continue;
            }
            const std::string& record = row[0];
            if (record == "META") {
                // META,name,mu,radius,...
                saw_meta = row.size() >= 4 &&
                    std::abs(value_at(row, 2) - loaded_case.earth_mu_m3s2) < 1.0e-3 &&
                    std::abs(value_at(row, 3) - loaded_case.earth_radius_m) < 1.0e-6;
            } else if (record == "STAGE") {
                ++stage_rows;
            } else if (record == "PHASE") {
                // PHASE,phase_index,name,steering_type,...,throttle_type,...
                const int phase_index = static_cast<int>(value_at(row, 1));
                const std::string steering_type = row.size() > 3 ? row[3] : "";
                const std::string throttle_type = row.size() > 7 ? row[7] : "";
                if (phase_index == 0) {
                    phase0_generic_poly = steering_type == "generic_poly";
                    phase0_poly_throttle = throttle_type == "poly" &&
                        std::abs(value_at(row, 8) - 0.5) < 1.0e-12;
                }
                if (phase_index == 1) {
                    phase1_non_poly_throttle = throttle_type == "t2w";
                }
            } else if (record == "POLY") {
                // POLY,phase_index,az_c0,az_c1,az_c2,el_c0,el_c1,el_c2,roll_c0,...
                if (static_cast<int>(value_at(row, 1)) == 0) {
                    phase0_poly_angles =
                        std::abs(value_at(row, 2) - 90.0) < 1.0e-12 &&
                        std::abs(value_at(row, 5) - 5.0) < 1.0e-12 &&
                        std::abs(value_at(row, 8)) < 1.0e-12;
                }
            } else if (record == "ACTION") {
                // ACTION,phase_index,time_s,type,value,stage_index,stage_name
                if (static_cast<int>(value_at(row, 1)) == 1) {
                    ++phase1_action_rows;
                    const std::string type = row.size() > 3 ? row[3] : "";
                    const int stage_index = static_cast<int>(value_at(row, 5));
                    if (type == "set_stage_active" && stage_index == 1) {
                        phase1_stage_active_action = true;
                    }
                    if (type == "set_stage_attached" && stage_index == 1) {
                        phase1_stage_attached_action = true;
                    }
                }
            }
        }

        if (!saw_meta ||
            stage_rows != static_cast<int>(loaded_case.vehicle.stages.size()) ||
            !phase0_generic_poly ||
            !phase0_poly_throttle ||
            !phase0_poly_angles ||
            !phase1_non_poly_throttle ||
            phase1_action_rows != 2 ||
            !phase1_stage_active_action ||
            !phase1_stage_attached_action) {
            std::cerr << "guidance script CSV did not encode expected per-phase guidance/actions\n";
            return 1;
        }
    }

    const auto metrics = post2::core::evaluate_trajectory_metrics(case_result.state_log, loaded_case);
    auto metric_value = [&](const std::string& name, double* value) {
        const auto it = std::find_if(metrics.begin(), metrics.end(), [&](const auto& candidate) {
            return candidate.metric == name;
        });
        if (it == metrics.end()) {
            return false;
        }
        *value = it->value;
        return true;
    };
    double terminal_altitude_m = 0.0;
    double terminal_speed_mps = 0.0;
    double inclination_deg = 0.0;
    double periapsis_altitude_m = 0.0;
    double payload_mass_kg = 0.0;
    if (!metric_value("terminal_altitude_m", &terminal_altitude_m) ||
        !metric_value("terminal_speed_mps", &terminal_speed_mps) ||
        !metric_value("inclination_deg", &inclination_deg) ||
        !metric_value("periapsis_altitude_m", &periapsis_altitude_m) ||
        !metric_value("payload_mass_kg", &payload_mass_kg) ||
        std::abs(terminal_altitude_m - case_result.state_log.back().altitude_m) > 1.0e-9 ||
        std::abs(terminal_speed_mps - case_result.state_log.back().speed_mps) > 1.0e-9 ||
        !std::isfinite(inclination_deg) ||
        !std::isfinite(periapsis_altitude_m)) {
        std::cerr << "trajectory metrics were not stable\n";
        return 1;
    }

    post2::core::CaseConfig force_off_case;
    force_off_case.step_s = 10.0;
    force_off_case.vehicle = post2::vehicle::default_vehicle_config();
    force_off_case.phases.clear();
    post2::core::PhaseConfig force_off_phase;
    force_off_phase.name = "force-off";
    force_off_phase.termination.value = 10.0;
    force_off_phase.inherit_initial_state = false;
    force_off_phase.initial_state_eci = post2::core::State{
        {post2::core::kEarthRadiusM + 1000.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
    };
    force_off_phase.force_models.gravity = false;
    force_off_phase.force_models.thrust = false;
    force_off_phase.force_models.normal_force = false;
    force_off_case.phases.push_back(force_off_phase);
    const auto force_off_result = service.simulate(force_off_case);
    if (!force_off_result.ok) {
        std::cerr << "force-off simulation failed: " << force_off_result.error << '\n';
        return 1;
    }
    if (std::abs(force_off_result.state_log.back().state.position_m.x -
            force_off_result.state_log.front().state.position_m.x) > 1.0e-6 ||
        force_off_result.state_log.back().speed_mps > 1.0e-9) {
        std::cerr << "force switches did not disable acceleration\n";
        return 1;
    }

    post2::core::CaseConfig impact_case;
    impact_case.step_s = 10.0;
    impact_case.vehicle = post2::vehicle::default_vehicle_config();
    impact_case.phases.clear();
    post2::core::PhaseConfig impact_phase;
    impact_phase.name = "impact";
    impact_phase.termination.value = 50.0;
    impact_phase.inherit_initial_state = false;
    impact_phase.initial_state_eci = post2::core::State{
        {post2::core::kEarthRadiusM + 1000.0, 0.0, 0.0},
        {-100.0, 0.0, 0.0},
    };
    impact_phase.force_models.thrust = false;
    impact_phase.force_models.normal_force = false;
    impact_phase.force_models.gravity_model.type = "point_mass";
    impact_case.phases.push_back(impact_phase);
    const auto impact_result = service.simulate(impact_case);
    if (impact_result.ok ||
        impact_result.error != "vehicle impacted Earth during propagation" ||
        impact_result.state_log.empty() ||
        impact_result.state_log.back().altitude_m < -50.0 ||
        impact_result.state_log.back().time_s >= impact_phase.termination.value) {
        std::cerr << "impact event did not terminate propagation at the surface\n";
        return 1;
    }

    post2::core::CaseConfig drop_case;
    drop_case.step_s = 0.5;
    drop_case.vehicle = post2::vehicle::default_vehicle_config();
    drop_case.vehicle.name = "payload optimizer";
    drop_case.vehicle.dry_mass_kg = 1200.0;
    post2::vehicle::StageConfig bus;
    bus.name = "bus";
    bus.active = true;
    bus.attached = true;
    bus.dry_mass_kg = 800.0;
    bus.tanks = {};
    post2::vehicle::StageConfig payload;
    payload.name = "payload";
    payload.active = false;
    payload.attached = true;
    payload.dry_mass_kg = 100.0;
    payload.tanks = {};
    drop_case.vehicle.stages = {bus, payload};
    drop_case.phases.clear();
    post2::core::PhaseConfig drop_phase;
    drop_phase.name = "drop";
    drop_phase.termination.value = 1.0;
    drop_phase.inherit_initial_state = false;
    drop_phase.initial_state_eci = post2::core::State{
        {post2::core::kEarthRadiusM + 1000.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
    };
    drop_phase.force_models.thrust = false;
    drop_phase.force_models.normal_force = false;
    drop_case.phases.push_back(drop_phase);
    drop_case.optimization.max_iterations = 40;
    drop_case.optimization.tolerance = 1.0e-2;
    drop_case.optimization.variables.push_back({"phases[0].termination.value", true, 1.0, 5.0});
    drop_case.optimization.variables.push_back({"vehicle.stages[1].dry_mass_kg", true, 100.0, 250.0});
    drop_case.optimization.targets.push_back({"terminal_altitude_m", "equal", 950.0, 0.0, 0.0, 1.0});
    drop_case.optimization.objective.enabled = true;
    drop_case.optimization.objective.metric = "payload_mass_kg";
    drop_case.optimization.objective.direction = "maximize";
    drop_case.optimization.objective.weight = 0.01;

    const auto before_drop = service.simulate(drop_case);
    if (!before_drop.ok) {
        std::cerr << "drop simulation failed before optimize: " << before_drop.error << '\n';
        return 1;
    }
    const double before_residual = std::abs(before_drop.state_log.back().altitude_m - 950.0);
    const auto optimize_result = post2::core::optimize_case(&drop_case, service);
    if (!optimize_result.ok) {
        std::cerr << "optimizer smoke failed: " << optimize_result.error << '\n';
        return 1;
    }
    const double after_residual = std::abs(optimize_result.final_simulation.state_log.back().altitude_m - 950.0);
    const auto payload_metric = std::find_if(
        optimize_result.final_metrics.begin(),
        optimize_result.final_metrics.end(),
        [](const auto& metric) {
            return metric.metric == "payload_mass_kg";
        });
    if (after_residual >= before_residual ||
        !drop_case.optimization.variables[0].enabled ||
        !drop_case.optimization.variables[1].enabled ||
        !drop_case.phases[0].optimize_enabled ||
        optimize_result.variable_changes.empty() ||
        payload_metric == optimize_result.final_metrics.end() ||
        payload_metric->value < 100.0 ||
        optimize_result.variable_changes[0].new_value <= optimize_result.variable_changes[0].old_value) {
        std::cerr << "optimizer did not improve target or preserve case flags\n";
        return 1;
    }

    const auto runtime = post2::vehicle::make_initial_runtime_state(
        loaded_vehicle_config,
        {{post2::core::kEarthRadiusM + 1000.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
        0.0);
    post2::core::PhaseContext context{&loaded_case, &loaded_case.phases.front(), 0, 0.0};

    post2::core::ThrottleModelConfig poly_throttle;
    poly_throttle.type = "poly";
    poly_throttle.c0 = 0.2;
    poly_throttle.c1 = 0.1;
    poly_throttle.c2 = 0.05;
    const auto poly_model = post2::core::make_throttle_model(poly_throttle);
    if (std::abs(poly_model->throttle(2.0, runtime, context) - 0.6) > 1.0e-12) {
        std::cerr << "poly throttle model returned unexpected value\n";
        return 1;
    }

    post2::core::ThrottleModelConfig interp_throttle;
    interp_throttle.type = "interpolated";
    interp_throttle.points = {{0.0, 0.0}, {10.0, 1.0}};
    const auto interp_model = post2::core::make_throttle_model(interp_throttle);
    if (std::abs(interp_model->throttle(5.0, runtime, context) - 0.5) > 1.0e-12 ||
        interp_model->throttle(-1.0, runtime, context) != 0.0 ||
        interp_model->throttle(11.0, runtime, context) != 1.0) {
        std::cerr << "interpolated throttle model returned unexpected values\n";
        return 1;
    }

    post2::core::ThrottleModelConfig t2w_throttle;
    t2w_throttle.type = "t2w";
    t2w_throttle.target_t2w = 100.0;
    const auto t2w_model = post2::core::make_throttle_model(t2w_throttle);
    if (t2w_model->throttle(0.0, runtime, context) != 1.0) {
        std::cerr << "T2W throttle model did not clamp to 1\n";
        return 1;
    }

    post2::core::State steering_state{{post2::core::kEarthRadiusM + 1000.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    post2::core::SteeringModelConfig generic_steering;
    generic_steering.type = "generic_poly";
    generic_steering.azimuth_deg.c0 = 90.0;
    generic_steering.elevation_deg.c0 = 0.0;
    const auto generic_model = post2::core::make_steering_model(generic_steering);
    const auto east = generic_model->thrust_direction_eci(0.0, steering_state, runtime, context);
    if (std::abs(east.x) > 1.0e-9 || std::abs(east.y - 1.0) > 1.0e-9 || std::abs(post2::vehicle::norm(east) - 1.0) > 1.0e-9) {
        std::cerr << "generic steering did not produce expected east unit direction\n";
        return 1;
    }

    post2::core::ThrottleModelConfig segmented_throttle;
    segmented_throttle.type = "segmented_poly";
    segmented_throttle.segmented_poly.order = 1;
    segmented_throttle.segmented_poly.segments = {
        {0.0, {0.2, 0.1}},
        {5.0, {0.8, -0.1}},
    };
    const auto segmented_throttle_model = post2::core::make_throttle_model(segmented_throttle);
    if (std::abs(segmented_throttle_model->throttle(2.0, runtime, context) - 0.4) > 1.0e-12 ||
        std::abs(segmented_throttle_model->throttle(7.0, runtime, context) - 0.6) > 1.0e-12) {
        std::cerr << "segmented throttle model returned unexpected values\n";
        return 1;
    }

    post2::core::SteeringModelConfig segmented_steering;
    segmented_steering.type = "segmented_poly";
    segmented_steering.segmented_poly.order = 1;
    segmented_steering.segmented_poly.segments = {
        {0.0, {90.0, 0.0}, {0.0, 0.0}},
        {5.0, {0.0, 0.0}, {0.0, 0.0}},
    };
    const auto segmented_steering_model = post2::core::make_steering_model(segmented_steering);
    const auto segmented_east = segmented_steering_model->thrust_direction_eci(2.0, steering_state, runtime, context);
    const auto segmented_north = segmented_steering_model->thrust_direction_eci(7.0, steering_state, runtime, context);
    if (std::abs(segmented_east.y - 1.0) > 1.0e-9 ||
        std::abs(segmented_north.z - 1.0) > 1.0e-9) {
        std::cerr << "segmented steering model did not switch segments correctly\n";
        return 1;
    }

    post2::core::SteeringModelConfig quat_steering;
    quat_steering.type = "generic_quat_interp";
    quat_steering.points = {
        {0.0, {1.0, 0.0, 0.0, 0.0}},
        {10.0, {0.7071067811865476, 0.0, 0.0, 0.7071067811865476}},
    };
    const auto quat_model = post2::core::make_steering_model(quat_steering);
    const auto quat_end = quat_model->thrust_direction_eci(10.0, steering_state, runtime, context);
    if (std::abs(quat_end.x) > 1.0e-9 || std::abs(quat_end.y - 1.0) > 1.0e-9) {
        std::cerr << "quat steering did not rotate body +X to +Y\n";
        return 1;
    }

    // (a) T2T mass-conservation: two-stage vehicle, engines off, one T2T flow
    // moves 1 kg/s from booster tank to upper tank. Total propellant must stay
    // constant within tight tolerance.
    {
        post2::core::CaseConfig t2t_case;
        t2t_case.step_s = 1.0;
        t2t_case.vehicle.name = "t2t-conservation";
        t2t_case.vehicle.dry_mass_kg = 500.0;
        post2::vehicle::StageConfig src_stage;
        src_stage.name = "src_stage";
        src_stage.active = true;
        src_stage.dry_mass_kg = 250.0;
        src_stage.engine.enabled = false;
        src_stage.engine.thrust_vac_n = 0.0;
        src_stage.engine.isp_vac_s = 0.0;
        src_stage.tanks = {{"src_tank", "rp1", 200.0, 200.0}};
        post2::vehicle::StageConfig dst_stage = src_stage;
        dst_stage.name = "dst_stage";
        dst_stage.tanks = {{"dst_tank", "rp1", 200.0, 50.0}};
        t2t_case.vehicle.stages = {src_stage, dst_stage};
        t2t_case.vehicle.tank_to_tank_connections = {
            {{"src_stage", "src_tank"}, {"dst_stage", "dst_tank"}, 1.0, std::nullopt, std::nullopt},
        };
        post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&t2t_case.vehicle);

        post2::core::PhaseConfig phase;
        phase.name = "t2t";
        phase.termination.value = 60.0;
        phase.inherit_initial_state = false;
        phase.initial_state_eci = post2::core::State{
            {post2::core::kEarthRadiusM + 200000.0, 0.0, 0.0},
            {0.0, 7800.0, 0.0},
        };
        phase.force_models.thrust = false;
        phase.force_models.normal_force = false;
        phase.throttle_model.c0 = 0.0;
        t2t_case.phases = {phase};

        const auto t2t_result = service.simulate(t2t_case);
        if (!t2t_result.ok) {
            std::cerr << "T2T case simulation failed: " << t2t_result.error << '\n';
            return 1;
        }
        const auto& head = t2t_result.state_log.front().runtime;
        const auto& tail = t2t_result.state_log.back().runtime;
        const double head_total = head.vehicle.propellant_mass_kg;
        const double tail_total = tail.vehicle.propellant_mass_kg;
        if (std::abs(tail_total - head_total) > 1.0e-6) {
            std::cerr << "T2T did not conserve mass: head=" << head_total
                      << " tail=" << tail_total << '\n';
            return 1;
        }
        const double src_before = head.stages[0].tanks[0].remaining_kg;
        const double src_after = tail.stages[0].tanks[0].remaining_kg;
        const double dst_before = head.stages[1].tanks[0].remaining_kg;
        const double dst_after = tail.stages[1].tanks[0].remaining_kg;
        // ~60 kg moved (60 s * 1 kg/s); tolerate the smoothstep epsilon.
        if (std::abs((src_before - src_after) - 60.0) > 0.5 ||
            std::abs((dst_after - dst_before) - 60.0) > 0.5) {
            std::cerr << "T2T did not move ~60 kg src->dst: src delta="
                      << (src_before - src_after) << " dst delta="
                      << (dst_after - dst_before) << '\n';
            return 1;
        }
    }

    // (b) Feed-tank priority: stage with two tanks, feed_tanks lists them
    // in order; engine should drain the first tank before touching the
    // second.
    {
        post2::core::CaseConfig prio_case;
        prio_case.step_s = 0.5;
        prio_case.vehicle.name = "feed-priority";
        prio_case.vehicle.dry_mass_kg = 600.0;
        post2::vehicle::StageConfig stage;
        stage.name = "core";
        stage.active = true;
        stage.dry_mass_kg = 600.0;
        stage.engine.enabled = true;
        stage.engine.thrust_vac_n = 9806.65;  // 1 kg/s at isp=1000
        stage.engine.isp_vac_s = 1000.0;
        stage.engine.feed_tanks = {{"core", "tank_a"}, {"core", "tank_b"}};
        stage.tanks = {
            {"tank_a", "rp1", 30.0, 30.0},
            {"tank_b", "rp1", 30.0, 30.0},
        };
        prio_case.vehicle.stages = {stage};
        post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&prio_case.vehicle);

        post2::core::PhaseConfig phase;
        phase.name = "burn";
        phase.termination.value = 50.0;
        phase.inherit_initial_state = false;
        phase.initial_state_eci = post2::core::State{
            {post2::core::kEarthRadiusM + 200000.0, 0.0, 0.0},
            {0.0, 7800.0, 0.0},
        };
        phase.force_models.normal_force = false;
        phase.throttle_model.c0 = 1.0;
        post2::core::PhaseAction enable_action;
        enable_action.time_s = 0.0;
        enable_action.type = "set_engine_enabled";
        enable_action.value = true;
        phase.actions.push_back(enable_action);
        prio_case.phases = {phase};

        const auto prio_result = service.simulate(prio_case);
        if (!prio_result.ok) {
            std::cerr << "feed-priority simulation failed: " << prio_result.error << '\n';
            return 1;
        }
        // At t=20s: ~20 kg consumed; tank_a should be ~10 kg, tank_b still ~30.
        // At t=50s: tank_a empty, tank_b drained ~20 kg.
        const auto& entries = prio_result.state_log.entries();
        const post2::core::TrajectoryPoint* mid = nullptr;
        for (const auto& e : entries) {
            if (e.time_s >= 20.0) { mid = &e; break; }
        }
        if (!mid) {
            std::cerr << "feed-priority: no mid sample\n";
            return 1;
        }
        const double mid_a = mid->runtime.stages[0].tanks[0].remaining_kg;
        const double mid_b = mid->runtime.stages[0].tanks[1].remaining_kg;
        if (mid_a > 12.0 || mid_a < 7.0) {
            std::cerr << "feed-priority: tank_a at t=20 was " << mid_a
                      << " (expected ~10)\n";
            return 1;
        }
        if (std::abs(mid_b - 30.0) > 0.5) {
            std::cerr << "feed-priority: tank_b drained before tank_a empty: "
                      << mid_b << " (expected ~30)\n";
            return 1;
        }
        const auto& tail = entries.back().runtime;
        const double tail_a = tail.stages[0].tanks[0].remaining_kg;
        const double tail_b = tail.stages[0].tanks[1].remaining_kg;
        if (tail_a > 0.1) {
            std::cerr << "feed-priority: tank_a not drained, mass=" << tail_a << '\n';
            return 1;
        }
        if (tail_b > 12.0 || tail_b < 7.0) {
            std::cerr << "feed-priority: tank_b final was " << tail_b
                      << " (expected ~10)\n";
            return 1;
        }
    }

    // (c) Hold-down clamp + burn: engine fires while clamped, tank should
    // drain linearly while altitude stays pinned.
    {
        post2::core::CaseConfig clamp_case;
        clamp_case.step_s = 1.0;
        clamp_case.launch_site.latitude_deg = 28.5;
        clamp_case.launch_site.longitude_deg = -80.6;
        clamp_case.vehicle.name = "clamp-burn";
        clamp_case.vehicle.dry_mass_kg = 1000.0;
        post2::vehicle::StageConfig stage;
        stage.name = "core";
        stage.active = true;
        stage.dry_mass_kg = 1000.0;
        stage.engine.enabled = true;
        stage.engine.thrust_vac_n = 9806.65;  // 1 kg/s @ isp 1000 s
        stage.engine.isp_vac_s = 1000.0;
        stage.engine.feed_tanks = {{"core", "main"}};
        stage.tanks = {{"main", "rp1", 200.0, 100.0}};
        clamp_case.vehicle.stages = {stage};
        post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&clamp_case.vehicle);

        post2::core::PhaseConfig phase;
        phase.name = "clamp burn";
        phase.termination.value = 30.0;
        phase.inherit_initial_state = false;
        phase.hold_down_clamp_initial_active = true;
        phase.force_models.gravity = true;
        phase.force_models.thrust = true;
        phase.force_models.normal_force = true;
        phase.throttle_model.c0 = 1.0;
        post2::core::PhaseAction enable_engine;
        enable_engine.time_s = 0.0;
        enable_engine.type = "set_engine_enabled";
        enable_engine.value = true;
        phase.actions.push_back(enable_engine);
        clamp_case.phases = {phase};

        const auto clamp_result = service.simulate(clamp_case);
        if (!clamp_result.ok) {
            std::cerr << "clamp+burn simulation failed: " << clamp_result.error << '\n';
            return 1;
        }
        const auto& last = clamp_result.state_log.back();
        if (!last.hold_down_clamp_active) {
            std::cerr << "clamp+burn: clamp released unexpectedly\n";
            return 1;
        }
        if (std::abs(last.altitude_m) > 1.0e-3) {
            std::cerr << "clamp+burn: altitude drifted while clamped: "
                      << last.altitude_m << '\n';
            return 1;
        }
        const double drained_kg = 100.0 - last.runtime.stages[0].tanks[0].remaining_kg;
        // 1 kg/s for 30 s -> ~30 kg drained, allow smoothstep slop.
        if (drained_kg < 28.0 || drained_kg > 30.5) {
            std::cerr << "clamp+burn: drained " << drained_kg
                      << " kg (expected ~30)\n";
            return 1;
        }
    }

    // (d) DOPRI5 integrator: same LEO orbit as the headline check, but use
    // the adaptive integrator path end-to-end. Energy should be conserved to
    // a tight tolerance over a single period.
    {
        post2::core::CaseConfig dopri_case;
        dopri_case.step_s = 30.0;
        dopri_case.vehicle.dry_mass_kg = 1000.0;
        dopri_case.vehicle.tanks.front().capacity_kg = 0.0;
        post2::core::PhaseConfig phase;
        phase.name = "dopri5 leo";
        phase.termination.value = 5400.0;
        phase.inherit_initial_state = false;
        phase.initial_state_eci = post2::core::State{
            {post2::core::kEarthRadiusM + 200000.0, 0.0, 0.0},
            {0.0, 7784.0, 0.0},
        };
        phase.integrator = "dopri5";
        phase.tolerances.rtol = 1.0e-9;
        phase.force_models.gravity = true;
        phase.force_models.thrust = false;
        phase.force_models.normal_force = false;
        phase.force_models.gravity_model.type = "point_mass";
        phase.throttle_model.c0 = 0.0;
        dopri_case.phases = {phase};

        const auto dopri_result = service.simulate(dopri_case);
        if (!dopri_result.ok) {
            std::cerr << "dopri5 LEO simulation failed: " << dopri_result.error << '\n';
            return 1;
        }
        const auto& head = dopri_result.state_log.front();
        const auto& tail = dopri_result.state_log.back();
        const double r0 = head.radius_m;
        const double v0 = head.speed_mps;
        const double r1 = tail.radius_m;
        const double v1 = tail.speed_mps;
        const double e0 = 0.5 * v0 * v0 - post2::core::kEarthMuM3S2 / r0;
        const double e1 = 0.5 * v1 * v1 - post2::core::kEarthMuM3S2 / r1;
        if (std::abs(e1 - e0) / std::abs(e0) > 1.0e-6) {
            std::cerr << "dopri5 LEO did not conserve energy: e0=" << e0
                      << " e1=" << e1 << '\n';
            return 1;
        }
    }

    return 0;
}
