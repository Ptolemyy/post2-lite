#include "post2/core/case_config_io.hpp"
#include "post2/core/trajectory_service.hpp"
#include "post2/core/types.hpp"
#include "post2/vehicle/vehicle.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

post2::vehicle::VehicleConfig make_test_vehicle()
{
    post2::vehicle::VehicleConfig vehicle;
    vehicle.name = "2.5dof test vehicle";

    post2::vehicle::StageConfig stage;
    stage.name = "stage1";
    stage.active = true;
    stage.attached = true;
    stage.dry_mass_kg = 1000.0;
    stage.engine.enabled = true;
    stage.engine.thrust_vac_n = 20000.0;
    stage.engine.isp_vac_s = 300.0;
    stage.engine.engine_count = 1;
    stage.engine.direction_body = {1.0, -0.1, 0.0};

    post2::vehicle::TankConfig tank;
    tank.name = "main";
    tank.propellant = "generic";
    tank.capacity_kg = 1000.0;
    tank.initial_kg = 1000.0;
    stage.tanks = {tank};
    stage.engine.feed_tanks = {{"stage1", "main"}};

    vehicle.stages = {stage};
    vehicle.dry_mass_kg = stage.dry_mass_kg;
    vehicle.tanks = stage.tanks;
    vehicle.engine = stage.engine;
    vehicle.rigid_body.moment_of_inertia_kgm2 = 5000.0;
    vehicle.rigid_body.initial_attitude_rad = 0.0;
    vehicle.rigid_body.initial_angular_velocity_radps = 0.05;
    vehicle.rigid_body.engine_moment_arm_m = 2.0;
    return vehicle;
}

post2::core::CaseConfig make_planar_case(const std::string& dynamics_dof)
{
    post2::core::CaseConfig config;
    config.name = "2.5dof planar constraint";
    config.step_s = 0.1;
    config.earth_rotation_rad_per_s = 0.0;
    config.vehicle = make_test_vehicle();

    post2::core::PhaseConfig phase;
    phase.name = "powered planar turn";
    phase.dynamics_dof = dynamics_dof;
    phase.termination = {"time", ">=", 4.0};
    phase.inherit_initial_state = false;
    phase.optimize_enabled = false;
    phase.integrator = "rk4";
    phase.initial_state_eci = post2::core::State{
        {post2::core::kEarthRadiusM + 1000.0, 0.0, 0.0},
        {0.0, 100.0, 0.0},
    };
    phase.force_models.gravity = false;
    phase.force_models.thrust = true;
    phase.force_models.normal_force = false;
    phase.force_models.aerodynamic = false;
    phase.force_models.atmosphere_model.type = "none";
    phase.throttle_model.type = "poly";
    phase.throttle_model.c0 = 1.0;

    config.phases = {phase};
    return config;
}

post2::core::CaseConfig make_planar_case_with_steering_model()
{
    post2::core::CaseConfig config = make_planar_case("2.5dof");
    config.phases.front().steering_model.type = "generic_poly";
    config.phases.front().steering_model.azimuth_deg.c0 = 90.0;
    config.phases.front().steering_model.elevation_deg.c0 = 90.0;
    return config;
}

int test_dynamics_dof_tokens()
{
    post2::core::DynamicsDof dof = post2::core::DynamicsDof::ThreeDof;
    if (!post2::core::dynamics_dof_from_string("2.5dof", &dof) ||
        dof != post2::core::DynamicsDof::TwoPointFiveDof) {
        std::cerr << "2.5dof token did not parse\n";
        return 1;
    }
    if (std::string(post2::core::dynamics_dof_to_string(dof)) != "2.5dof") {
        std::cerr << "2.5dof token did not serialize\n";
        return 1;
    }
    if (post2::core::dynamics_dof_from_string("1.5dof", &dof)) {
        std::cerr << "legacy 1.5dof token should not parse\n";
        return 1;
    }
    return 0;
}

int test_case_json_roundtrip()
{
    const post2::core::CaseConfig config = make_planar_case("2.5dof");
    const std::string json = post2::core::case_config_to_json(config);
    if (json.find("\"2.5dof\"") == std::string::npos) {
        std::cerr << "2.5dof was not written to case JSON\n";
        return 1;
    }

    post2::core::CaseConfig reloaded;
    std::string error;
    if (!post2::core::case_config_from_json(json, &reloaded, &error)) {
        std::cerr << "2.5dof case JSON did not reload: " << error << '\n';
        return 1;
    }
    if (reloaded.phases.empty() || reloaded.phases.front().dynamics_dof != "2.5dof") {
        std::cerr << "2.5dof did not survive case JSON roundtrip\n";
        return 1;
    }
    return 0;
}

int test_legacy_name_rejected()
{
    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(make_planar_case("1.5dof"));
    if (result.ok) {
        std::cerr << "legacy 1.5dof case unexpectedly simulated\n";
        return 1;
    }
    if (result.error.find("2.5dof") == std::string::npos) {
        std::cerr << "legacy 1.5dof rejection did not mention 2.5dof: "
                  << result.error << '\n';
        return 1;
    }
    return 0;
}

int test_two_point_five_dof_rejects_steering_model()
{
    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(make_planar_case_with_steering_model());
    if (result.ok) {
        std::cerr << "2.5dof case with steering_model unexpectedly simulated\n";
        return 1;
    }
    if (result.error.find("does not support steering_model") == std::string::npos) {
        std::cerr << "2.5dof steering_model rejection was unclear: "
                  << result.error << '\n';
        return 1;
    }
    return 0;
}

int test_two_point_five_dof_planar_constraint()
{
    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(make_planar_case("2.5dof"));
    if (!result.ok) {
        std::cerr << "2.5dof case failed to simulate: " << result.error << '\n';
        return 1;
    }
    if (result.state_log.size() < 5) {
        std::cerr << "2.5dof case produced too few samples\n";
        return 1;
    }

    double max_abs_position_z = 0.0;
    double max_abs_velocity_z = 0.0;
    double max_abs_acceleration_z = 0.0;
    double max_abs_engine_direction_z = 0.0;
    double max_abs_angular_velocity = 0.0;
    double final_attitude = 0.0;
    bool saw_projected_engine_direction = false;
    for (std::size_t i = 0; i < result.state_log.entries().size(); ++i) {
        const auto& entry = result.state_log.entries()[i];
        max_abs_position_z = std::max(max_abs_position_z, std::abs(entry.state.position_m.z));
        max_abs_velocity_z = std::max(max_abs_velocity_z, std::abs(entry.state.velocity_mps.z));
        max_abs_acceleration_z = std::max(max_abs_acceleration_z, std::abs(entry.acceleration_eci_mps2.z));
        max_abs_angular_velocity =
            std::max(max_abs_angular_velocity, std::abs(entry.rigid_body_angular_velocity_radps));
        final_attitude = entry.rigid_body_attitude_rad;
        if (i > 0 && entry.engine_thrust_n > 0.0) {
            saw_projected_engine_direction = true;
            max_abs_engine_direction_z =
                std::max(max_abs_engine_direction_z, std::abs(entry.engine_direction_eci.z));
        }
    }

    if (max_abs_position_z > 1.0e-6 ||
        max_abs_velocity_z > 1.0e-9 ||
        max_abs_acceleration_z > 1.0e-9 ||
        !saw_projected_engine_direction ||
        max_abs_engine_direction_z > 1.0e-9 ||
        max_abs_angular_velocity <= 0.0 ||
        std::abs(final_attitude) <= 1.0e-6) {
        std::cerr << "2.5dof did not keep the trajectory in-plane:"
                  << " max_z_pos=" << max_abs_position_z
                  << " max_z_vel=" << max_abs_velocity_z
                  << " max_z_accel=" << max_abs_acceleration_z
                  << " max_z_engine_dir=" << max_abs_engine_direction_z
                  << " saw_engine_dir=" << saw_projected_engine_direction
                  << " max_abs_omega=" << max_abs_angular_velocity
                  << " final_attitude=" << final_attitude << '\n';
        return 1;
    }
    return 0;
}

} // namespace

int main()
{
    int failures = 0;
    failures += test_dynamics_dof_tokens();
    failures += test_case_json_roundtrip();
    failures += test_legacy_name_rejected();
    failures += test_two_point_five_dof_rejects_steering_model();
    failures += test_two_point_five_dof_planar_constraint();
    if (failures != 0) {
        std::cerr << "dynamics_dof_test: " << failures << " case(s) failed\n";
        return 1;
    }
    return 0;
}
