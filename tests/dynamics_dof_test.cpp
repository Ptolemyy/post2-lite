#include "post2/core/case_config_io.hpp"
#include "post2/core/trajectory_service.hpp"
#include "post2/core/types.hpp"
#include "post2/propagation/force_models.hpp"
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

// Two-phase case: a 3-DOF phase whose commanded thrust direction points out of
// the orbital plane, followed by a 2.5-DOF phase that inherits the state. The
// 3-DOF phase runs at zero throttle so it builds no out-of-plane velocity (the
// orbital plane stays exactly the x-y plane), letting us check the seeded pitch
// against a closed-form value.
constexpr double kTransitionAzimuthDeg = 45.0;
constexpr double kTransitionElevationDeg = 30.0;

post2::core::CaseConfig make_transition_case()
{
    post2::core::CaseConfig config;
    config.name = "3dof->2.5dof transition";
    config.step_s = 0.05;
    config.earth_rotation_rad_per_s = 0.0;
    config.vehicle = make_test_vehicle();
    // Thrust along body-X so the 2.5-DOF engine direction equals the body axis
    // (no gimbal offset) and no pitch torque develops; keep attitude steady so
    // every 2.5-DOF sample carries the seeded pitch.
    config.vehicle.stages.front().engine.direction_body = {1.0, 0.0, 0.0};
    config.vehicle.engine.direction_body = {1.0, 0.0, 0.0};
    config.vehicle.rigid_body.initial_attitude_rad = 0.0;
    config.vehicle.rigid_body.initial_angular_velocity_radps = 0.0;

    post2::core::PhaseConfig p0;
    p0.name = "3dof steer out of plane";
    p0.dynamics_dof = "3dof";
    p0.termination = {"time", ">=", 0.3};
    p0.inherit_initial_state = false;
    p0.optimize_enabled = false;
    p0.integrator = "dopri5";
    p0.initial_state_eci = post2::core::State{
        {post2::core::kEarthRadiusM + 1000.0, 0.0, 0.0},
        {0.0, 100.0, 0.0},
    };
    p0.force_models.gravity = false;
    p0.force_models.thrust = true;
    p0.force_models.normal_force = false;
    p0.force_models.aerodynamic = false;
    p0.force_models.atmosphere_model.type = "none";
    p0.throttle_model.type = "poly";
    p0.throttle_model.c0 = 0.0;  // logged orientation only; no acceleration
    p0.steering_model.type = "generic_poly";
    p0.steering_model.azimuth_deg.c0 = kTransitionAzimuthDeg;
    p0.steering_model.elevation_deg.c0 = kTransitionElevationDeg;

    post2::core::PhaseConfig p1 = p0;
    p1.name = "2.5dof inherit";
    p1.dynamics_dof = "2.5dof";
    p1.termination = {"time", ">=", 0.3};
    p1.inherit_initial_state = true;
    p1.initial_state_eci.reset();
    p1.throttle_model.c0 = 1.0;  // fire so the projected engine direction is logged
    p1.steering_model = post2::core::SteeringModelConfig{};  // 2.5-DOF uses rigid body

    config.phases = {p0, p1};
    return config;
}

int test_three_dof_to_two_point_five_dof_projects_orientation()
{
    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(make_transition_case());
    if (!result.ok) {
        std::cerr << "transition case failed to simulate: " << result.error << '\n';
        return 1;
    }

    // Seeded pitch = atan2 of the in-plane (east, up) heading components of the
    // inherited 3-D thrust direction. Out-of-plane (north / +z) part is dropped.
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double az = kTransitionAzimuthDeg * kDegToRad;
    const double el = kTransitionElevationDeg * kDegToRad;
    const double expected_attitude_rad =
        std::atan2(std::cos(el) * std::sin(az), std::sin(el));

    double last_three_dof_dir_z = 0.0;
    bool saw_three_dof = false;
    bool saw_two_point_five_dof = false;
    double max_two_point_five_dir_z = 0.0;
    double max_attitude_error = 0.0;
    for (const auto& entry : result.state_log.entries()) {
        if (entry.phase_index == 0) {
            saw_three_dof = true;
            last_three_dof_dir_z = entry.engine_direction_eci.z;
        } else if (entry.phase_index == 1) {
            saw_two_point_five_dof = true;
            max_attitude_error = std::max(
                max_attitude_error,
                std::abs(entry.rigid_body_attitude_rad - expected_attitude_rad));
            if (entry.engine_thrust_n > 0.0) {
                max_two_point_five_dir_z =
                    std::max(max_two_point_five_dir_z, std::abs(entry.engine_direction_eci.z));
            }
        }
    }

    if (!saw_three_dof || !saw_two_point_five_dof) {
        std::cerr << "transition case did not produce both phases\n";
        return 1;
    }
    // The 3-DOF orientation must genuinely leave the plane (otherwise the
    // projection is vacuous).
    if (std::abs(last_three_dof_dir_z) < 0.5) {
        std::cerr << "3-DOF orientation was not out of plane: z="
                  << last_three_dof_dir_z << '\n';
        return 1;
    }
    // After the switch the orientation must lie in the plane and the seeded
    // pitch must match the projected heading (not the default zero attitude).
    if (max_two_point_five_dir_z > 1.0e-9 ||
        max_attitude_error > 1.0e-2 ||
        std::abs(expected_attitude_rad) < 0.1) {
        std::cerr << "3-DOF -> 2.5-DOF did not project orientation into the plane:"
                  << " max_2p5_dir_z=" << max_two_point_five_dir_z
                  << " expected_attitude=" << expected_attitude_rad
                  << " max_attitude_error=" << max_attitude_error << '\n';
        return 1;
    }
    return 0;
}

// Off-equator coast under J2, where the gravity vector deviates from the pure
// radial direction. The 2.5-DOF plane must be spanned by velocity and gravity,
// so gravity lies in the plane while the radial (position) direction does not.
post2::core::CaseConfig make_j2_offequator_case()
{
    post2::core::CaseConfig config;
    config.name = "2.5dof j2 plane";
    config.step_s = 0.1;
    config.earth_rotation_rad_per_s = 0.0;
    config.vehicle = make_test_vehicle();

    const double r = post2::core::kEarthRadiusM + 300000.0;
    const double c = std::sqrt(0.5);  // 45 deg latitude

    post2::core::PhaseConfig phase;
    phase.name = "2.5dof coast under j2";
    phase.dynamics_dof = "2.5dof";
    phase.termination = {"time", ">=", 1.0};
    phase.inherit_initial_state = false;
    phase.optimize_enabled = false;
    phase.integrator = "dopri5";
    phase.initial_state_eci = post2::core::State{
        {r * c, 0.0, r * c},
        {0.0, 5000.0, 0.0},
    };
    phase.force_models.gravity = true;
    phase.force_models.gravity_model.type = "j2";
    phase.force_models.thrust = false;
    phase.force_models.normal_force = false;
    phase.force_models.aerodynamic = false;
    phase.force_models.atmosphere_model.type = "none";

    config.phases = {phase};
    return config;
}

int test_two_point_five_dof_plane_follows_gravity_under_j2()
{
    post2::core::LocalTrajectoryService service;
    const post2::core::CaseConfig case_config = make_j2_offequator_case();
    const auto result = service.simulate(case_config);
    if (!result.ok) {
        std::cerr << "j2 plane case failed to simulate: " << result.error << '\n';
        return 1;
    }
    if (result.state_log.size() < 3) {
        std::cerr << "j2 plane case produced too few samples\n";
        return 1;
    }

    auto cross = [](const post2::vehicle::Vec3& a, const post2::vehicle::Vec3& b) {
        return post2::vehicle::Vec3{
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
        };
    };
    auto unit = [](const post2::vehicle::Vec3& v) {
        const double n = post2::vehicle::norm(v);
        return n > 0.0 ? post2::vehicle::Vec3{v.x / n, v.y / n, v.z / n} : v;
    };

    // The 2.5-DOF plane is fixed: any logged (position, velocity) pair lies in
    // it, so their cross product recovers the plane normal.
    const auto& sample = result.state_log.entries().back();
    const post2::vehicle::Vec3 normal =
        unit(cross(sample.state.position_m, sample.state.velocity_mps));

    post2::core::SimulationConfig grav_config;
    grav_config.gravity_model.type = "j2";
    grav_config.earth_mu_m3s2 = case_config.earth_mu_m3s2;
    grav_config.earth_radius_m = case_config.earth_radius_m;
    grav_config.earth_j2 = case_config.earth_j2;
    grav_config.gravity_model.j2 = case_config.earth_j2;
    const post2::core::State initial = *case_config.phases.front().initial_state_eci;
    const post2::vehicle::Vec3 gravity0 = unit(
        post2::propagation::gravity_acceleration_mps2(grav_config, initial.position_m));
    const post2::vehicle::Vec3 radial0 = unit(initial.position_m);

    const double gravity_out_of_plane = std::abs(post2::vehicle::dot(gravity0, normal));
    const double radial_out_of_plane = std::abs(post2::vehicle::dot(radial0, normal));

    // Gravity lies in the plane; the radial direction does not (J2 tilts gravity
    // off-radial, so a position-spanned plane would put gravity out of plane).
    if (gravity_out_of_plane > 1.0e-6 || radial_out_of_plane < 1.0e-4) {
        std::cerr << "2.5dof plane is not spanned by velocity and gravity:"
                  << " gravity_out_of_plane=" << gravity_out_of_plane
                  << " radial_out_of_plane=" << radial_out_of_plane << '\n';
        return 1;
    }
    return 0;
}

// A booster reentering engine-first is a bluff body (alpha ~180 deg) whose drag
// the slender-body model misses. The base-first descent drag (AeroConfig
// descent_cd) only engages if the angle of attack is taken from the commanded
// attitude while coasting -- this exercises both the attitude fix and the
// descent-drag override.
post2::core::CaseConfig make_descent_drag_case(const std::string& steering, double descent_cd)
{
    post2::core::CaseConfig config;
    config.name = "descent drag";
    config.step_s = 0.1;
    config.earth_rotation_rad_per_s = 0.0;

    post2::vehicle::VehicleConfig v;
    v.name = "drag body";
    post2::vehicle::StageConfig s;
    s.name = "s";
    s.active = true;
    s.attached = true;
    s.dry_mass_kg = 5000.0;
    s.engine.enabled = true;
    s.engine.thrust_vac_n = 100000.0;
    s.engine.isp_vac_s = 300.0;
    s.engine.engine_count = 1;
    s.engine.direction_body = {1.0, 0.0, 0.0};
    post2::vehicle::TankConfig t;
    t.name = "main";
    t.propellant = "generic";
    t.capacity_kg = 1000.0;
    t.initial_kg = 1000.0;
    s.tanks = {t};
    s.engine.feed_tanks = {{"s", "main"}};
    v.stages = {s};
    v.dry_mass_kg = 5000.0;
    v.tanks = s.tanks;
    v.engine = s.engine;
    v.aero.enabled = true;
    v.aero.use_table = false;
    v.aero.cd = 0.5;
    v.aero.reference_area_m2 = 10.0;
    v.aero.descent_cd = descent_cd;
    config.vehicle = v;

    post2::core::PhaseConfig p;
    p.name = "coast";
    p.dynamics_dof = "3dof";
    p.termination = {"time", ">=", 5.0};
    p.inherit_initial_state = false;
    p.optimize_enabled = false;
    p.integrator = "dopri5";
    p.initial_state_eci = post2::core::State{
        {post2::core::kEarthRadiusM + 25000.0, 0.0, 0.0},
        {0.0, 800.0, 0.0},
    };
    p.force_models.gravity = true;
    p.force_models.gravity_model.type = "point_mass";
    p.force_models.thrust = true;  // enabled so the steering updates the attitude
    p.force_models.aerodynamic = true;
    p.force_models.atmosphere_model.type = "exponential";
    p.throttle_model.type = "poly";
    p.throttle_model.c0 = 0.0;     // ... but zero throttle: no actual thrust
    p.steering_model.type = steering;
    config.phases = {p};
    return config;
}

int test_base_first_descent_drag()
{
    post2::core::LocalTrajectoryService service;
    const auto with_drag = service.simulate(make_descent_drag_case("retrograde", 2.0));
    const auto without_drag = service.simulate(make_descent_drag_case("retrograde", 0.0));
    if (!with_drag.ok || !without_drag.ok) {
        std::cerr << "descent drag case failed to simulate: with='" << with_drag.error
                  << "' without='" << without_drag.error << "'\n";
        return 1;
    }
    const double v_drag = with_drag.state_log.entries().back().speed_mps;
    const double v_nodrag = without_drag.state_log.entries().back().speed_mps;
    // Engaging the base-first descent drag (alpha ~180 from the retrograde
    // attitude) must decelerate the body substantially more than the slender
    // constant-Cd case. If alpha were stuck at 0 during the coast, descent_cd
    // would never engage and the two would match.
    if (!(v_drag < v_nodrag - 50.0)) {
        std::cerr << "base-first descent drag did not engage: v_drag=" << v_drag
                  << " v_nodrag=" << v_nodrag << '\n';
        return 1;
    }
    return 0;
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
    failures += test_three_dof_to_two_point_five_dof_projects_orientation();
    failures += test_two_point_five_dof_plane_follows_gravity_under_j2();
    failures += test_base_first_descent_drag();
    if (failures != 0) {
        std::cerr << "dynamics_dof_test: " << failures << " case(s) failed\n";
        return 1;
    }
    return 0;
}
