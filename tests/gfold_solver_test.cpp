#include "post2/core/gfold_solver.hpp"
#include "post2/core/trajectory_service.hpp"
#include "post2/core/types.hpp"
#include "post2/vehicle/vehicle.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

post2::core::GfoldProblem make_landing_problem()
{
    post2::core::GfoldProblem p;
    p.g0 = 9.80665;
    p.m_wet = 30000.0;                 // booster + residual propellant [kg]
    p.m_dry = 22000.0;                 // dry floor [kg] -> 8 t usable
    p.exhaust_velocity_mps = 282.0 * 9.80665;  // Merlin-ish sea-level Isp
    p.t_max_n = 845000.0;              // one sea-level engine
    p.min_throttle = 0.4;
    p.max_throttle = 1.0;
    p.max_tilt_deg = 45.0;
    p.glide_slope_deg = 5.0;
    p.r0x = 1000.0;                    // downrange [m]
    p.r0y = 2000.0;                    // altitude [m]
    p.v0x = -30.0;
    p.v0y = -80.0;
    p.free_landing = true;
    return p;
}

int test_optimal_landing_is_feasible_and_soft()
{
    const post2::core::GfoldProblem p = make_landing_problem();
    const post2::core::GfoldSolution sol =
        post2::core::gfold_solve_optimal(p, 40, 5.0, 120.0);

    if (!sol.feasible || sol.nodes.empty()) {
        std::cerr << "gfold optimal landing was infeasible\n";
        return 1;
    }
    // Soft landing on the ground: terminal velocity ~ 0 and terminal altitude ~ 0.
    const auto& last = sol.nodes.back();
    if (std::abs(last.vx) > 1.0 || std::abs(last.vy) > 1.0) {
        std::cerr << "gfold terminal velocity not zero: vx=" << last.vx
                  << " vy=" << last.vy << '\n';
        return 1;
    }
    if (std::abs(last.ry) > 1.0) {
        std::cerr << "gfold did not land on the ground: ry=" << last.ry << '\n';
        return 1;
    }
    // Mass budget: final mass between dry floor and wet, fuel positive.
    if (sol.final_mass_kg < p.m_dry - 1.0 || sol.final_mass_kg > p.m_wet ||
        sol.fuel_used_kg <= 0.0) {
        std::cerr << "gfold mass budget invalid: final=" << sol.final_mass_kg
                  << " fuel=" << sol.fuel_used_kg << '\n';
        return 1;
    }
    // Throttle stays within the commanded band where firing.
    for (const auto& n : sol.nodes) {
        if (n.throttle > p.max_throttle + 0.05 || n.throttle < -0.05) {
            std::cerr << "gfold throttle out of band: " << n.throttle << '\n';
            return 1;
        }
    }
    return 0;
}

int test_golden_section_beats_coarse_sweep()
{
    const post2::core::GfoldProblem p = make_landing_problem();
    const post2::core::GfoldSolution best =
        post2::core::gfold_solve_optimal(p, 40, 5.0, 120.0);
    if (!best.feasible) {
        std::cerr << "gfold optimal infeasible in sweep test\n";
        return 1;
    }
    double coarse_min = std::numeric_limits<double>::infinity();
    for (double tf : {20.0, 30.0, 40.0, 50.0, 60.0, 80.0}) {
        const auto r = post2::core::gfold_solve_fixed_tf(p, tf, 40);
        if (r.feasible) {
            coarse_min = std::min(coarse_min, r.fuel_used_kg);
        }
    }
    // The golden-section optimum should be no worse than any coarse sample
    // (allowing a small slack for discretization differences).
    if (best.fuel_used_kg > coarse_min + 1.0) {
        std::cerr << "gfold optimal fuel " << best.fuel_used_kg
                  << " worse than coarse min " << coarse_min << '\n';
        return 1;
    }
    return 0;
}

post2::vehicle::VehicleConfig make_landing_vehicle()
{
    post2::vehicle::VehicleConfig vehicle;
    vehicle.name = "gfold landing booster";

    post2::vehicle::StageConfig stage;
    stage.name = "booster";
    stage.active = true;
    stage.attached = true;
    stage.dry_mass_kg = 10000.0;
    stage.engine.enabled = true;
    stage.engine.thrust_vac_n = 900000.0;   // one engine
    stage.engine.isp_vac_s = 300.0;
    stage.engine.engine_count = 1;
    stage.engine.direction_body = {1.0, 0.0, 0.0};

    post2::vehicle::TankConfig tank;
    tank.name = "main";
    tank.propellant = "generic";
    tank.capacity_kg = 8000.0;
    tank.initial_kg = 8000.0;
    stage.tanks = {tank};
    stage.engine.feed_tanks = {{"booster", "main"}};

    vehicle.stages = {stage};
    vehicle.dry_mass_kg = stage.dry_mass_kg;
    vehicle.tanks = stage.tanks;
    vehicle.engine = stage.engine;
    return vehicle;
}

post2::core::CaseConfig make_gfold_landing_case()
{
    post2::core::CaseConfig config;
    config.name = "gfold landing playback";
    config.step_s = 0.1;
    config.earth_rotation_rad_per_s = 0.0;
    config.vehicle = make_landing_vehicle();

    post2::core::PhaseConfig phase;
    phase.name = "gfold powered landing";
    phase.dynamics_dof = "2.5dof";
    phase.termination = {"time", ">=", 600.0};  // ignored: gfold owns its cutoff
    phase.inherit_initial_state = false;
    phase.optimize_enabled = false;
    phase.integrator = "rk4";
    phase.initial_state_eci = post2::core::State{
        {post2::core::kEarthRadiusM + 3000.0, 0.0, 0.0},  // 3 km altitude
        {-100.0, 50.0, 0.0},                              // descending + downrange
    };
    phase.force_models.gravity = true;
    phase.force_models.gravity_model.type = "point_mass";
    phase.force_models.thrust = true;
    phase.force_models.normal_force = false;
    phase.force_models.aerodynamic = false;
    phase.force_models.atmosphere_model.type = "none";
    phase.steering_model.type = "gfold";
    phase.steering_model.gfold.engine_count = 1;
    phase.steering_model.gfold.min_throttle = 0.2;
    phase.steering_model.gfold.max_throttle = 1.0;
    phase.steering_model.gfold.num_nodes = 40;
    phase.steering_model.gfold.tf_min_s = 2.0;
    phase.steering_model.gfold.tf_max_s = 80.0;
    phase.steering_model.gfold.free_landing = true;

    config.phases = {phase};
    return config;
}

int test_gfold_phase_lands_in_driver()
{
    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(make_gfold_landing_case());
    if (!result.ok) {
        std::cerr << "gfold landing case failed: " << result.error << '\n';
        return 1;
    }
    if (result.state_log.size() < 5) {
        std::cerr << "gfold landing produced too few samples\n";
        return 1;
    }
    const auto& last = result.state_log.entries().back();
    // Touchdown: near the ground and essentially at rest.
    if (last.altitude_m > 50.0 || last.speed_mps > 2.0) {
        std::cerr << "gfold landing did not touch down softly: alt=" << last.altitude_m
                  << " speed=" << last.speed_mps << '\n';
        return 1;
    }
    // The descent must actually start high and end low (a real landing arc).
    if (result.state_log.entries().front().altitude_m < 2000.0) {
        std::cerr << "gfold landing did not start from the handoff altitude\n";
        return 1;
    }
    // The thrust direction should be logged (engine firing during descent).
    bool saw_thrust = false;
    for (const auto& e : result.state_log.entries()) {
        if (e.engine_thrust_n > 0.0) {
            saw_thrust = true;
            break;
        }
    }
    if (!saw_thrust) {
        std::cerr << "gfold landing recorded no thrust\n";
        return 1;
    }
    return 0;
}

}  // namespace

int main()
{
    if (!post2::core::gfold_available()) {
        std::cout << "gfold_solver_test: Clarabel not compiled in; skipping\n";
        return 0;
    }
    int failures = 0;
    failures += test_optimal_landing_is_feasible_and_soft();
    failures += test_golden_section_beats_coarse_sweep();
    failures += test_gfold_phase_lands_in_driver();
    if (failures != 0) {
        std::cerr << "gfold_solver_test: " << failures << " case(s) failed\n";
        return 1;
    }
    return 0;
}
