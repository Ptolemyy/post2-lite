// UPFG (Unified Powered Flight Guidance) correctness tests.
//
//   1. Conic state extrapolation (universal-variable Kepler) -- the gravity
//      propagator UPFG relies on.
//   2. Target setup from an apsis/inclination orbit spec.
//   3. UPFG convergence at a representative insertion state (sane thrust vector).
//   4. End-to-end: a single closed-loop UPFG phase lifts a sub-orbital state to
//      the configured ~200 km orbit.

#include "post2/core/optimization.hpp"
#include "post2/core/trajectory_service.hpp"
#include "post2/core/types.hpp"
#include "post2/core/upfg.hpp"
#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

using post2::vehicle::Vec3;
using post2::vehicle::dot;
using post2::vehicle::norm;

Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

bool fail(const std::string& message)
{
    std::cerr << message << '\n';
    return false;
}

constexpr double kMu = post2::core::kEarthMuM3S2;
constexpr double kRe = post2::core::kEarthRadiusM;

// Periapsis/apoapsis altitude (m) of the osculating orbit of (r, v).
void apsides_altitude_m(const Vec3& r, const Vec3& v, double* peri_alt, double* apo_alt)
{
    const double rmag = norm(r);
    const double vmag2 = dot(v, v);
    const double energy = 0.5 * vmag2 - kMu / rmag;
    const double sma = -kMu / (2.0 * energy);
    const Vec3 ev = r * ((vmag2 - kMu / rmag) / kMu) - v * (dot(r, v) / kMu);
    const double ecc = norm(ev);
    *peri_alt = sma * (1.0 - ecc) - kRe;
    *apo_alt = sma * (1.0 + ecc) - kRe;
}

bool test_conic_propagator()
{
    const double rc = kRe + 300000.0;
    const double vc = std::sqrt(kMu / rc);
    const Vec3 r0{rc, 0.0, 0.0};
    const Vec3 v0{0.0, vc, 0.0};

    const double period = 2.0 * 3.14159265358979323846 * std::sqrt(rc * rc * rc / kMu);

    // A quarter orbit (prograde, +x->+y) must rotate the state by +90 degrees.
    const auto quarter = post2::core::conic_state_extrapolate(r0, v0, period / 4.0, kMu);
    if (!quarter.ok) {
        return fail("conic: quarter-orbit propagation failed");
    }
    if (std::abs(quarter.position_m.x) > 1.0 || std::abs(quarter.position_m.y - rc) > 1.0 ||
        std::abs(quarter.velocity_mps.x + vc) > 1.0e-3 || std::abs(quarter.velocity_mps.y) > 1.0e-3) {
        return fail("conic: quarter orbit did not land at (0, rc) moving -x");
    }
    if (std::abs(norm(quarter.position_m) - rc) > 1.0e-3) {
        return fail("conic: circular radius not preserved");
    }

    // Round trip: propagate forward then back returns the original state.
    const auto fwd = post2::core::conic_state_extrapolate(r0, v0, 1234.5, kMu);
    const auto back = post2::core::conic_state_extrapolate(fwd.position_m, fwd.velocity_mps, -1234.5, kMu);
    if (!fwd.ok || !back.ok || norm(back.position_m - r0) > 1.0e-3 ||
        norm(back.velocity_mps - v0) > 1.0e-6) {
        return fail("conic: forward/back round trip did not return to start");
    }

    // Energy is conserved on an eccentric arc.
    const Vec3 re{kRe + 500000.0, 0.0, 0.0};
    const Vec3 ve{1000.0, 7000.0, 500.0};
    const auto ecc = post2::core::conic_state_extrapolate(re, ve, 900.0, kMu);
    const double e0 = 0.5 * dot(ve, ve) - kMu / norm(re);
    const double e1 = 0.5 * dot(ecc.velocity_mps, ecc.velocity_mps) - kMu / norm(ecc.position_m);
    if (!ecc.ok || std::abs(e1 - e0) / std::abs(e0) > 1.0e-9) {
        return fail("conic: specific energy not conserved on eccentric arc");
    }
    return true;
}

bool test_target_setup()
{
    const Vec3 r{kRe + 150000.0, 0.0, 0.0};
    const Vec3 v{0.0, 7000.0, 0.0};
    const auto target = post2::core::make_upfg_orbit_target(200000.0, 200000.0, 0.0, r, v, kMu, kRe);

    const double expected_radius = kRe + 200000.0;
    const double expected_speed = std::sqrt(kMu / expected_radius);
    if (std::abs(target.radius_m - expected_radius) > 1.0e-6) {
        return fail("target: insertion radius wrong");
    }
    if (std::abs(target.velocity_mps - expected_speed) > 1.0) {
        return fail("target: circular insertion speed wrong");
    }
    if (std::abs(target.flight_path_angle_rad) > 1.0e-12) {
        return fail("target: circular insertion should have zero flight-path angle");
    }
    if (std::abs(norm(target.plane_normal) - 1.0) > 1.0e-9) {
        return fail("target: plane normal not unit length");
    }
    if (std::abs(dot(target.plane_normal, r)) > 1.0e-6 * norm(r)) {
        return fail("target: plane normal not perpendicular to position");
    }
    // Equatorial (inc 0): orbit normal is +z, so the UPFG iy is -z.
    if (std::abs(target.plane_normal.z + 1.0) > 1.0e-6) {
        return fail("target: equatorial plane normal should be -z");
    }
    return true;
}

bool test_upfg_convergence()
{
    // A representative upper-stage insertion state (equatorial, climbing east).
    post2::core::UpfgVehicleState state;
    state.time_s = 0.0;
    state.mass_kg = 60000.0;
    state.position_m = {kRe + 150000.0, 0.0, 0.0};
    state.velocity_mps = {150.0, 7300.0, 0.0};

    const auto target =
        post2::core::make_upfg_orbit_target(200000.0, 200000.0, 0.0, state.position_m, state.velocity_mps, kMu, kRe);

    post2::core::UpfgStage stage;
    stage.mode = 1;
    stage.thrust_n = 1.2e6;
    stage.exhaust_velocity_mps = 340.0 * 9.80665;
    stage.mdot_kgps = stage.thrust_n / stage.exhaust_velocity_mps;
    stage.mass_total_kg = state.mass_kg;
    stage.max_burn_time_s = 52000.0 / stage.mdot_kgps;

    // A single warm-started pass from a cold init must produce a sane command.
    const auto prev = post2::core::upfg_cold_init(target, state, kMu);
    const auto result = post2::core::upfg_step({stage}, target, state, prev, kMu);
    if (!result.ok) {
        return fail("upfg: single pass reported not-ok");
    }
    const Vec3 iF = result.thrust_unit_eci;
    if (!std::isfinite(iF.x) || !std::isfinite(iF.y) || !std::isfinite(iF.z) ||
        std::abs(norm(iF) - 1.0) > 1.0e-6) {
        return fail("upfg: thrust direction not a finite unit vector");
    }
    if (!std::isfinite(result.tgo_s) || result.tgo_s <= 0.0 || result.tgo_s > stage.max_burn_time_s) {
        return fail("upfg: time-to-go not in a sane range");
    }
    // The command should be mostly downrange (prograde) and nearly in-plane.
    const Vec3 orbit_normal = target.plane_normal * -1.0;
    const Vec3 prograde = cross(orbit_normal, state.position_m);
    const Vec3 prograde_hat = prograde / norm(prograde);
    if (dot(iF, prograde_hat) < 0.5) {
        return fail("upfg: thrust direction is not predominantly prograde");
    }
    if (std::abs(dot(iF, target.plane_normal)) > 0.2) {
        return fail("upfg: thrust direction has a large out-of-plane component");
    }
    return true;
}

bool test_upfg_reaches_orbit()
{
    // Single upper stage, generous margin, engaged from a realistic upper-stage
    // ignition state (exo-atmospheric, ~3 km/s, climbing -- a large velocity
    // deficit, the regime UPFG is built for). UPFG must loft it and insert near
    // the configured ~200 km orbit.
    post2::vehicle::VehicleConfig vehicle;
    vehicle.name = "upfg-upper";
    post2::vehicle::StageConfig stage;
    stage.name = "upper";
    stage.active = true;
    stage.attached = true;
    stage.dry_mass_kg = 10000.0;
    stage.engine.enabled = true;
    stage.engine.thrust_vac_n = 1.2e6;
    stage.engine.isp_vac_s = 340.0;
    stage.engine.engine_count = 1;
    stage.engine.feed_tanks = {{"upper", "main"}};
    stage.tanks = {{"main", "lox", 62000.0, 62000.0}};
    vehicle.stages = {stage};
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&vehicle);

    post2::core::CaseConfig case_config;
    case_config.name = "upfg-insertion";
    case_config.earth_radius_m = kRe;
    case_config.earth_mu_m3s2 = kMu;
    case_config.earth_rotation_rad_per_s = 0.0;  // pure inertial frame for the test
    case_config.step_s = 0.5;
    case_config.vehicle = vehicle;

    const double circular_speed = std::sqrt(kMu / (kRe + 200000.0));

    post2::core::PhaseConfig phase;
    phase.name = "upfg insert";
    phase.inherit_initial_state = false;
    phase.initial_state_eci = post2::core::State{{kRe + 100000.0, 0.0, 0.0}, {200.0, 3000.0, 0.0}};
    // Insertion is complete when the (rising) periapsis reaches near the target:
    // UPFG raises periapsis sharply in the final seconds of the burn.
    phase.termination.type = "periapsis_altitude_m";
    phase.termination.comparison = ">=";
    phase.termination.value = 150000.0;
    phase.integrator = "dopri5";
    phase.tolerances.rtol = 1.0e-7;
    phase.force_models.gravity = true;
    phase.force_models.thrust = true;
    phase.force_models.normal_force = false;
    phase.force_models.aerodynamic = false;
    phase.force_models.gravity_model.type = "point_mass";
    phase.throttle_model.type = "poly";
    phase.throttle_model.c0 = 1.0;
    phase.steering_model.type = "upfg";
    phase.steering_model.upfg.periapsis_km = 200.0;
    phase.steering_model.upfg.apoapsis_km = 200.0;
    phase.steering_model.upfg.inclination_deg = 0.0;
    post2::core::PhaseAction enable_engine;
    enable_engine.time_s = 0.0;
    enable_engine.type = "set_engine_enabled";
    enable_engine.value = true;
    phase.actions.push_back(enable_engine);
    case_config.phases = {phase};

    // Sanity: the starting state really is sub-orbital (periapsis below surface).
    double start_peri = 0.0, start_apo = 0.0;
    apsides_altitude_m(phase.initial_state_eci->position_m, phase.initial_state_eci->velocity_mps,
                       &start_peri, &start_apo);
    if (start_peri > 0.0) {
        return fail("upfg-orbit: test setup is not sub-orbital");
    }

    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(case_config);
    if (!result.ok) {
        return fail("upfg-orbit: simulation failed: " + result.error);
    }
    if (result.state_log.size() < 20) {
        return fail("upfg-orbit: phase terminated almost immediately");
    }

    const auto& last = result.state_log.back();
    double peri_alt = 0.0, apo_alt = 0.0;
    apsides_altitude_m(last.state.position_m, last.state.velocity_mps, &peri_alt, &apo_alt);

    // Periapsis lifted from far below the surface to a real orbit near the target.
    // Tolerances are loose: this simplified UPFG circularises to within a few tens
    // of km, which is what matters for a closed-loop "reaches orbit" check.
    if (peri_alt < 145000.0 || peri_alt > 250000.0) {
        std::cerr << "upfg-orbit: periapsis altitude " << peri_alt << " m outside [145,250] km\n";
        return false;
    }
    if (apo_alt < 180000.0 || apo_alt > 320000.0) {
        std::cerr << "upfg-orbit: apoapsis altitude " << apo_alt << " m outside [180,320] km\n";
        return false;
    }
    // Climbed substantially from the 100 km ignition toward the 200 km target.
    if (last.altitude_m < 170000.0) {
        std::cerr << "upfg-orbit: did not loft, final altitude " << last.altitude_m << " m\n";
        return false;
    }
    const double final_speed = norm(last.state.velocity_mps);
    if (final_speed < circular_speed - 50.0 || final_speed > circular_speed + 250.0) {
        std::cerr << "upfg-orbit: final speed " << final_speed << " not near circular "
                  << circular_speed << '\n';
        return false;
    }
    return true;
}

} // namespace

int main()
{
    if (!test_conic_propagator()) {
        return 1;
    }
    if (!test_target_setup()) {
        return 1;
    }
    if (!test_upfg_convergence()) {
        return 1;
    }
    if (!test_upfg_reaches_orbit()) {
        return 1;
    }
    return 0;
}
