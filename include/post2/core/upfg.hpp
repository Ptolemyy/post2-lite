#pragma once

#include <vector>

#include "post2/vehicle/vehicle.hpp"

namespace post2::core {

using Vec3 = post2::vehicle::Vec3;

// =============================================================================
// NASA-standard Unified Powered Flight Guidance (UPFG)
//
// Closed-loop exo-atmospheric ascent/insertion guidance, after Jaggers' "An
// Explicit Solution to the Exponential-Acceleration Powered-Flight Guidance"
// (the Space Shuttle "Unified Powered Flight Guidance", NASA/Draper). This is
// the same algorithm the kOS PEGAS project implements; the thrust integrals and
// the steering sequence here follow that formulation exactly.
//
// In post2-lite it backs the open-loop polynomial steering: select it per phase
// with SteeringModelConfig.type == "upfg". It is meant for vacuum/upper-stage
// phases only (it ignores drag and assumes the thrust vector can point freely)
// and assumes full vacuum thrust per stage (pair it with a full-throttle phase).
// =============================================================================

// Two-body (conic) state extrapolation: propagate (r0, v0) forward by dt under a
// point-mass field of gravitational parameter mu, returning the conic state.
// Universal-variable / Stumpff formulation -- the "conic state extrapolation"
// UPFG uses to evaluate the gravity contribution accumulated over a burn.
struct ConicState {
    Vec3 position_m;
    Vec3 velocity_mps;
    bool ok = false;
};
ConicState conic_state_extrapolate(const Vec3& r0, const Vec3& v0, double dt_s, double mu_m3s2);

// One UPFG burn stage. mode 1 = constant thrust, mode 2 = constant acceleration
// (accel-limited / throttled). Built from the vehicle's remaining propulsive
// stages, in burn order (active stage first).
struct UpfgStage {
    int mode = 1;
    double thrust_n = 0.0;              // vacuum thrust of the stage engine cluster
    double mdot_kgps = 0.0;             // mass flow at full thrust
    double exhaust_velocity_mps = 0.0;  // ve = isp_vac * g0
    double mass_total_kg = 0.0;         // total vehicle mass at this stage's ignition
    double max_burn_time_s = 0.0;       // propellant / mdot
    double accel_limit_mps2 = 0.0;      // mode-2 acceleration limit (g-limit * g0)
};

// Desired cutoff (insertion) condition, in the inertial frame.
struct UpfgTarget {
    double radius_m = 0.0;               // |r| at cutoff
    double velocity_mps = 0.0;           // |v| at cutoff
    double flight_path_angle_rad = 0.0;  // angle of v above the local horizontal
    Vec3 plane_normal = {0.0, 0.0, 0.0}; // iy: the NEGATIVE of the orbit angular-
                                         // momentum direction (UPFG convention)
};

// Vehicle dynamic state handed to UPFG (inertial frame).
struct UpfgVehicleState {
    double time_s = 0.0;
    double mass_kg = 0.0;
    Vec3 position_m;
    Vec3 velocity_mps;
};

// UPFG state carried between guidance cycles (and refined across the inner
// predictor-corrector iterations of upfg_converge).
struct UpfgInternal {
    Vec3 rbias;
    Vec3 rd;
    Vec3 rgrav;
    double tgo_s = 0.0;
    double tb_s = 0.0;   // burn time already accumulated on the active stage
    double time_s = 0.0;
    Vec3 v_prev;
    Vec3 vgo;
    bool initialized = false;
};

struct UpfgResult {
    Vec3 thrust_unit_eci = {1.0, 0.0, 0.0};  // commanded thrust direction (iF)
    double tgo_s = 0.0;
    UpfgInternal next;  // refreshed internal state (for warm-starting / inspection)
    bool ok = false;
};

// Cold-start internal state (mirrors PEGAS setupUPFG): a downrange guess for the
// desired cutoff position rd and the velocity-to-be-gained vgo.
UpfgInternal upfg_cold_init(const UpfgTarget& target, const UpfgVehicleState& state, double mu_m3s2);

// One UPFG pass. Given the remaining stage stack, target, current state, and the
// previous internal state, returns the commanded thrust direction, time-to-go,
// and the refreshed internal state. If earlier stages already supply the full
// velocity-to-go it drops the trailing stage and re-runs (matching PEGAS). The
// steering turn from `lambda` is clamped so a transient unconverged cycle cannot
// command retrograde thrust; this also makes the cold-start fixed-state iteration
// in upfg_converge contractive.
UpfgResult upfg_step(
    const std::vector<UpfgStage>& stages,
    const UpfgTarget& target,
    const UpfgVehicleState& state,
    const UpfgInternal& previous,
    double mu_m3s2);

// Solve UPFG at the current state: iterate upfg_step from a cold init at a fixed
// state for a fixed number of passes. With the clamped steering turn the cycle
// map is contractive and settles in a few passes, yielding a deterministic,
// smooth function of state -- the form used to embed UPFG in an integrator (each
// derivative evaluation re-solves independently, so the command is reentrant and
// free of warm-state hazards).
UpfgResult upfg_converge(
    const std::vector<UpfgStage>& stages,
    const UpfgTarget& target,
    const UpfgVehicleState& state,
    double mu_m3s2,
    int iterations = 12);

// Build the UPFG target from an apsis/inclination orbit spec, evaluated at the
// current state. Insertion is at the target periapsis (flight-path angle 0); for
// periapsis == apoapsis it is a circular orbit. The plane normal is chosen to
// contain the current position at the requested inclination, oriented to match
// current motion (minimising the commanded yaw).
UpfgTarget make_upfg_orbit_target(
    double periapsis_alt_m,
    double apoapsis_alt_m,
    double inclination_deg,
    const Vec3& position_m,
    const Vec3& velocity_mps,
    double mu_m3s2,
    double body_radius_m);

} // namespace post2::core
