#pragma once

#include <vector>

namespace post2::core {

// Inputs to the single-burn re-entry guidance solver. Everything is evaluated in
// the descent plane (atmosphere-relative). The solver sizes ONE retrograde burn
// so that future dynamic pressure and heat flux stay under their limits, using
// the least fuel. US1976 atmosphere is assumed internally.
struct EntryBurnProblem {
    double altitude_m = 0.0;             // current geometric altitude
    double speed_mps = 0.0;              // current atmosphere-relative speed (>0)
    double flight_path_angle_rad = 0.0;  // below local horizontal, >0 descending

    double mass_kg = 0.0;
    double max_thrust_n = 0.0;           // max retrograde thrust (e.g. 3 engines @100%)
    double min_thrust_n = 0.0;           // min thrust while firing (e.g. 1 engine @40%)
    double exhaust_velocity_mps = 3000.0;// ve = isp*g0, for the fuel objective

    double drag_cd = 0.8;                // representative descent Cd
    double ref_area_m2 = 0.0;
    double nose_radius_m = 0.5;          // stagnation nose radius for heat flux

    double q_limit_pa = 0.0;             // <=0 disables the dynamic-pressure limit
    double heat_flux_limit_wpm2 = 0.0;   // <=0 disables the heat-flux limit

    double earth_radius_m = 6371000.0;
    double earth_mu_m3s2 = 3.986004418e14;

    double dt_s = 2.0;                   // discretization step of the descent
    int max_nodes = 160;                 // horizon cap
    int scvx_iterations = 4;             // frozen-atmosphere re-linearisations
    double safety_margin = 0.03;         // size for limit*(1-margin)
};

// Result of solving the burn. ignition_delay_s/burn_duration_s describe the
// single contiguous window; thrust_n is the representative (peak) thrust to
// command in it. predicted_peak_* are the post-solve predicted loads.
struct EntryBurnSolution {
    bool ok = false;            // solver ran and converged
    bool burn_needed = false;   // a burn was required (ballistic peak exceeds a limit)
    double ignition_delay_s = 0.0;
    double burn_duration_s = 0.0;
    double thrust_n = 0.0;
    double predicted_peak_q_pa = 0.0;
    double predicted_peak_heat_flux_wpm2 = 0.0;
    std::vector<double> thrust_profile_n;  // per-node thrust over the horizon
};

// Solves the entry-burn guidance. Returns false (solution->ok=false) when
// Clarabel is not compiled in or the solve fails. When the ballistic descent
// already keeps loads under the limits, returns ok=true / burn_needed=false.
bool solve_entry_burn(const EntryBurnProblem& problem, EntryBurnSolution* solution);

// Whether this build linked the Clarabel conic solver (compiled with
// POST2_HAVE_CLARABEL, gated by the CMake option POST2_USE_CLARABEL). When
// false the Clarabel-based re-entry burn guidance is unavailable and callers
// must fall back.
bool clarabel_available();

// Solves a trivial reference convex QP through Clarabel to verify the whole
// link -> load -> solve chain is wired up:
//     minimize x^2   subject to   x >= 1     ->   x* = 1
// Returns true and writes x* to *x_out (if non-null) on success. Returns false
// when Clarabel is not compiled in (the *x_out is left untouched).
bool clarabel_selftest(double* x_out);

} // namespace post2::core
