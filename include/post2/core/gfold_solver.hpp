#pragma once

#include <vector>

namespace post2::core {

// True when the Clarabel conic solver is compiled in (POST2_HAVE_CLARABEL).
// When false, every solve returns an infeasible GfoldSolution.
bool gfold_available();

// 2D fuel-optimal powered-descent (G-FOLD) problem, expressed in the planar
// coordinates of a 2.5-DOF vertical plane: x = downrange, y = up (altitude),
// gravity acting along -y. Lossless convexification of the soft-landing problem
// (Acikmese, Carson, Blackmore 2013); the only non-convex parameter is the
// time-of-flight tf, recovered by an outer golden-section search.
struct GfoldProblem {
    double g0 = 9.80665;                 // gravity magnitude [m/s^2], along -up
    double m_wet = 0.0;                  // initial mass [kg]
    double m_dry = 0.0;                  // dry / final-floor mass [kg]
    double exhaust_velocity_mps = 0.0;   // ve = Isp * g_e; mass flow per thrust = 1/ve
    double t_max_n = 0.0;                // max thrust [N] (engine_count * per-engine)
    double min_throttle = 0.2;           // rho1 = min_throttle * t_max_n
    double max_throttle = 1.0;           // rho2 = max_throttle * t_max_n
    double max_tilt_deg = 45.0;          // max thrust tilt from local vertical
    double glide_slope_deg = 5.0;        // min glide-slope angle above ground

    // Boundary conditions in plane coordinates (downrange, up).
    double r0x = 0.0, r0y = 0.0;         // initial position [m]
    double v0x = 0.0, v0y = 0.0;         // initial velocity [m/s]
    double rfx = 0.0, rfy = 0.0;         // target landing position [m] (when !free_landing)
    bool free_landing = true;            // free downrange touchdown; only zero terminal velocity
};

// One discretization node of a solved trajectory.
struct GfoldNode {
    double t_s = 0.0;
    double rx = 0.0, ry = 0.0;   // position [m] (downrange, up)
    double vx = 0.0, vy = 0.0;   // velocity [m/s]
    double mass_kg = 0.0;
    double ux = 0.0, uy = 0.0;   // thrust acceleration [m/s^2]
    double thrust_n = 0.0;       // ||T|| = mass * ||u||
    double throttle = 0.0;       // thrust_n / t_max_n
};

struct GfoldSolution {
    bool feasible = false;
    double tf_s = 0.0;
    double final_mass_kg = 0.0;
    double fuel_used_kg = 0.0;
    std::vector<GfoldNode> nodes;
};

// Solve the min-fuel SOCP for a fixed time-of-flight tf and node count.
GfoldSolution gfold_solve_fixed_tf(const GfoldProblem& problem, double tf_s, int num_nodes);

// Find the fuel-optimal landing: bisect the feasibility floor in tf, then
// golden-section the (unimodal) fuel(tf) curve. Returns the best feasible
// solution, or an infeasible one if no tf in [tf_min_s, tf_max_s] works.
GfoldSolution gfold_solve_optimal(
    const GfoldProblem& problem,
    int num_nodes,
    double tf_min_s,
    double tf_max_s);

}  // namespace post2::core
