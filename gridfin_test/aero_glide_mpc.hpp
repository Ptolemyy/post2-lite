// aero_glide_mpc.hpp -- outer-loop sampling-based (CEM) 6-DOF MPC.
//
// The booster is UNPOWERED on the way down: the only way to steer is to tilt the
// body off retrograde by a bounded angle of attack so the airframe develops lift
// (the grid fins trim that angle of attack; the inner per-fin MPC realises it).
// This MPC plans the angle-of-attack steering over a receding horizon so the
// booster arrives over the target lat/lon AT the target altitude with:
//     * horizontal position error -> 0   (over the pad)
//     * horizontal velocity       -> 0   (descending straight down)
//     * nose attitude             -> vertical, settled (low body rate).
//
// 6-DOF forward model: the rollout propagates the RIGID-BODY state -- position,
// velocity, the actual nose direction and the body angular rate -- not a point
// mass. The aero force/moment use the ACTUAL (laggy) angle of attack, and the
// attitude tracks the per-node steering setpoint through a rate/acceleration-
// limited model of the inner loop + airframe (transverse MOI from kRPC, fin
// authority scaled with dynamic pressure). So the planned trajectory is
// DYNAMICALLY FEASIBLE -- it accounts for the time/authority it takes to rotate
// to the commanded attitude, which is what a point-mass model gets wrong near the
// terminal (high q, fast dynamics) and what limited terminal accuracy.
//
// The control is optimised by the cross-entropy method (CEM): many candidate
// angle-of-attack profiles are rolled out through the 6-DOF model, the elite
// (lowest terminal-cost) fraction defines the next sampling distribution, and the
// first node of the converged mean is emitted as the desired nose direction for
// the fast inner loop. Re-planned every outer step.
//
// Pure math (no kRPC types): runs unchanged in the offline --dry-run sim.

#pragma once

#include "glide_math.hpp"
#include "post2/aero/aero_table.hpp"

#include <random>
#include <vector>

namespace gridfin {

// Density / speed-of-sound vs altitude. A sampled profile (filled from kRPC at
// startup) with an exponential fallback/continuation above the top sample.
struct Atmosphere {
    std::vector<double> h;      // ascending altitudes (m, MSL)
    std::vector<double> rho;    // density at h (kg/m^3)
    double a_const  = 300.0;    // speed of sound (m/s); constant if no profile
    double rho0     = 1.225;    // sea-level density (exponential fallback)
    double scale_h  = 7000.0;   // density scale height (exponential fallback)

    double density(double alt) const;
    double sound_speed(double /*alt*/) const { return a_const; }
};

struct GlideEnv {
    double mu        = 3.5316e12;   // gravitational parameter (m^3/s^2)
    double r_body    = 600000.0;    // body radius (m)
    double mass      = 25000.0;     // booster mass (kg)
    double s_ref     = 10.52;       // aero reference area (m^2), matches table
    double max_aoa   = 0.18;        // angle-of-attack cap (rad, ~10 deg)
    double lift_sign = 1.0;         // sign of body lift vs the nose-tilt (-1 if
                                    // the real finned booster's lateral force
                                    // opposes the tilt; see main).
    const post2::aero::AeroTable* table = nullptr;
    const Atmosphere* atmo = nullptr;

    // ---- 6-DOF attitude dynamics (rigid cylinder + inner-loop tracking) ----
    double att_kp    = 1.0;         // attitude-error -> desired-rate gain (models
                                    // the inner loop's pointing cascade)
    double omega_max = 0.30;        // body-rate clamp (rad/s) the inner loop holds
    double accel_max = 0.60;        // max angular accel (rad/s^2) at q_ref --
                                    // available_torque/MOI; scales ~ q/q_ref
    double q_ref     = 30000.0;     // dynamic pressure accel_max is referenced to
};

struct MpcConfig {
    double dt_sim     = 0.22;   // forward-integration step (s)
    double t_max      = 45.0;   // horizon cap (s)
    int    n_nodes    = 6;      // piecewise-constant AoA control nodes (finer ->
                                // can represent "accelerate toward target then
                                // reverse to null the horizontal velocity")
    int    cem_samples= 96;     // candidate rollouts per CEM iteration
    int    cem_iters  = 6;      // CEM refinement iterations
    int    cem_elite  = 14;     // elite (kept) candidates per iteration
    double init_std   = 0.12;   // initial AoA sampling std-dev (rad)
    double std_floor  = 0.01;   // sampling std-dev floor (rad)

    // Terminal cost scales (cost contribution is (value/scale)^2).
    double pos_scale  = 90.0;   // m  -- horizontal miss tolerance (tighter)
    double vh_scale   = 12.0;   // m/s-- horizontal speed tolerance
    double alt_scale  = 300.0;  // m  -- "did not reach target alt" penalty
    double w_altmiss  = 1.0;    // weight on the not-reached altitude error
    double w_effort   = 0.4;    // running AoA effort penalty
    double w_smooth   = 0.6;    // node-to-node AoA change penalty
    double w_term_aoa = 1.5;    // terminal AoA -> 0 (nose vertical at arrival)
    double w_attvert  = 0.8;    // terminal nose-NOT-vertical penalty (6-DOF)
    double w_omega    = 0.5;    // terminal body-rate penalty (settled at arrival)

    double min_speed  = 25.0;   // below this just hold vertical (aero too weak)
};

struct MpcResult {
    Vec3   aim_hat   = {0.0, 1.0, 0.0};  // desired nose direction (body-fixed)
    Vec3   lift_dir  = {0.0, 0.0, 0.0};  // commanded lift direction (body-fixed)
    double aoa_rad   = 0.0;              // commanded angle of attack
    double pred_pos_err_m = 0.0;         // predicted terminal horizontal miss
    double pred_vh_mps    = 0.0;         // predicted terminal horizontal speed
    double pred_att_err_deg = 0.0;       // predicted terminal nose-from-vertical
    double pred_time_s    = 0.0;         // predicted time to reach target altitude
    bool   reached   = false;            // horizon reached the target altitude
    bool   holding   = false;            // low-speed: holding vertical, not steering
    double cost      = 0.0;              // best terminal cost
};

class AeroGlideMpc {
public:
    explicit AeroGlideMpc(const MpcConfig& cfg = {});

    // Plan from the current state to `target` (fixed point, body-fixed frame, at
    // the target altitude). `pos`/`vel` are body-fixed; `nose` is the CURRENT
    // actual nose direction (body-fixed); `omega` is the current body angular
    // rate (body-fixed, rad/s). The 6-DOF rollout starts from this real attitude
    // state so the plan accounts for where the vehicle is actually pointing.
    MpcResult plan(const Vec3& pos, const Vec3& vel, const Vec3& nose,
                   const Vec3& omega, const Vec3& target, const GlideEnv& env,
                   double target_alt);

    const MpcConfig& config() const { return cfg_; }

private:
    double rollout(const std::vector<double>& u, const Vec3& pos, const Vec3& vel,
                   const Vec3& nose0, const Vec3& omega0, const Vec3& target,
                   const GlideEnv& env, double target_alt,
                   const Vec3& h1, const Vec3& h2,
                   double* pos_err, double* vh, double* att_err, double* t_reach,
                   bool* reached) const;

    // Steering setpoint (desired nose direction) for control node 0 at the
    // current velocity -- this is what is emitted to the inner loop.
    Vec3 node0_aim(const std::vector<double>& mean, const Vec3& vel,
                   const Vec3& h1, const Vec3& h2, double max_aoa,
                   Vec3* lift_dir, double* aoa) const;

    MpcConfig cfg_;
    std::mt19937 rng_;
    std::vector<double> warm_;   // warm-started CEM mean (size 2*n_nodes)
    bool has_warm_ = false;
    double lift_sign_ = 1.0;     // cached from env for node0_aim
};

} // namespace gridfin
