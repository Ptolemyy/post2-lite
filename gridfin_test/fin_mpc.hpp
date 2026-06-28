// fin_mpc.hpp -- inner-loop per-fin control-allocation MPC (roll-suppressing).
//
// Pure grid-fin control, NO RCS. The four grid fins (90 deg apart around the
// roll axis) are the only actuators. kRPC exposes no per-control-surface angle
// setter -- the fins react to the vessel's aggregate Control.set_pitch/yaw/roll
// and KSP distributes those onto the fins -- so this allocator REASONS about the
// four fins explicitly (a 3xN mixing matrix M with the roll coupling), solves a
// short box-constrained QP for the per-fin deflections, then collapses them back
// into the three axis commands kRPC accepts.
//
// Why per-fin instead of three independent axis PIDs: this booster's roll moment
// of inertia is ~90x smaller than pitch/yaw, so the fin deflections needed to
// point in pitch/yaw pump roll through aero coupling and the roll runs away. The
// QP shares ONE per-fin deflection budget across all three axes and weights roll
// far above pitch/yaw, so it spends fin authority on killing roll FIRST and only
// uses what's left to point -- i.e. roll suppression is built into the allocation
// (the kOS/RCS approach is unavailable here, so the fins must do it themselves).
//
// Robustness (learned from live runs): the per-axis effectiveness sign AND
// magnitude are identified online from (axis command -> axis angular accel),
// gated on dynamic pressure, so a wrong roll sign cannot lock in positive
// feedback. Until identified a conservative default is used with a low roll gain.
//
// Pure math (no kRPC types): usable in the offline --dry-run sim.

#pragma once

#include "glide_math.hpp"

#include <vector>

namespace gridfin {

struct FinMpcConfig {
    int    nfins      = 4;
    double ring_phase = 0.0;     // angle of fin 0 (rad); 0 = '+', pi/4 = 'x'
    double fin_limit  = 1.0;     // |deflection| per fin (normalised units)

    double dt         = 0.05;    // control period (s)
    double k_track    = 2.0;     // desired-accel gain: g_a = k_track*(rate* - rate)
                                 // (closed loop is omega_dot ~= -k_track*omega,
                                 //  independent of b; keep k_track*dt well < 1)
    double w_pitch    = 1.0;
    double w_yaw      = 1.0;
    double w_roll     = 8.0;     // >> pitch/yaw: SUPPRESS roll first

    // Roll priority: when the roll rate is large, throttle the pitch/yaw demand
    // toward zero so the fins are NOT pulled toward pointing (whose deflections
    // re-inject roll through coupling) -- they fully kill roll first, then steer.
    double roll_prio_rate = 0.20;  // |roll rate| (rad/s) at which throttling starts
    double roll_prio_k    = 2.5;   // throttle slope per (rad/s) of excess
    double reg        = 0.02;    // per-fin effort regularisation
    int    pgd_iters  = 150;     // projected-gradient iterations
    double pgd_lr     = 0.30;    // projected-gradient step

    // online identification of per-axis effectiveness (axis cmd -> axis accel)
    double q_ident_pa = 1500.0;  // identify only above this dynamic pressure
    double b_floor    = 0.25;    // |b| never collapses below this
    double b_ceil     = 100.0;
    Vec3   b_default  = {1.2, 1.2, 3.0};   // (pitch,yaw,roll) magnitude seed

    // Startup roll-sign dither: once dynamic pressure is high enough, briefly
    // command a known common-mode (pure roll) deflection and watch the roll
    // response, so the roll control-sign is identified CLEANLY (no yaw-leak
    // corruption) and FAST -- before any steering -- then frozen. Without this the
    // online ID is too slow and a wrong initial sign diverges roll in ~1 s.
    double dither_amp    = 0.30;  // per-axis deflection amplitude (normalised)
    double dither_hold   = 0.40;  // seconds per +/- dither slot
    double dither_window = 3.6;   // total dither (s): 3 stages (roll,pitch,yaw) x 1.2 s

    // command shaping
    double slew_fin   = 1.00;    // max |delta deflection|/step per fin (actuator
                                 // rate model, normalised units). 1.0 = no
                                 // artificial lag (KSP surfaces are fast); LOWER
                                 // it only if a flight step test shows real lag --
                                 // too low adds phase lag and limit-cycles roll.
    double rate_abort = 8.0;     // |any body rate| over this -> abort flag (rad/s)
};

struct FinCommand {
    Vec3                axis = {0.0, 0.0, 0.0};   // [pitch,yaw,roll] in [-1,1]
    std::vector<double> fin_deflection;            // per-fin solution (logging)
    bool                abort = false;
    Vec3                dbg_g  = {0.0, 0.0, 0.0};  // desired accel (debug)
    Vec3                dbg_be = {0.0, 0.0, 0.0};  // b*sign used by QP (debug)
};

enum class FinMode { WaitQ, Dither, Active };

class FinMpc {
public:
    explicit FinMpc(const FinMpcConfig& cfg = {});

    void   set_layout(int nfins, double ring_phase);
    // Build the mixing from each fin's MEASURED azimuth around the roll axis
    // (rad), so the per-fin allocation matches the real geometry.
    void   set_layout_azimuths(const std::vector<double>& phi_rad);
    void   set_target_rate(const Vec3& pyr) { target_rate_ = pyr; }   // rad/s
    void   reset();

    // One step. omega = measured body rate [pitch,yaw,roll] (rad/s); q = dynamic
    // pressure (Pa). Returns the 3 axis commands + the per-fin deflections.
    FinCommand update(const Vec3& omega, double dt, double q);

    // Introspection.
    Vec3    b_id()      const { return b_; }
    Vec3    roll_sign() const { return sign_; }   // identified sign per axis
    FinMode mode()      const { return mode_; }
    int     nfins()     const { return cfg_.nfins; }
    const std::vector<std::vector<double>>& mixing() const { return M_; }

private:
    void   build_mixing();                 // from equally-spaced ring_phase
    void   build_mixing(const std::vector<double>& phi);   // from explicit azimuths
    std::vector<double> solve_qp(const Vec3& g) const;     // per-fin deflections
    void   identify(const Vec3& omega, double dt, double q);

    FinMpcConfig cfg_;
    std::vector<std::vector<double>> M_;   // 3 x nfins normalised mixing matrix
    std::vector<double> last_fin_;         // last per-fin deflection (slew + ID)

    Vec3   target_rate_ = {0.0, 0.0, 0.0};
    Vec3   b_   = {1.2, 1.2, 3.0};         // identified |effectiveness| per axis
    Vec3   sign_= {1.0, 1.0, 1.0};         // identified control sign per axis
    Vec3   last_axis_ = {0.0, 0.0, 0.0};   // last applied axis command (regressor)
    Vec3   prev_omega_= {0.0, 0.0, 0.0};
    bool   have_prev_ = false;
    Vec3   Suu_ = {0.0, 0.0, 0.0};         // decaying sum(u^2) per axis
    Vec3   Suy_ = {0.0, 0.0, 0.0};         // decaying sum(u*accel) per axis

    FinMode mode_ = FinMode::WaitQ;        // startup state machine
    double  dither_time_ = 0.0;            // time spent dithering (s)
    bool    sign_frozen_ = false;          // roll sign locked after the dither
};

} // namespace gridfin
