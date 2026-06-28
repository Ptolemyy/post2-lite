// fin_mpc.cpp -- implementation of the per-fin roll-suppressing inner MPC.

#include "fin_mpc.hpp"

#include <algorithm>
#include <cmath>

namespace gridfin {

FinMpc::FinMpc(const FinMpcConfig& cfg) : cfg_(cfg) {
    b_ = cfg_.b_default;
    build_mixing();
}

void FinMpc::set_layout(int nfins, double ring_phase) {
    cfg_.nfins = std::max(1, nfins);
    cfg_.ring_phase = ring_phase;
    build_mixing();
    last_fin_.assign(cfg_.nfins, 0.0);
}

void FinMpc::set_layout_azimuths(const std::vector<double>& phi) {
    cfg_.nfins = std::max(1, static_cast<int>(phi.size()));
    build_mixing(phi);
    last_fin_.assign(cfg_.nfins, 0.0);
}

void FinMpc::reset() {
    have_prev_ = false;
    Suu_ = {0.0, 0.0, 0.0};
    Suy_ = {0.0, 0.0, 0.0};
    sign_ = {1.0, 1.0, 1.0};
    b_ = cfg_.b_default;
    last_axis_ = {0.0, 0.0, 0.0};
    prev_omega_ = {0.0, 0.0, 0.0};
    last_fin_.assign(cfg_.nfins, 0.0);
    mode_ = FinMode::WaitQ;
    dither_time_ = 0.0;
    sign_frozen_ = false;
}

// Normalised 3 x nfins mixing matrix for fins equally spaced around the roll
// axis. Fin i sits at angle phi_i = ring_phase + i*2pi/n. Deflecting it produces
// pitch ~ cos(phi), yaw ~ sin(phi), and roll ~ +1 (all fins roll together).
// Rows are scaled by their L1 norm so an axis command M_a.delta lands in [-1,1]
// when every |delta_i| <= 1 -- i.e. the axis command IS the normalised input the
// per-axis effectiveness b_a was identified against.
void FinMpc::build_mixing() {
    const int n = cfg_.nfins;
    std::vector<double> phi(n);
    for (int i = 0; i < n; ++i) phi[i] = cfg_.ring_phase + (2.0 * M_PI * i) / n;
    build_mixing(phi);
}

void FinMpc::build_mixing(const std::vector<double>& phi) {
    const int n = static_cast<int>(phi.size());
    M_.assign(3, std::vector<double>(n, 0.0));
    std::array<double, 3> l1 = {0.0, 0.0, 0.0};
    for (int i = 0; i < n; ++i) {
        M_[0][i] = std::cos(phi[i]);   // pitch
        M_[1][i] = std::sin(phi[i]);   // yaw
        M_[2][i] = 1.0;                // roll (all fins roll together)
        for (int a = 0; a < 3; ++a) l1[a] += std::abs(M_[a][i]);
    }
    for (int a = 0; a < 3; ++a)
        if (l1[a] > 1e-9)
            for (int i = 0; i < n; ++i) M_[a][i] /= l1[a];
}

// Box-constrained projected-gradient solve of
//   min_delta  sum_a w_a (b_a*sign_a*(M_a.delta) - g_a)^2 + reg*|delta|^2
//   s.t. |delta_i| <= fin_limit.
// g is the desired per-axis angular acceleration. Returns the per-fin deflection.
std::vector<double> FinMpc::solve_qp(const Vec3& g) const {
    const int n = cfg_.nfins;
    const double w[3]  = {cfg_.w_pitch, cfg_.w_yaw, cfg_.w_roll};
    const double be[3] = {b_[0] * sign_[0], b_[1] * sign_[1], b_[2] * sign_[2]};

    // Lipschitz constant of the gradient: H = sum_a 2 w_a be_a^2 m_a m_a^T + 2 reg I,
    // so lambda_max(H) <= sum_a 2 w_a be_a^2 ||m_a||^2 + 2 reg. A fixed step that
    // ignores this (the previous bug) overshoots the box every iteration and bangs
    // between corners -- the result then depends only on the iteration parity.
    double L = 2.0 * cfg_.reg;
    for (int a = 0; a < 3; ++a) {
        double m2 = 0.0;
        for (int i = 0; i < n; ++i) m2 += M_[a][i] * M_[a][i];
        L += 2.0 * w[a] * be[a] * be[a] * m2;
    }
    const double step = 0.9 / std::max(1e-9, L);

    std::vector<double> d(n, 0.0);
    for (int it = 0; it < cfg_.pgd_iters; ++it) {
        // Residual per axis: r_a = be_a*(M_a.delta) - g_a.
        double r[3];
        for (int a = 0; a < 3; ++a) {
            double md = 0.0;
            for (int i = 0; i < n; ++i) md += M_[a][i] * d[i];
            r[a] = be[a] * md - g[a];
        }
        // Gradient wrt delta_j; projected-gradient step with the Lipschitz step.
        for (int j = 0; j < n; ++j) {
            double grad = 2.0 * cfg_.reg * d[j];
            for (int a = 0; a < 3; ++a)
                grad += 2.0 * w[a] * r[a] * be[a] * M_[a][j];
            d[j] = clampd(d[j] - step * grad, -cfg_.fin_limit, cfg_.fin_limit);
        }
    }
    return d;
}

// Recursive least-squares-through-origin of the per-axis plant gain from the
// LAST applied axis command and the resulting axis angular acceleration:
//   accel_a ~= g_a * u_a   ->   sign+magnitude of g_a.
// Decaying sums so it tracks; gated on dynamic pressure (fins make no torque in
// near-vacuum, so a low-q response is just noise).
void FinMpc::identify(const Vec3& omega, double dt, double q) {
    if (have_prev_ && dt > 1e-4 && q >= cfg_.q_ident_pa) {
        const double decay = 0.96;
        for (int a = 0; a < 3; ++a) {
            const double accel = (omega[a] - prev_omega_[a]) / dt;
            const double u = last_axis_[a];
            Suu_[a] = decay * Suu_[a] + u * u;
            Suy_[a] = decay * Suy_[a] + u * accel;
            if (Suu_[a] > 0.02) {
                const double b = Suy_[a] / Suu_[a];   // signed gain estimate
                if (std::abs(b) > 1e-3) {
                    // All control signs are frozen after the startup dither (the
                    // global deployAngle sign); only the magnitude keeps adapting.
                    if (!sign_frozen_) sign_[a] = b > 0.0 ? 1.0 : -1.0;
                    b_[a] = clampd(std::abs(b), cfg_.b_floor, cfg_.b_ceil);
                }
            }
        }
    }
    prev_omega_ = omega;
    have_prev_ = true;
}

FinCommand FinMpc::update(const Vec3& omega, double dt, double q) {
    FinCommand cmd;
    if (static_cast<int>(last_fin_.size()) != cfg_.nfins) last_fin_.assign(cfg_.nfins, 0.0);

    identify(omega, dt, q);

    // ---- startup state machine: wait for q, then dither to lock the roll sign --
    if (mode_ == FinMode::WaitQ) {
        if (q >= cfg_.q_ident_pa) { mode_ = FinMode::Dither; dither_time_ = 0.0; }
        else { last_axis_ = {0,0,0}; last_fin_.assign(cfg_.nfins, 0.0);
               cmd.fin_deflection = last_fin_; return cmd; }   // no authority yet
    }
    if (mode_ == FinMode::Dither) {
        dither_time_ += dt;
        // Three sequential stages -- excite ONE axis at a time so each axis's
        // control sign is identified cleanly and independently (no global-sign
        // assumption; a deployAngle inversion may differ per axis / per azimuth
        // convention). Stage order: roll, pitch, yaw. For axis `ax`, the minimum-
        // norm deflection giving a pure axis command is d = (s*amp/||M_ax||^2)*M_ax,
        // which leaves the other (orthogonal) axes ~0.
        const double stage_dur = cfg_.dither_window / 3.0;
        const int    stage = std::min(2, static_cast<int>(dither_time_ / stage_dur));
        const int    ax = (stage == 0) ? 2 : (stage == 1) ? 0 : 1;   // roll, pitch, yaw
        const double local_t = dither_time_ - stage * stage_dur;
        const int    slot = static_cast<int>(local_t / cfg_.dither_hold);
        const double s = (slot % 2 == 0) ? 1.0 : -1.0;
        double m2 = 0.0;
        for (int i = 0; i < cfg_.nfins; ++i) m2 += M_[ax][i] * M_[ax][i];
        const double scale = (m2 > 1e-9) ? (s * cfg_.dither_amp / m2) : 0.0;
        std::vector<double> d(cfg_.nfins, 0.0);
        for (int i = 0; i < cfg_.nfins; ++i)
            d[i] = clampd(scale * M_[ax][i], -cfg_.fin_limit, cfg_.fin_limit);
        Vec3 axis = {0.0, 0.0, 0.0};
        for (int a = 0; a < 3; ++a)
            for (int i = 0; i < cfg_.nfins; ++i) axis[a] += M_[a][i] * d[i];
        cmd.axis = axis; cmd.fin_deflection = d;
        last_fin_ = d; last_axis_ = axis;
        if (dither_time_ >= cfg_.dither_window) { sign_frozen_ = true; mode_ = FinMode::Active; }
        return cmd;
    }

    // ---- Active: track rate, suppress roll ----
    // Desired per-axis angular acceleration: drive the body rate to the target
    // (roll target is always 0 -> active roll damping).
    Vec3 g;
    for (int a = 0; a < 3; ++a) g[a] = cfg_.k_track * (target_rate_[a] - omega[a]);

    // ROLL PRIORITY: while the roll rate is large, throttle the pitch/yaw demand
    // toward zero. Pointing deflections feed roll through the coupling, so chasing
    // the aim while roll is diverging is self-defeating -- kill roll first.
    const double roll_excess = std::abs(omega[2]) - cfg_.roll_prio_rate;
    if (roll_excess > 0.0) {
        const double s = std::max(0.0, 1.0 - cfg_.roll_prio_k * roll_excess);
        g[0] *= s;
        g[1] *= s;
    }

    // Per-fin allocation (these ARE the actuator commands -- written straight to
    // each fin's deployAngle), slew-limited per fin to model the actuator rate.
    std::vector<double> d = solve_qp(g);
    if (static_cast<int>(last_fin_.size()) != cfg_.nfins) last_fin_.assign(cfg_.nfins, 0.0);
    auto slew = [](double t, double p, double m) {
        const double dd = t - p; return dd > m ? p + m : (dd < -m ? p - m : t);
    };
    for (int i = 0; i < cfg_.nfins; ++i)
        d[i] = clampd(slew(d[i], last_fin_[i], cfg_.slew_fin), -cfg_.fin_limit, cfg_.fin_limit);

    // Effective per-axis command (for online ID + telemetry): axis = M . delta.
    Vec3 axis = {0.0, 0.0, 0.0};
    for (int a = 0; a < 3; ++a) {
        double md = 0.0;
        for (int i = 0; i < cfg_.nfins; ++i) md += M_[a][i] * d[i];
        axis[a] = clampd(md, -1.0, 1.0);
    }

    const double wmax = std::max({std::abs(omega[0]), std::abs(omega[1]), std::abs(omega[2])});
    cmd.abort = wmax > cfg_.rate_abort;

    cmd.axis = axis;
    cmd.fin_deflection = d;     // normalised [-1,1] per fin -> scale to deployAngle
    cmd.dbg_g = g;
    cmd.dbg_be = {b_[0]*sign_[0], b_[1]*sign_[1], b_[2]*sign_[2]};
    last_fin_ = d;
    last_axis_ = axis;          // regressor for the next identify() step
    return cmd;
}

} // namespace gridfin
