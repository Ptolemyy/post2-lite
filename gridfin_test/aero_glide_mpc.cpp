// aero_glide_mpc.cpp -- implementation of the 6-DOF CEM aero-glide MPC.

#include "aero_glide_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace gridfin {

// ---------------------------------------------------------------------------
double Atmosphere::density(double alt) const {
    if (h.size() < 2 || rho.size() != h.size()) {
        return rho0 * std::exp(-std::max(0.0, alt) / scale_h);
    }
    if (alt <= h.front()) return rho.front();
    if (alt >= h.back()) {
        const double h0 = h[h.size() - 2], h1 = h.back();
        const double r0 = std::max(1e-9, rho[rho.size() - 2]), r1 = std::max(1e-12, rho.back());
        const double H = (h1 - h0) / std::max(1e-6, std::log(r0 / r1));
        return r1 * std::exp(-(alt - h1) / std::max(500.0, H));
    }
    const auto it = std::upper_bound(h.begin(), h.end(), alt);
    const size_t j = static_cast<size_t>(it - h.begin());
    const double t = (alt - h[j - 1]) / std::max(1e-6, h[j] - h[j - 1]);
    return rho[j - 1] + t * (rho[j] - rho[j - 1]);
}

// ---------------------------------------------------------------------------
AeroGlideMpc::AeroGlideMpc(const MpcConfig& cfg) : cfg_(cfg), rng_(0xC0FFEEu) {}

static void horizontal_basis(const Vec3& pos, Vec3* h1, Vec3* h2) {
    const Vec3 up = v_hat(pos);
    Vec3 ref = {0.0, 1.0, 0.0};                 // body spin axis (north pole)
    if (std::abs(v_dot(ref, up)) > 0.95) ref = {1.0, 0.0, 0.0};
    *h1 = v_hat(v_perp(ref, up));
    *h2 = v_hat(v_cross(up, *h1));
}

// Desired nose direction for a steering control (u1,u2) at velocity unit `vhat`:
// retrograde (-vhat) tilted by angle of attack toward the in-plane steer
// direction. lift_sign flips the tilt side so the resulting REAL aero force (the
// rollout uses lift_sign*CL too) points toward the steer direction.
static Vec3 steer_to_nose(double u1, double u2, const Vec3& h1, const Vec3& h2,
                          const Vec3& vhat, double max_aoa, double lift_sign,
                          Vec3* lift_dir_out, double* aoa_out) {
    Vec3 d = v_add(v_scale(h1, u1), v_scale(h2, u2));
    const double alpha = std::min(v_norm(d), max_aoa);
    Vec3 L = v_scale(v_hat(v_perp(d, vhat)), lift_sign);
    if (lift_dir_out) *lift_dir_out = L;
    if (aoa_out) *aoa_out = alpha;
    Vec3 nose = v_add(v_scale(vhat, -std::cos(alpha)), v_scale(L, std::sin(alpha)));
    return v_hat(nose);
}

double AeroGlideMpc::rollout(const std::vector<double>& u, const Vec3& pos0,
                             const Vec3& vel0, const Vec3& nose0, const Vec3& omega0,
                             const Vec3& target, const GlideEnv& env, double target_alt,
                             const Vec3& h1, const Vec3& h2,
                             double* pos_err_out, double* vh_out, double* att_err_out,
                             double* t_reach_out, bool* reached_out) const {
    const int    nodes   = cfg_.n_nodes;
    const double node_dt = cfg_.t_max / std::max(1, nodes);
    const double dt      = cfg_.dt_sim;
    const double S       = env.s_ref;
    const double m       = std::max(1.0, env.mass);

    Vec3 p = pos0, v = vel0, nhat = v_hat(nose0), w = omega0;
    double t = 0.0, effort = 0.0, last_alpha = 0.0;
    bool reached = false;

    auto node_of = [&](double tt) {
        int k = static_cast<int>(tt / node_dt);
        return k < 0 ? 0 : (k >= nodes ? nodes - 1 : k);
    };

    for (int step = 0; step < 4000; ++step) {
        const double r  = v_norm(p);
        const double h  = r - env.r_body;
        const Vec3   up = v_scale(p, 1.0 / std::max(1e-6, r));
        if (h <= target_alt) { reached = true; break; }
        if (t >= cfg_.t_max) break;

        const double speed = v_norm(v);
        const Vec3   vhat  = speed > 1e-6 ? v_scale(v, 1.0 / speed) : v_scale(up, -1.0);

        // Steering setpoint for this node.
        const int    k  = node_of(t);
        double alpha_cmd = 0.0; Vec3 lcmd;
        const Vec3 n_cmd = steer_to_nose(u[2 * k], u[2 * k + 1], h1, h2, vhat,
                                         env.max_aoa, env.lift_sign, &lcmd, &alpha_cmd);
        last_alpha = alpha_cmd;

        // ---- attitude dynamics: rotate the ACTUAL nose toward n_cmd, rate- and
        // acceleration-limited (models the inner loop + airframe). Roll (w along
        // nhat) does not move the nose and is inert here.
        const double rho  = env.atmo ? env.atmo->density(h) : 0.0;
        const double a_snd= env.atmo ? env.atmo->sound_speed(h) : 300.0;
        const double q    = 0.5 * rho * speed * speed;
        Vec3 w_des = v_scale(v_cross(nhat, n_cmd), env.att_kp);   // ~ att_kp*sin(err)
        const double wdn = v_norm(w_des);
        if (wdn > env.omega_max) w_des = v_scale(w_des, env.omega_max / wdn);
        const double accel_lim = std::max(0.02, env.accel_max * (q / std::max(1.0, env.q_ref)));
        Vec3 dw = v_sub(w_des, w);
        const double dwn = v_norm(dw);
        if (dwn > accel_lim * dt) dw = v_scale(dw, accel_lim * dt / dwn);
        w = v_add(w, dw);
        nhat = v_hat(v_add(nhat, v_scale(v_cross(w, nhat), dt)));

        // ---- aero force from the ACTUAL attitude ----
        const double aoa = std::acos(clampd(-v_dot(nhat, vhat), -1.0, 1.0));
        const Vec3   L_act = v_hat(v_perp(nhat, vhat));
        const double mach = speed / std::max(1.0, a_snd);
        double cd = 0.0, cl = 0.0;
        if (env.table) env.table->lookup(mach, aoa * kRadToDeg, &cd, &cl);
        const Vec3 f_aero = v_scale(v_sub(v_scale(L_act, cl * env.lift_sign),
                                          v_scale(vhat, cd)), q * S);
        const Vec3 a_aero = v_scale(f_aero, 1.0 / m);
        const Vec3 a_grav = v_scale(up, -env.mu / (r * r));
        const Vec3 acc    = v_add(a_aero, a_grav);

        for (int i = 0; i < 3; ++i) { v[i] += acc[i] * dt; p[i] += v[i] * dt; }
        t += dt;
        effort += alpha_cmd * alpha_cmd * dt;
    }

    // Node-to-node smoothness of the control profile.
    double smooth = 0.0;
    for (int k = 1; k < nodes; ++k) {
        const double da1 = u[2 * k] - u[2 * (k - 1)];
        const double da2 = u[2 * k + 1] - u[2 * (k - 1) + 1];
        smooth += da1 * da1 + da2 * da2;
    }

    const double r  = v_norm(p);
    const double h  = r - env.r_body;
    const Vec3   up = v_scale(p, 1.0 / std::max(1e-6, r));
    const double pos_err = v_norm(v_perp(v_sub(p, target), up));   // horizontal miss
    const double vh      = v_norm(v_perp(v, up));                  // horizontal speed
    const double alt_err = reached ? 0.0 : std::abs(h - target_alt);
    const double att_err = std::acos(clampd(v_dot(nhat, up), -1.0, 1.0)); // nose-from-vertical
    const double w_term  = v_norm(w);

    if (pos_err_out) *pos_err_out = pos_err;
    if (vh_out)      *vh_out      = vh;
    if (att_err_out) *att_err_out = att_err;
    if (t_reach_out) *t_reach_out = t;
    if (reached_out) *reached_out = reached;

    const double c_pos  = (pos_err / cfg_.pos_scale) * (pos_err / cfg_.pos_scale);
    const double c_vh   = (vh / cfg_.vh_scale) * (vh / cfg_.vh_scale);
    const double c_alt  = cfg_.w_altmiss * (alt_err / cfg_.alt_scale) * (alt_err / cfg_.alt_scale);
    const double c_term = cfg_.w_term_aoa * last_alpha * last_alpha;
    const double c_att  = cfg_.w_attvert * att_err * att_err;
    const double c_om   = cfg_.w_omega * w_term * w_term;
    return c_pos + c_vh + c_alt + cfg_.w_effort * effort + cfg_.w_smooth * smooth
         + c_term + c_att + c_om;
}

Vec3 AeroGlideMpc::node0_aim(const std::vector<double>& mean, const Vec3& vel,
                             const Vec3& h1, const Vec3& h2, double max_aoa,
                             Vec3* lift_dir, double* aoa) const {
    const double speed = v_norm(vel);
    const Vec3   vhat  = speed > 1e-6 ? v_scale(vel, 1.0 / speed) : Vec3{0.0, 0.0, 0.0};
    return steer_to_nose(mean[0], mean[1], h1, h2, vhat, max_aoa,
                         lift_sign_, lift_dir, aoa);
}

MpcResult AeroGlideMpc::plan(const Vec3& pos, const Vec3& vel, const Vec3& nose,
                             const Vec3& omega, const Vec3& target,
                             const GlideEnv& env, double target_alt) {
    MpcResult res;
    lift_sign_ = env.lift_sign;
    const int dim = 2 * cfg_.n_nodes;

    const double speed = v_norm(vel);
    if (speed < cfg_.min_speed) {
        res.aim_hat = v_hat(pos);
        res.holding = true;
        has_warm_ = false;
        return res;
    }

    Vec3 h1, h2;
    horizontal_basis(pos, &h1, &h2);

    std::vector<double> mean(dim, 0.0);
    if (has_warm_ && static_cast<int>(warm_.size()) == dim) {
        for (int k = 0; k < cfg_.n_nodes - 1; ++k) {
            mean[2 * k]     = warm_[2 * (k + 1)];
            mean[2 * k + 1] = warm_[2 * (k + 1) + 1];
        }
        mean[2 * (cfg_.n_nodes - 1)]     = warm_[2 * (cfg_.n_nodes - 1)];
        mean[2 * (cfg_.n_nodes - 1) + 1] = warm_[2 * (cfg_.n_nodes - 1) + 1];
    }
    std::vector<double> stdv(dim, cfg_.init_std);

    std::vector<std::vector<double>> samples(cfg_.cem_samples, std::vector<double>(dim));
    std::vector<std::pair<double, int>> scored(cfg_.cem_samples);

    for (int iter = 0; iter < cfg_.cem_iters; ++iter) {
        for (int s = 0; s < cfg_.cem_samples; ++s) {
            auto& x = samples[s];
            for (int k = 0; k < cfg_.n_nodes; ++k) {
                std::normal_distribution<double> g1(mean[2 * k], stdv[2 * k]);
                std::normal_distribution<double> g2(mean[2 * k + 1], stdv[2 * k + 1]);
                double a1 = g1(rng_), a2 = g2(rng_);
                const double mag = std::sqrt(a1 * a1 + a2 * a2);
                if (mag > env.max_aoa && mag > 1e-9) {
                    a1 *= env.max_aoa / mag;
                    a2 *= env.max_aoa / mag;
                }
                x[2 * k] = a1;
                x[2 * k + 1] = a2;
            }
            const double c = rollout(x, pos, vel, nose, omega, target, env, target_alt,
                                     h1, h2, nullptr, nullptr, nullptr, nullptr, nullptr);
            scored[s] = {c, s};
        }
        std::partial_sort(scored.begin(), scored.begin() + cfg_.cem_elite, scored.end());

        std::vector<double> newmean(dim, 0.0), newvar(dim, 0.0);
        for (int e = 0; e < cfg_.cem_elite; ++e) {
            const auto& x = samples[scored[e].second];
            for (int d = 0; d < dim; ++d) newmean[d] += x[d];
        }
        for (int d = 0; d < dim; ++d) newmean[d] /= cfg_.cem_elite;
        for (int e = 0; e < cfg_.cem_elite; ++e) {
            const auto& x = samples[scored[e].second];
            for (int d = 0; d < dim; ++d) { const double dv = x[d] - newmean[d]; newvar[d] += dv * dv; }
        }
        mean = newmean;
        for (int d = 0; d < dim; ++d)
            stdv[d] = std::max(cfg_.std_floor, std::sqrt(newvar[d] / cfg_.cem_elite));
    }

    res.aim_hat = node0_aim(mean, vel, h1, h2, env.max_aoa, &res.lift_dir, &res.aoa_rad);
    double att_err = 0.0;
    res.cost = rollout(mean, pos, vel, nose, omega, target, env, target_alt, h1, h2,
                       &res.pred_pos_err_m, &res.pred_vh_mps, &att_err, &res.pred_time_s,
                       &res.reached);
    res.pred_att_err_deg = att_err * kRadToDeg;

    warm_ = mean;
    has_warm_ = true;
    return res;
}

} // namespace gridfin
