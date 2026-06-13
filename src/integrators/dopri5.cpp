#include "post2/integrators/dopri5.hpp"

#include <algorithm>
#include <cmath>

namespace post2::integrators {

namespace {

// --- Dormand-Prince 5(4) Butcher tableau ------------------------------------
// c_i, a_ij, b_i (5th-order solution), e_i = b_i - b*_i (embedded error).
constexpr double kC2 = 1.0 / 5.0;
constexpr double kC3 = 3.0 / 10.0;
constexpr double kC4 = 4.0 / 5.0;
constexpr double kC5 = 8.0 / 9.0;
constexpr double kC6 = 1.0;
constexpr double kC7 = 1.0;

constexpr double kA21 = 1.0 / 5.0;

constexpr double kA31 = 3.0 / 40.0;
constexpr double kA32 = 9.0 / 40.0;

constexpr double kA41 = 44.0 / 45.0;
constexpr double kA42 = -56.0 / 15.0;
constexpr double kA43 = 32.0 / 9.0;

constexpr double kA51 = 19372.0 / 6561.0;
constexpr double kA52 = -25360.0 / 2187.0;
constexpr double kA53 = 64448.0 / 6561.0;
constexpr double kA54 = -212.0 / 729.0;

constexpr double kA61 = 9017.0 / 3168.0;
constexpr double kA62 = -355.0 / 33.0;
constexpr double kA63 = 46732.0 / 5247.0;
constexpr double kA64 = 49.0 / 176.0;
constexpr double kA65 = -5103.0 / 18656.0;

constexpr double kA71 = 35.0 / 384.0;
constexpr double kA72 = 0.0;
constexpr double kA73 = 500.0 / 1113.0;
constexpr double kA74 = 125.0 / 192.0;
constexpr double kA75 = -2187.0 / 6784.0;
constexpr double kA76 = 11.0 / 84.0;

// 5th-order solution coefficients (= a7j).
constexpr double kB1 = kA71;
constexpr double kB3 = kA73;
constexpr double kB4 = kA74;
constexpr double kB5 = kA75;
constexpr double kB6 = kA76;
constexpr double kB7 = 0.0;

// Error coefficients e_i = b_i - b*_i, where b*_i is the 4th-order solution.
constexpr double kE1 = 71.0 / 57600.0;
constexpr double kE3 = -71.0 / 16695.0;
constexpr double kE4 = 71.0 / 1920.0;
constexpr double kE5 = -17253.0 / 339200.0;
constexpr double kE6 = 22.0 / 525.0;
constexpr double kE7 = -1.0 / 40.0;

constexpr double kSafetyFactor = 0.9;
constexpr double kFacMin = 0.2;
constexpr double kFacMax = 5.0;
constexpr double kOrderInv = 1.0 / 5.0;

ExtendedDerivative scale_derivative(const ExtendedDerivative& d, double s)
{
    ExtendedDerivative out;
    out.motion_dot.d_position_mps = d.motion_dot.d_position_mps * s;
    out.motion_dot.d_velocity_mps2 = d.motion_dot.d_velocity_mps2 * s;
    out.tank_mass_dots_kgps.resize(d.tank_mass_dots_kgps.size());
    for (std::size_t i = 0; i < d.tank_mass_dots_kgps.size(); ++i) {
        out.tank_mass_dots_kgps[i] = d.tank_mass_dots_kgps[i] * s;
    }
    return out;
}

ExtendedState add_state_scaled_derivative(
    const ExtendedState& s, const ExtendedDerivative& d, double scale)
{
    ExtendedState out;
    out.motion.position_m = s.motion.position_m + d.motion_dot.d_position_mps * scale;
    out.motion.velocity_mps = s.motion.velocity_mps + d.motion_dot.d_velocity_mps2 * scale;
    out.tank_masses_kg.resize(s.tank_masses_kg.size());
    const std::size_t n = std::min(s.tank_masses_kg.size(), d.tank_mass_dots_kgps.size());
    for (std::size_t i = 0; i < n; ++i) {
        out.tank_masses_kg[i] = s.tank_masses_kg[i] + d.tank_mass_dots_kgps[i] * scale;
    }
    for (std::size_t i = n; i < s.tank_masses_kg.size(); ++i) {
        out.tank_masses_kg[i] = s.tank_masses_kg[i];
    }
    return out;
}

// Computes y_new = y + h * sum(coef_i * k_i) for a 7-stage RK step. Stages
// with coef == 0 are skipped.
ExtendedState combine_stages(
    const ExtendedState& y,
    double h,
    const ExtendedDerivative* ks,
    const double* coefs,
    std::size_t n_stages)
{
    ExtendedState out = y;
    for (std::size_t s = 0; s < n_stages; ++s) {
        if (coefs[s] == 0.0) {
            continue;
        }
        const double scale = h * coefs[s];
        out.motion.position_m = out.motion.position_m + ks[s].motion_dot.d_position_mps * scale;
        out.motion.velocity_mps = out.motion.velocity_mps + ks[s].motion_dot.d_velocity_mps2 * scale;
        const std::size_t n = std::min(out.tank_masses_kg.size(), ks[s].tank_mass_dots_kgps.size());
        for (std::size_t i = 0; i < n; ++i) {
            out.tank_masses_kg[i] += ks[s].tank_mass_dots_kgps[i] * scale;
        }
    }
    return out;
}

// Hermite cubic interpolation between (t_n, y_n, k1) and (t_n+h, y_new, k7)
// at fractional position theta in [0, 1]. k7 here uses FSAL: it's f at the
// end-of-step state, which equals next-step's k1.
ExtendedState hermite_interpolate(
    const ExtendedState& y_n,
    const ExtendedState& y_new,
    const ExtendedDerivative& k1,
    const ExtendedDerivative& k7,
    double h,
    double theta)
{
    const double a1 = 1.0 - 3.0 * theta * theta + 2.0 * theta * theta * theta;
    const double a2 = 3.0 * theta * theta - 2.0 * theta * theta * theta;
    const double a3 = (theta - 2.0 * theta * theta + theta * theta * theta) * h;
    const double a4 = (-theta * theta + theta * theta * theta) * h;

    ExtendedState out;
    out.motion.position_m =
        y_n.motion.position_m * a1 + y_new.motion.position_m * a2 +
        k1.motion_dot.d_position_mps * a3 + k7.motion_dot.d_position_mps * a4;
    out.motion.velocity_mps =
        y_n.motion.velocity_mps * a1 + y_new.motion.velocity_mps * a2 +
        k1.motion_dot.d_velocity_mps2 * a3 + k7.motion_dot.d_velocity_mps2 * a4;
    out.tank_masses_kg.resize(y_n.tank_masses_kg.size());
    for (std::size_t i = 0; i < y_n.tank_masses_kg.size(); ++i) {
        const double y0 = y_n.tank_masses_kg[i];
        const double y1 = i < y_new.tank_masses_kg.size() ? y_new.tank_masses_kg[i] : y0;
        const double d0 = i < k1.tank_mass_dots_kgps.size() ? k1.tank_mass_dots_kgps[i] : 0.0;
        const double d1 = i < k7.tank_mass_dots_kgps.size() ? k7.tank_mass_dots_kgps[i] : 0.0;
        out.tank_masses_kg[i] = y0 * a1 + y1 * a2 + d0 * a3 + d1 * a4;
    }
    return out;
}

// WRMS error norm using per-group atol + global rtol.
double error_norm(
    const ExtendedState& y,
    const ExtendedState& y_new,
    const ExtendedDerivative& err_per_unit_h,
    double h,
    const IntegratorTolerances& tol)
{
    auto scale = [](double atol, double rtol, double a, double b) {
        return atol + rtol * std::max(std::abs(a), std::abs(b));
    };

    auto sq = [](double v) { return v * v; };

    double sum_sq = 0.0;
    std::size_t count = 0;

    // Position components
    {
        const double sx = scale(tol.atol_position_m, tol.rtol, y.motion.position_m.x, y_new.motion.position_m.x);
        const double sy = scale(tol.atol_position_m, tol.rtol, y.motion.position_m.y, y_new.motion.position_m.y);
        const double sz = scale(tol.atol_position_m, tol.rtol, y.motion.position_m.z, y_new.motion.position_m.z);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_position_mps.x / sx);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_position_mps.y / sy);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_position_mps.z / sz);
        count += 3;
    }
    // Velocity components
    {
        const double sx = scale(tol.atol_velocity_mps, tol.rtol, y.motion.velocity_mps.x, y_new.motion.velocity_mps.x);
        const double sy = scale(tol.atol_velocity_mps, tol.rtol, y.motion.velocity_mps.y, y_new.motion.velocity_mps.y);
        const double sz = scale(tol.atol_velocity_mps, tol.rtol, y.motion.velocity_mps.z, y_new.motion.velocity_mps.z);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_velocity_mps2.x / sx);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_velocity_mps2.y / sy);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_velocity_mps2.z / sz);
        count += 3;
    }
    // Tank masses
    for (std::size_t i = 0; i < y.tank_masses_kg.size(); ++i) {
        const double y0 = y.tank_masses_kg[i];
        const double y1 = i < y_new.tank_masses_kg.size() ? y_new.tank_masses_kg[i] : y0;
        const double sc = scale(tol.atol_tank_mass_kg, tol.rtol, y0, y1);
        const double err_i = i < err_per_unit_h.tank_mass_dots_kgps.size()
            ? err_per_unit_h.tank_mass_dots_kgps[i] : 0.0;
        sum_sq += sq(h * err_i / sc);
        count += 1;
    }

    if (count == 0) {
        return 0.0;
    }
    return std::sqrt(sum_sq / static_cast<double>(count));
}

// Bisection root-finder for g(theta) on theta in [0, 1].
// Converges to |theta_hi - theta_lo| < 1e-12 (i.e. |Δt| < 1e-12 * h_used).
double bisect_zero(
    double g_lo,
    double g_hi,
    const std::function<double(double)>& g_at_theta)
{
    double lo = 0.0;
    double hi = 1.0;
    double f_lo = g_lo;
    double f_hi = g_hi;
    for (int iter = 0; iter < 80; ++iter) {
        const double mid = 0.5 * (lo + hi);
        const double f_mid = g_at_theta(mid);
        if ((f_lo <= 0.0 && f_mid >= 0.0) || (f_lo >= 0.0 && f_mid <= 0.0)) {
            hi = mid;
            f_hi = f_mid;
        } else {
            lo = mid;
            f_lo = f_mid;
        }
        if (hi - lo < 1.0e-12) {
            return 0.5 * (lo + hi);
        }
        // Avoid infinite work on pathological g
        if (std::abs(f_mid) < 1.0e-15) {
            return mid;
        }
        (void)f_hi;
    }
    return 0.5 * (lo + hi);
}

// Common helper: examine events for sign changes between t_n and t_n+h_used,
// pick the earliest crossing (Brent on dense output), and return the
// corresponding EventHit. Returns nullopt if no event fires.
std::optional<EventHit> find_first_event(
    const std::vector<EventFunction>& events,
    double t_n,
    double h_used,
    const ExtendedState& y_n,
    const ExtendedState& y_new,
    const std::function<ExtendedState(double theta)>& interp)
{
    std::optional<EventHit> best;
    for (std::size_t i = 0; i < events.size(); ++i) {
        const auto& ev = events[i];
        if (!ev.g) {
            continue;
        }
        const double g0 = ev.g(t_n, y_n);
        const double g1 = ev.g(t_n + h_used, y_new);
        const bool sign_change = (g0 < 0.0 && g1 > 0.0) || (g0 > 0.0 && g1 < 0.0) ||
                                 (g0 == 0.0 && g1 != 0.0);
        if (!sign_change) {
            continue;
        }
        const double theta = bisect_zero(g0, g1, [&](double th) {
            const ExtendedState s = interp(th);
            return ev.g(t_n + th * h_used, s);
        });
        EventHit hit;
        hit.event_index = i;
        hit.name = ev.name;
        hit.t_s = t_n + theta * h_used;
        hit.state = interp(theta);
        if (!best.has_value() || hit.t_s < best->t_s) {
            best = hit;
        }
    }
    return best;
}

} // namespace

// =========================================================================
// Dopri5Integrator
// =========================================================================

Dopri5Integrator::Dopri5Integrator(IntegratorTolerances tolerances)
    : tolerances_(tolerances)
{
}

StepResult Dopri5Integrator::step(
    const ExtendedState& state,
    double t_s,
    double h_suggested,
    const DynamicsFunction& dynamics,
    const std::vector<EventFunction>& events)
{
    double h = std::abs(h_suggested);
    if (h <= 0.0) {
        StepResult res;
        res.state_end = state;
        res.t_end = t_s;
        res.h_used = 0.0;
        res.h_next_suggested = 0.0;
        res.accepted = true;
        return res;
    }

    // Cap retries; in practice 10 is generous.
    for (int attempt = 0; attempt < 12; ++attempt) {
        const ExtendedDerivative k1 = dynamics(t_s, state);

        const ExtendedState y2 = add_state_scaled_derivative(state, k1, h * kA21);
        const ExtendedDerivative k2 = dynamics(t_s + kC2 * h, y2);

        const double a31[2] = {kA31, kA32};
        const ExtendedDerivative kt31[2] = {k1, k2};
        const ExtendedState y3 = combine_stages(state, h, kt31, a31, 2);
        const ExtendedDerivative k3 = dynamics(t_s + kC3 * h, y3);

        const double a41[3] = {kA41, kA42, kA43};
        const ExtendedDerivative kt41[3] = {k1, k2, k3};
        const ExtendedState y4 = combine_stages(state, h, kt41, a41, 3);
        const ExtendedDerivative k4 = dynamics(t_s + kC4 * h, y4);

        const double a51[4] = {kA51, kA52, kA53, kA54};
        const ExtendedDerivative kt51[4] = {k1, k2, k3, k4};
        const ExtendedState y5 = combine_stages(state, h, kt51, a51, 4);
        const ExtendedDerivative k5 = dynamics(t_s + kC5 * h, y5);

        const double a61[5] = {kA61, kA62, kA63, kA64, kA65};
        const ExtendedDerivative kt61[5] = {k1, k2, k3, k4, k5};
        const ExtendedState y6 = combine_stages(state, h, kt61, a61, 5);
        const ExtendedDerivative k6 = dynamics(t_s + kC6 * h, y6);

        const double b_solution[6] = {kB1, kA72, kB3, kB4, kB5, kB6};
        const ExtendedDerivative kt_sol[6] = {k1, k2, k3, k4, k5, k6};
        ExtendedState y_new = combine_stages(state, h, kt_sol, b_solution, 6);

        // FSAL stage k7 = f at the end-of-step state (used by both the error
        // estimate and the Hermite interpolation).
        const ExtendedDerivative k7 = dynamics(t_s + h, y_new);

        // Error per unit step: e_i * k_i summed.
        ExtendedDerivative err;
        err.motion_dot.d_position_mps =
            k1.motion_dot.d_position_mps * kE1 +
            k3.motion_dot.d_position_mps * kE3 +
            k4.motion_dot.d_position_mps * kE4 +
            k5.motion_dot.d_position_mps * kE5 +
            k6.motion_dot.d_position_mps * kE6 +
            k7.motion_dot.d_position_mps * kE7;
        err.motion_dot.d_velocity_mps2 =
            k1.motion_dot.d_velocity_mps2 * kE1 +
            k3.motion_dot.d_velocity_mps2 * kE3 +
            k4.motion_dot.d_velocity_mps2 * kE4 +
            k5.motion_dot.d_velocity_mps2 * kE5 +
            k6.motion_dot.d_velocity_mps2 * kE6 +
            k7.motion_dot.d_velocity_mps2 * kE7;
        err.tank_mass_dots_kgps.assign(state.tank_masses_kg.size(), 0.0);
        for (std::size_t i = 0; i < err.tank_mass_dots_kgps.size(); ++i) {
            auto val = [&](const ExtendedDerivative& d) {
                return i < d.tank_mass_dots_kgps.size() ? d.tank_mass_dots_kgps[i] : 0.0;
            };
            err.tank_mass_dots_kgps[i] =
                val(k1) * kE1 + val(k3) * kE3 + val(k4) * kE4 +
                val(k5) * kE5 + val(k6) * kE6 + val(k7) * kE7;
        }

        const double e_norm = error_norm(state, y_new, err, h, tolerances_);

        const double accept_threshold = 1.0;
        if (e_norm <= accept_threshold) {
            // Suggest next step. If e_norm == 0, multiplicative factor would
            // be infinite — clamp to facmax.
            const double factor = e_norm > 0.0
                ? std::min(kFacMax, std::max(kFacMin, kSafetyFactor * std::pow(1.0 / e_norm, kOrderInv)))
                : kFacMax;
            const double h_next = h * factor;

            // Event detection on the accepted step via Hermite interpolation
            // between (state @ t_s, k1) and (y_new @ t_s + h, k7).
            auto interp = [&](double theta) {
                return hermite_interpolate(state, y_new, k1, k7, h, theta);
            };
            const auto event = find_first_event(events, t_s, h, state, y_new, interp);

            StepResult res;
            if (event.has_value()) {
                res.state_end = event->state;
                res.t_end = event->t_s;
                res.h_used = event->t_s - t_s;
                res.event = event;
            } else {
                res.state_end = y_new;
                res.t_end = t_s + h;
                res.h_used = h;
            }
            res.h_next_suggested = h_next;
            res.accepted = true;
            return res;
        }

        // Step rejected: shrink and retry.
        const double factor = std::max(kFacMin, kSafetyFactor * std::pow(1.0 / e_norm, kOrderInv));
        const double h_new = h * factor;
        // Guard against zero-progress
        if (h_new < 1.0e-15) {
            StepResult res;
            res.state_end = y_new;
            res.t_end = t_s + h;
            res.h_used = h;
            res.h_next_suggested = h;
            res.accepted = true;  // give up adapting; emit the result
            return res;
        }
        h = h_new;
    }

    // Exhausted retries: emit the last attempted state as a "best effort"
    // accepted step. This avoids hanging the integrator on hyper-stiff
    // problems we never expect in our domain.
    StepResult res;
    res.state_end = state;
    res.t_end = t_s;
    res.h_used = 0.0;
    res.h_next_suggested = h;
    res.accepted = false;
    return res;
}

// =========================================================================
// Rk4IntegratorAdapter — fixed-step RK4 with linear event detection
// =========================================================================

StepResult Rk4IntegratorAdapter::step(
    const ExtendedState& state,
    double t_s,
    double h_suggested,
    const DynamicsFunction& dynamics,
    const std::vector<EventFunction>& events)
{
    const double h = h_suggested;
    Rk4OdeIntegrator integrator(OdeIntegratorOptions{h});
    const ExtendedState y_new = integrator.step(state, t_s, h, dynamics);

    auto interp = [&](double theta) {
        // Linear interpolation between (state, y_new) - rough but consistent
        // with RK4 being a coarse 4th-order solver: position/velocity scale
        // doesn't justify a fancier interpolant in event detection here.
        ExtendedState out;
        out.motion.position_m =
            state.motion.position_m * (1.0 - theta) + y_new.motion.position_m * theta;
        out.motion.velocity_mps =
            state.motion.velocity_mps * (1.0 - theta) + y_new.motion.velocity_mps * theta;
        out.tank_masses_kg.resize(state.tank_masses_kg.size());
        for (std::size_t i = 0; i < out.tank_masses_kg.size(); ++i) {
            const double y0 = state.tank_masses_kg[i];
            const double y1 = i < y_new.tank_masses_kg.size() ? y_new.tank_masses_kg[i] : y0;
            out.tank_masses_kg[i] = y0 * (1.0 - theta) + y1 * theta;
        }
        return out;
    };
    const auto event = find_first_event(events, t_s, h, state, y_new, interp);

    StepResult res;
    if (event.has_value()) {
        res.state_end = event->state;
        res.t_end = event->t_s;
        res.h_used = event->t_s - t_s;
        res.event = event;
    } else {
        res.state_end = y_new;
        res.t_end = t_s + h;
        res.h_used = h;
    }
    res.h_next_suggested = h;
    res.accepted = true;
    return res;
}

// =========================================================================
// Factory
// =========================================================================

std::unique_ptr<IIntegrator> make_integrator(
    const std::string& type,
    double /*step_s*/,
    const IntegratorTolerances& tolerances)
{
    if (type == "dopri5") {
        return std::make_unique<Dopri5Integrator>(tolerances);
    }
    if (type == "rk4" || type == "ode") {
        return std::make_unique<Rk4IntegratorAdapter>();
    }
    return std::make_unique<Rk4IntegratorAdapter>();
}

} // namespace post2::integrators
