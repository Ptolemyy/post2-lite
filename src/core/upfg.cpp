#include "post2/core/upfg.hpp"

#include <algorithm>
#include <cmath>

namespace post2::core {

namespace {

constexpr double kStandardGravityMps2 = 9.80665;
constexpr double kDegToRad = 0.017453292519943295769;

double clampd(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

Vec3 unit_or(const Vec3& v, const Vec3& fallback)
{
    const double n = post2::vehicle::norm(v);
    if (!(n > 1.0e-12) || !std::isfinite(n)) {
        return fallback;
    }
    return v / n;
}

bool finite(const Vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Rotate vector v about unit-ish axis by `angle_rad` (Rodrigues' formula).
Vec3 rodrigues(const Vec3& v, const Vec3& axis, double angle_rad)
{
    const Vec3 k = unit_or(axis, {0.0, 0.0, 1.0});
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return v * c + cross(k, v) * s + k * (post2::vehicle::dot(k, v) * (1.0 - c));
}

// Stumpff functions C(psi) and S(psi), with series expansions near psi = 0.
double stumpff_c2(double psi)
{
    if (psi > 1.0e-6) {
        const double s = std::sqrt(psi);
        return (1.0 - std::cos(s)) / psi;
    }
    if (psi < -1.0e-6) {
        const double s = std::sqrt(-psi);
        return (1.0 - std::cosh(s)) / psi;
    }
    return 1.0 / 2.0 - psi / 24.0 + psi * psi / 720.0;
}

double stumpff_c3(double psi)
{
    if (psi > 1.0e-6) {
        const double s = std::sqrt(psi);
        return (s - std::sin(s)) / (s * s * s);
    }
    if (psi < -1.0e-6) {
        const double s = std::sqrt(-psi);
        return (std::sinh(s) - s) / (s * s * s);
    }
    return 1.0 / 6.0 - psi / 120.0 + psi * psi / 5040.0;
}

} // namespace

// -----------------------------------------------------------------------------
// Conic (two-body) state extrapolation -- universal-variable Kepler propagation.
// -----------------------------------------------------------------------------
ConicState conic_state_extrapolate(const Vec3& r0, const Vec3& v0, double dt, double mu)
{
    ConicState out;
    out.position_m = r0;
    out.velocity_mps = v0;

    const double r0mag = post2::vehicle::norm(r0);
    if (!(r0mag > 0.0) || !(mu > 0.0)) {
        return out;
    }
    if (std::abs(dt) < 1.0e-12) {
        out.ok = true;
        return out;
    }

    const double sqrtmu = std::sqrt(mu);
    const double rv = post2::vehicle::dot(r0, v0);
    const double v0sq = post2::vehicle::dot(v0, v0);
    const double alpha = 2.0 / r0mag - v0sq / mu;  // reciprocal of semi-major axis

    // Initial guess for the universal anomaly chi.
    double chi;
    if (alpha > 1.0e-9) {  // ellipse
        chi = sqrtmu * dt * alpha;
    } else if (alpha < -1.0e-9) {  // hyperbola
        const double a = 1.0 / alpha;
        const double sign = (dt >= 0.0) ? 1.0 : -1.0;
        const double num = -2.0 * mu * alpha * dt;
        const double den = rv + sign * std::sqrt(-mu * a) * (1.0 - r0mag * alpha);
        chi = (std::abs(den) > 1.0e-12) ? sign * std::sqrt(-a) * std::log(num / den)
                                        : sqrtmu * dt / r0mag;
    } else {  // near-parabolic
        chi = sqrtmu * dt / r0mag;
    }

    double r = r0mag;
    double psi = 0.0;
    double c2 = 0.5;
    double c3 = 1.0 / 6.0;
    for (int i = 0; i < 200; ++i) {
        psi = chi * chi * alpha;
        c2 = stumpff_c2(psi);
        c3 = stumpff_c3(psi);
        r = chi * chi * c2 + (rv / sqrtmu) * chi * (1.0 - psi * c3) + r0mag * (1.0 - psi * c2);
        if (!(std::abs(r) > 1.0e-9)) {
            r = (r >= 0.0) ? 1.0e-9 : -1.0e-9;
        }
        const double g_minus_t =
            chi * chi * chi * c3 + (rv / sqrtmu) * chi * chi * c2 + r0mag * chi * (1.0 - psi * c3);
        const double dchi = (sqrtmu * dt - g_minus_t) / r;
        chi += dchi;
        if (std::abs(dchi) < 1.0e-9) {
            break;
        }
        if (!std::isfinite(chi)) {
            break;
        }
    }

    psi = chi * chi * alpha;
    c2 = stumpff_c2(psi);
    c3 = stumpff_c3(psi);

    const double f = 1.0 - (chi * chi / r0mag) * c2;
    const double g = dt - (chi * chi * chi / sqrtmu) * c3;
    const Vec3 pos = r0 * f + v0 * g;
    const double rmag = post2::vehicle::norm(pos);
    if (!(rmag > 0.0)) {
        out.position_m = r0 + v0 * dt;
        out.velocity_mps = v0;
        return out;
    }
    const double gdot = 1.0 - (chi * chi / rmag) * c2;
    const double fdot = (sqrtmu / (rmag * r0mag)) * chi * (psi * c3 - 1.0);
    const Vec3 vel = r0 * fdot + v0 * gdot;

    if (!finite(pos) || !finite(vel)) {
        // Fall back to a straight-line coast rather than emit NaNs.
        out.position_m = r0 + v0 * dt;
        out.velocity_mps = v0;
        return out;
    }
    out.position_m = pos;
    out.velocity_mps = vel;
    out.ok = true;
    return out;
}

// -----------------------------------------------------------------------------
// Cold-start internal state.
// -----------------------------------------------------------------------------
UpfgInternal upfg_cold_init(const UpfgTarget& target, const UpfgVehicleState& state, double mu)
{
    UpfgInternal p;
    const Vec3 r = state.position_m;
    const Vec3 v = state.velocity_mps;
    const double rmag = post2::vehicle::norm(r);

    // Orbit angular-momentum direction is the negation of the UPFG plane normal.
    const Vec3 n = target.plane_normal * -1.0;

    // Desired cutoff position guess: straight up at the target radius. (A fixed
    // downrange rotation overshoots when time-to-go is short and seeds a spurious
    // radial velocity-to-go; block 8 walks rd downrange to the prediction over
    // the first few cycles.)
    p.rd = unit_or(r, {1.0, 0.0, 0.0}) * target.radius_m;

    // Velocity-to-be-gained guess: target speed along prograde at the CURRENT
    // position, minus current velocity.
    const Vec3 prograde = unit_or(cross(n, r), unit_or(v, {0.0, 1.0, 0.0}));
    p.vgo = prograde * target.velocity_mps - v;

    p.rbias = {0.0, 0.0, 0.0};
    // Seed gravity term; refined by the conic extrapolation each pass.
    p.rgrav = (rmag > 0.0) ? r * (-mu / (2.0 * rmag * rmag * rmag)) : Vec3{0.0, 0.0, 0.0};
    p.tgo_s = 0.0;
    p.tb_s = 0.0;
    p.time_s = state.time_s;
    p.v_prev = v;
    p.initialized = true;
    return p;
}

// -----------------------------------------------------------------------------
// One UPFG pass.
// -----------------------------------------------------------------------------
UpfgResult upfg_step(
    const std::vector<UpfgStage>& stages_in,
    const UpfgTarget& target,
    const UpfgVehicleState& state,
    const UpfgInternal& previous,
    double mu)
{
    UpfgResult res;
    res.next = previous;

    std::vector<UpfgStage> stages = stages_in;
    int n = static_cast<int>(stages.size());
    if (n == 0) {
        res.thrust_unit_eci = unit_or(state.velocity_mps, unit_or(state.position_m, {1.0, 0.0, 0.0}));
        return res;
    }

    const Vec3 iy = target.plane_normal;
    const double rdval = target.radius_m;
    const double vdval = target.velocity_mps;
    const double gamma = target.flight_path_angle_rad;
    const double t = state.time_s;
    const double m = state.mass_kg;
    const Vec3 r = state.position_m;
    const Vec3 v = state.velocity_mps;

    Vec3 rbias = previous.rbias;
    Vec3 rd = previous.rd;
    Vec3 rgrav = previous.rgrav;
    Vec3 vgo = previous.vgo;

    // --- Block 1: per-stage parameters --------------------------------------
    std::vector<int> SM(n);
    std::vector<double> aL(n), ve(n), fT(n), aT(n), tu(n), tb(n);
    for (int i = 0; i < n; ++i) {
        SM[i] = stages[i].mode;
        aL[i] = stages[i].accel_limit_mps2;
        fT[i] = stages[i].thrust_n;
        ve[i] = stages[i].exhaust_velocity_mps;
        aT[i] = (stages[i].mass_total_kg > 0.0) ? fT[i] / stages[i].mass_total_kg : 0.0;
        tu[i] = (aT[i] > 0.0) ? ve[i] / aT[i] : 0.0;
        tb[i] = stages[i].max_burn_time_s;
    }

    // --- Block 2: remove the velocity sensed since the previous cycle -------
    const double dt = t - previous.time_s;
    const Vec3 dvsensed = v - previous.v_prev;
    vgo = vgo - dvsensed;
    tb[0] = tb[0] - previous.tb_s;

    // --- Block 3: per-stage delta-V split and time-to-go --------------------
    if (SM[0] == 1) {
        aT[0] = (m > 0.0) ? fT[0] / m : 0.0;
    } else if (SM[0] == 2) {
        aT[0] = aL[0];
    }
    tu[0] = (aT[0] > 0.0) ? ve[0] / aT[0] : 0.0;

    const double vgo_mag = post2::vehicle::norm(vgo);
    std::vector<double> Li(n, 0.0);
    double L = 0.0;
    for (int i = 0; i < n - 1; ++i) {
        if (SM[i] == 1) {
            const double denom = tu[i] - tb[i];
            Li[i] = (denom > 0.0 && tu[i] > 0.0) ? ve[i] * std::log(tu[i] / denom) : 0.0;
        } else if (SM[i] == 2) {
            Li[i] = aL[i] * tb[i];
        }
        L += Li[i];
        if (L > vgo_mag) {
            // Earlier stages already provide the full velocity-to-go: cutoff
            // happens before the last stage, so drop it and recompute.
            stages.pop_back();
            return upfg_step(stages, target, state, previous, mu);
        }
    }
    Li[n - 1] = vgo_mag - L;

    std::vector<double> tgoi(n, 0.0);
    for (int i = 0; i < n; ++i) {
        if (SM[i] == 1) {
            tb[i] = (ve[i] > 0.0) ? tu[i] * (1.0 - std::exp(-Li[i] / ve[i])) : 0.0;
        } else if (SM[i] == 2) {
            tb[i] = (aL[i] > 0.0) ? Li[i] / aL[i] : 0.0;
        }
        tgoi[i] = (i == 0) ? tb[i] : tgoi[i - 1] + tb[i];
    }
    const double tgo = tgoi[n - 1];

    // --- Block 4: thrust integrals (L, J, S, Q, P, H) -----------------------
    L = 0.0;
    double J = 0.0, S = 0.0, Q = 0.0, P = 0.0, H = 0.0;
    double tgoi1 = 0.0;
    for (int i = 0; i < n; ++i) {
        if (i > 0) {
            tgoi1 = tgoi[i - 1];
        }
        double Ji = 0.0, Si = 0.0, Qi = 0.0, Pi = 0.0;
        if (SM[i] == 1) {
            Ji = tu[i] * Li[i] - ve[i] * tb[i];
            Si = -Ji + tb[i] * Li[i];
            Qi = Si * (tu[i] + tgoi1) - 0.5 * ve[i] * tb[i] * tb[i];
            Pi = Qi * (tu[i] + tgoi1) - 0.5 * ve[i] * tb[i] * tb[i] * (tb[i] / 3.0 + tgoi1);
        } else if (SM[i] == 2) {
            Ji = 0.5 * Li[i] * tb[i];
            Si = Ji;
            Qi = Si * (tb[i] / 3.0 + tgoi1);
            Pi = (1.0 / 6.0) * Si * (tgoi[i] * tgoi[i] + 2.0 * tgoi[i] * tgoi1 + 3.0 * tgoi1 * tgoi1);
        }
        // Shift each per-stage integral by the burn time already accumulated.
        Ji += Li[i] * tgoi1;
        Si += L * tb[i];
        Qi += J * tb[i];
        Pi += H * tb[i];

        L += Li[i];
        J += Ji;
        S += Si;
        Q += Qi;
        P += Pi;
        H = J * tgoi[i] - Q;
    }

    const double safeL = (std::abs(L) > 1.0e-9) ? L : 1.0e-9;

    // --- Block 5: steering (thrust direction and turning rate) --------------
    const Vec3 lambda = unit_or(vgo, unit_or(v, {1.0, 0.0, 0.0}));
    if (previous.tgo_s > 0.0) {
        const double sc = tgo / previous.tgo_s;
        rgrav = rgrav * (sc * sc);
    }
    Vec3 rgo = rd - (r + v * tgo + rgrav);
    const Vec3 iz = unit_or(cross(rd, iy), {0.0, 0.0, 1.0});
    const Vec3 rgoxy = rgo - iz * post2::vehicle::dot(iz, rgo);
    const double denom_z = post2::vehicle::dot(lambda, iz);
    const double rgoz =
        (std::abs(denom_z) > 1.0e-9) ? (S - post2::vehicle::dot(lambda, rgoxy)) / denom_z : 0.0;
    rgo = rgoxy + iz * rgoz + rbias;

    const double lambdade = Q - S * J / safeL;
    const Vec3 lambdadot =
        (std::abs(lambdade) > 1.0e-9) ? (rgo - lambda * S) / lambdade : Vec3{0.0, 0.0, 0.0};
    const double JL = J / safeL;
    Vec3 iF = unit_or(lambda - lambdadot * JL, lambda);

    // Limit the commanded turn from lambda. UPFG's converged solution turns only
    // a few degrees; a large angle means an unconverged cycle (e.g. a cold start
    // whose rd is still far from the prediction). Clamping keeps the thrust
    // integrals (which expand in phi) valid while rd/rbias converge over cycles,
    // and prevents the vthrust coefficient (1 - 0.5*phi^2 - ...) from flipping
    // sign and commanding retrograde thrust.
    constexpr double kMaxTurnRad = 0.6;
    double phi = std::acos(clampd(post2::vehicle::dot(iF, lambda), -1.0, 1.0));
    if (phi > kMaxTurnRad) {
        const Vec3 axis = cross(lambda, iF);
        if (post2::vehicle::norm(axis) > 1.0e-12) {
            iF = rodrigues(lambda, axis, kMaxTurnRad);
            phi = kMaxTurnRad;
        }
    }
    const double phidot = (std::abs(J) > 1.0e-9) ? -phi * L / J : 0.0;
    const Vec3 lambdadot_u = unit_or(lambdadot, {0.0, 0.0, 0.0});

    const Vec3 vthrust =
        lambda * (L - 0.5 * L * phi * phi - J * phi * phidot - 0.5 * H * phidot * phidot);
    const double rthrust_s = S - 0.5 * S * phi * phi - Q * phi * phidot - 0.5 * P * phidot * phidot;
    const Vec3 rthrust = lambda * rthrust_s - lambdadot_u * (S * phi + Q * phidot);

    const Vec3 vbias = vgo - vthrust;
    rbias = rgo - rthrust;

    // --- Block 7: gravity over the burn via conic state extrapolation -------
    Vec3 vgrav = {0.0, 0.0, 0.0};
    if (tgo > 1.0e-9) {
        const Vec3 rc1 = r - rthrust * 0.1 - vthrust * (tgo / 30.0);
        const Vec3 vc1 = v + rthrust * (1.2 / tgo) - vthrust * 0.1;
        const ConicState cs = conic_state_extrapolate(rc1, vc1, tgo, mu);
        rgrav = cs.position_m - rc1 - vc1 * tgo;
        vgrav = cs.velocity_mps - vc1;
    }

    // --- Block 8: update desired cutoff position rd and velocity-to-go vgo --
    Vec3 rp = r + v * tgo + rgrav + rthrust;
    rp = rp - iy * post2::vehicle::dot(rp, iy);  // project into the target plane
    rd = unit_or(rp, unit_or(r, {1.0, 0.0, 0.0})) * rdval;
    const Vec3 ix = unit_or(rd, {1.0, 0.0, 0.0});
    const Vec3 iz2 = cross(ix, iy);
    const Vec3 vd = (ix * std::sin(gamma) + iz2 * std::cos(gamma)) * vdval;
    vgo = vd - v - vgrav + vbias;

    // --- Assemble result ----------------------------------------------------
    res.thrust_unit_eci = iF;
    res.tgo_s = tgo;
    res.next.rbias = rbias;
    res.next.rd = rd;
    res.next.rgrav = rgrav;
    res.next.tgo_s = tgo;
    res.next.tb_s = previous.tb_s + dt;
    res.next.time_s = t;
    res.next.v_prev = v;
    res.next.vgo = vgo;
    res.next.initialized = true;
    res.ok = finite(iF) && std::isfinite(tgo) && tgo > 0.0;
    return res;
}

// -----------------------------------------------------------------------------
// Converge UPFG at the current state.
//
// Iterates upfg_step from a cold init at a fixed state. With the steering turn
// clamped (in upfg_step) the cycle map is contractive, so this settles to the
// self-consistent solution in a few passes -- giving a deterministic, smooth
// function of state suitable for embedding in an integrator (each derivative
// evaluation re-solves independently). A fixed iteration count keeps the result
// a smooth function of state (no tolerance-boundary kinks).
// -----------------------------------------------------------------------------
UpfgResult upfg_converge(
    const std::vector<UpfgStage>& stages,
    const UpfgTarget& target,
    const UpfgVehicleState& state,
    double mu,
    int iterations)
{
    UpfgInternal prev = upfg_cold_init(target, state, mu);
    UpfgResult res;
    const int iters = std::max(1, iterations);
    for (int i = 0; i < iters; ++i) {
        UpfgResult step = upfg_step(stages, target, state, prev, mu);
        if (!step.ok) {
            return res.ok ? res : step;
        }
        prev = step.next;
        res = step;
    }
    return res;
}

// -----------------------------------------------------------------------------
// Target setup from an apsis/inclination orbit spec.
// -----------------------------------------------------------------------------
UpfgTarget make_upfg_orbit_target(
    double periapsis_alt_m,
    double apoapsis_alt_m,
    double inclination_deg,
    const Vec3& position_m,
    const Vec3& velocity_mps,
    double mu,
    double body_radius_m)
{
    UpfgTarget t;

    double rp = body_radius_m + periapsis_alt_m;  // periapsis radius
    double ra = body_radius_m + apoapsis_alt_m;   // apoapsis radius
    if (ra < rp) {
        std::swap(ra, rp);
    }
    const double sma = 0.5 * (rp + ra);

    // Insert at the target periapsis: radius = rp, flight-path angle = 0.
    t.radius_m = rp;
    t.velocity_mps = (sma > 0.0 && rp > 0.0) ? std::sqrt(std::max(0.0, mu * (2.0 / rp - 1.0 / sma)))
                                             : 0.0;
    t.flight_path_angle_rad = 0.0;

    // Orbit-normal of the target plane: the plane that contains the current
    // position at the requested inclination. n.z = cos(inc); n perpendicular to
    // r; choose the in-plane sign that matches the current motion (prograde).
    const Vec3 rhat = unit_or(position_m, {1.0, 0.0, 0.0});
    const Vec3 zhat = {0.0, 0.0, 1.0};
    const double inc = inclination_deg * kDegToRad;
    const double rz = post2::vehicle::dot(rhat, zhat);
    const double coslat = std::sqrt(std::max(0.0, 1.0 - rz * rz));

    const Vec3 e1 = unit_or(zhat - rhat * rz, zhat);     // in plane perp r, toward pole
    const Vec3 e2 = unit_or(cross(e1, rhat), {0.0, 1.0, 0.0});  // perp r and e1

    const double cth = (coslat > 1.0e-9) ? clampd(std::cos(inc) / coslat, -1.0, 1.0) : 1.0;
    const double sth = std::sqrt(std::max(0.0, 1.0 - cth * cth));

    double sign = 1.0;
    const Vec3 hnow = cross(position_m, velocity_mps);
    if (post2::vehicle::norm(hnow) > 1.0e-3 &&
        post2::vehicle::dot(e2, unit_or(hnow, {0.0, 0.0, 1.0})) < 0.0) {
        sign = -1.0;
    }

    const Vec3 n = unit_or(e1 * cth + e2 * (sth * sign), zhat);  // orbit normal
    t.plane_normal = n * -1.0;  // UPFG iy convention
    return t;
}

} // namespace post2::core
