#include "post2/core/reentry_solver.hpp"

#include "post2/aero/aero_model.hpp"
#include "post2/environment/atmosphere.hpp"

#ifdef POST2_HAVE_CLARABEL
// The vendored Clarabel C headers have no extern "C" guard, so wrap them to get
// C linkage (otherwise the C++ compiler mangles the names and the import lib's
// plain-C exports don't resolve).
extern "C" {
#include <clarabel.h>
}

#include <cstdint>
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace post2::core {

bool clarabel_available()
{
#ifdef POST2_HAVE_CLARABEL
    return true;
#else
    return false;
#endif
}

bool clarabel_selftest(double* x_out)
{
#ifdef POST2_HAVE_CLARABEL
    // Cost 1/2 x'Px + q'x with P = [2], q = [0] is x^2; minimizing it subject to
    // x >= 1 gives x* = 1. Clarabel form: min 1/2 x'Px + q'x s.t. Ax + s = b,
    // s in K. The constraint x >= 1 becomes -x + s = -1, s >= 0 (NonnegativeCone).
    const std::uintptr_t p_colptr[] = {0, 1};
    const std::uintptr_t p_rowval[] = {0};
    const double p_nzval[] = {2.0};
    ClarabelCscMatrix P;
    clarabel_CscMatrix_init(&P, 1, 1, p_colptr, p_rowval, p_nzval);

    const double q[] = {0.0};

    const std::uintptr_t a_colptr[] = {0, 1};
    const std::uintptr_t a_rowval[] = {0};
    const double a_nzval[] = {-1.0};
    ClarabelCscMatrix A;
    clarabel_CscMatrix_init(&A, 1, 1, a_colptr, a_rowval, a_nzval);

    const double b[] = {-1.0};

    // One nonnegative cone of size 1 (constructed without the C compound-literal
    // macro so it compiles as C++).
    ClarabelSupportedConeT cone;
    cone.tag = ClarabelNonnegativeConeT_Tag;
    cone.nonnegative_cone_t = 1;
    ClarabelSupportedConeT cones[] = {cone};

    ClarabelDefaultSettings settings = clarabel_DefaultSettings_default();
    settings.verbose = false;

    ClarabelDefaultSolver* solver =
        clarabel_DefaultSolver_new(&P, q, &A, b, 1, cones, &settings);
    if (solver == nullptr) {
        return false;
    }
    clarabel_DefaultSolver_solve(solver);
    const ClarabelDefaultSolution solution = clarabel_DefaultSolver_solution(solver);

    const bool ok = solution.status == ClarabelSolved;
    if (ok && x_out != nullptr && solution.x != nullptr) {
        *x_out = solution.x[0];
    }
    clarabel_DefaultSolver_free(solver);
    return ok;
#else
    (void)x_out;
    return false;
#endif
}

#ifdef POST2_HAVE_CLARABEL
namespace {

constexpr double kSuttonGravesK = 1.7415e-4;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};
double norm2(const Vec2& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

// One frozen linearisation node: the reference state + the quantities that are
// held constant inside the SOCP for that time step.
struct RefNode {
    double altitude_m = 0.0;
    Vec2 vel;
    double v_ceil = 0.0;  // max |v| keeping both loads under limit (*margin)
    Vec2 grav;            // gravity acceleration (frozen)
    Vec2 drag;            // drag acceleration (frozen at reference v)
    double a_max = 0.0;   // max thrust acceleration = max_thrust / mass
    double mass = 0.0;
};

struct Reference {
    std::vector<RefNode> nodes;
    double peak_q_pa = 0.0;
    double peak_hf_wpm2 = 0.0;
};

// Velocity ceiling: the largest |v| that keeps q and heat flux under their
// (margined) limits at this density. +inf when both limits are disabled.
double velocity_ceiling(double rho, const EntryBurnProblem& p)
{
    double v_ceil = std::numeric_limits<double>::infinity();
    const double scale = std::max(0.0, 1.0 - p.safety_margin);
    if (p.q_limit_pa > 0.0 && rho > 0.0) {
        v_ceil = std::min(v_ceil, std::sqrt(2.0 * p.q_limit_pa * scale / rho));
    }
    if (p.heat_flux_limit_wpm2 > 0.0 && rho > 0.0 && p.nose_radius_m > 0.0) {
        const double denom = kSuttonGravesK * std::sqrt(rho / p.nose_radius_m);
        if (denom > 0.0) {
            v_ceil = std::min(v_ceil, std::cbrt(p.heat_flux_limit_wpm2 * scale / denom));
        }
    }
    return v_ceil;
}

// Forward-integrate the descent (real gravity + drag + the given thrust-accel
// profile) to produce the reference nodes and the peak loads along it.
Reference predict(const EntryBurnProblem& p, const std::vector<Vec2>& u_profile, int n_nodes)
{
    Reference ref;
    Vec2 pos{0.0, p.altitude_m};
    Vec2 vel{p.speed_mps * std::cos(p.flight_path_angle_rad),
             -p.speed_mps * std::sin(p.flight_path_angle_rad)};
    double mass = std::max(1.0, p.mass_kg);
    for (int k = 0; k < n_nodes; ++k) {
        if (pos.y < 0.0) {
            break;
        }
        const auto atm = post2::environment::us_standard_1976(pos.y);
        const double rho = atm.density_kgpm3;
        const double speed = norm2(vel);
        const double q = 0.5 * rho * speed * speed;
        const double hf = post2::aero::stagnation_heat_flux_wpm2(rho, speed, p.nose_radius_m);
        ref.peak_q_pa = std::max(ref.peak_q_pa, q);
        ref.peak_hf_wpm2 = std::max(ref.peak_hf_wpm2, hf);

        RefNode node;
        node.altitude_m = pos.y;
        node.vel = vel;
        node.v_ceil = velocity_ceiling(rho, p);
        const double g_mag = p.earth_mu_m3s2 /
            ((p.earth_radius_m + pos.y) * (p.earth_radius_m + pos.y));
        node.grav = {0.0, -g_mag};
        const double drag_coef = (mass > 0.0)
            ? 0.5 * rho * speed * p.drag_cd * p.ref_area_m2 / mass : 0.0;
        node.drag = {-drag_coef * vel.x, -drag_coef * vel.y};
        node.a_max = (mass > 0.0) ? p.max_thrust_n / mass : 0.0;
        node.mass = mass;
        ref.nodes.push_back(node);

        const Vec2 u = (k < static_cast<int>(u_profile.size())) ? u_profile[k] : Vec2{0.0, 0.0};
        const Vec2 acc{node.grav.x + node.drag.x + u.x, node.grav.y + node.drag.y + u.y};
        vel = {vel.x + acc.x * p.dt_s, vel.y + acc.y * p.dt_s};
        pos = {pos.x + vel.x * p.dt_s, pos.y + vel.y * p.dt_s};
        const double thrust_force = norm2(u) * mass;
        if (p.exhaust_velocity_mps > 0.0) {
            mass -= (thrust_force / p.exhaust_velocity_mps) * p.dt_s;
        }
        mass = std::max(1.0, mass);
    }
    return ref;
}

struct Triplet {
    int row = 0;
    int col = 0;
    double val = 0.0;
};

// Solves one frozen-atmosphere SOCP around `ref`, returning the thrust-accel
// profile (size = ref.nodes.size()). Returns false on solver failure.
bool solve_socp(const EntryBurnProblem& p, const Reference& ref, std::vector<Vec2>* u_out)
{
    const int n_nodes = static_cast<int>(ref.nodes.size());
    if (n_nodes < 2) {
        return false;
    }
    const int n_vel = n_nodes + 1;             // v[0..n_nodes]
    const int base_u = 2 * n_vel;              // u[0..n_nodes-1]
    const int base_t = base_u + 2 * n_nodes;   // t[0..n_nodes-1]
    const int n_var = base_t + n_nodes;        // 5*n_nodes + 2

    const auto vi = [&](int k, int c) { return 2 * k + c; };
    const auto ui = [&](int k, int c) { return base_u + 2 * k + c; };
    const auto ti = [&](int k) { return base_t + k; };

    const Vec2 v_init{p.speed_mps * std::cos(p.flight_path_angle_rad),
                      -p.speed_mps * std::sin(p.flight_path_angle_rad)};
    const double dt = p.dt_s;

    std::vector<Triplet> trips;
    std::vector<double> b;
    std::vector<ClarabelSupportedConeT> cones;
    int row = 0;
    const auto coef = [&](int r, int c, double v) { trips.push_back({r, c, v}); };

    // --- equality block (ZeroCone): Ax = b ---
    coef(row, vi(0, 0), 1.0); b.push_back(v_init.x); ++row;
    coef(row, vi(0, 1), 1.0); b.push_back(v_init.y); ++row;
    for (int k = 0; k < n_nodes; ++k) {
        const RefNode& nd = ref.nodes[k];
        for (int c = 0; c < 2; ++c) {
            coef(row, vi(k + 1, c), 1.0);
            coef(row, vi(k, c), -1.0);
            coef(row, ui(k, c), -dt);
            const double drift = (c == 0 ? nd.grav.x + nd.drag.x : nd.grav.y + nd.drag.y);
            b.push_back(drift * dt);
            ++row;
        }
    }
    {
        ClarabelSupportedConeT zero;
        zero.tag = ClarabelZeroConeT_Tag;
        zero.zero_cone_t = static_cast<std::uintptr_t>(row);
        cones.push_back(zero);
    }

    const auto add_soc3 = [&](double s0_const, int x_var, int y_var) {
        // s = (s0_const, value(x_var), value(y_var)) in SOC3.
        b.push_back(s0_const); ++row;            // s0 (constant when no coef)
        coef(row, x_var, -1.0); b.push_back(0.0); ++row;
        coef(row, y_var, -1.0); b.push_back(0.0); ++row;
        ClarabelSupportedConeT soc;
        soc.tag = ClarabelSecondOrderConeT_Tag;
        soc.second_order_cone_t = 3;
        cones.push_back(soc);
    };
    const auto add_soc3_var0 = [&](int s0_var, int x_var, int y_var) {
        coef(row, s0_var, -1.0); b.push_back(0.0); ++row;  // s0 = value(s0_var)
        coef(row, x_var, -1.0); b.push_back(0.0); ++row;
        coef(row, y_var, -1.0); b.push_back(0.0); ++row;
        ClarabelSupportedConeT soc;
        soc.tag = ClarabelSecondOrderConeT_Tag;
        soc.second_order_cone_t = 3;
        cones.push_back(soc);
    };

    // Velocity ceiling on the interior nodes: |v[k]| <= v_ceil_k.
    for (int k = 1; k < n_nodes; ++k) {
        double ceil = ref.nodes[k].v_ceil;
        if (!std::isfinite(ceil)) {
            ceil = 1.0e9;  // disabled limit -> effectively unbounded
        }
        add_soc3(ceil, vi(k, 0), vi(k, 1));
    }
    // Thrust magnitude bound: |u[k]| <= a_max_k.
    for (int k = 0; k < n_nodes; ++k) {
        add_soc3(ref.nodes[k].a_max, ui(k, 0), ui(k, 1));
    }
    // Fuel epigraph: t[k] >= |u[k]|.
    for (int k = 0; k < n_nodes; ++k) {
        add_soc3_var0(ti(k), ui(k, 0), ui(k, 1));
    }

    const int n_con = row;

    // --- cost: minimize sum_k t[k] * mass_k * dt  (proportional to impulse) ---
    std::vector<double> q(n_var, 0.0);
    for (int k = 0; k < n_nodes; ++k) {
        q[ti(k)] = ref.nodes[k].mass * dt;
    }

    // --- assemble A in CSC (column-major) from the triplets ---
    std::vector<std::uintptr_t> a_colptr(n_var + 1, 0);
    for (const auto& t : trips) {
        ++a_colptr[t.col + 1];
    }
    for (int c = 0; c < n_var; ++c) {
        a_colptr[c + 1] += a_colptr[c];
    }
    std::vector<std::uintptr_t> a_rowval(trips.size());
    std::vector<double> a_nzval(trips.size());
    std::vector<std::uintptr_t> fill(a_colptr.begin(), a_colptr.end());
    for (const auto& t : trips) {
        const std::uintptr_t dst = fill[t.col]++;
        a_rowval[dst] = static_cast<std::uintptr_t>(t.row);
        a_nzval[dst] = t.val;
    }

    ClarabelCscMatrix A;
    clarabel_CscMatrix_init(&A, n_con, n_var, a_colptr.data(), a_rowval.data(), a_nzval.data());

    // Empty quadratic cost P (n_var x n_var, no nonzeros).
    std::vector<std::uintptr_t> p_colptr(n_var + 1, 0);
    ClarabelCscMatrix P;
    clarabel_CscMatrix_init(&P, n_var, n_var, p_colptr.data(), nullptr, nullptr);

    ClarabelDefaultSettings settings = clarabel_DefaultSettings_default();
    settings.verbose = false;
    settings.max_iter = 200;

    ClarabelDefaultSolver* solver = clarabel_DefaultSolver_new(
        &P, q.data(), &A, b.data(),
        static_cast<std::uintptr_t>(cones.size()), cones.data(), &settings);
    if (solver == nullptr) {
        return false;
    }
    clarabel_DefaultSolver_solve(solver);
    const ClarabelDefaultSolution sol = clarabel_DefaultSolver_solution(solver);
    const bool solved = (sol.status == ClarabelSolved || sol.status == ClarabelAlmostSolved) &&
                        sol.x != nullptr;
    if (solved) {
        u_out->assign(n_nodes, Vec2{0.0, 0.0});
        for (int k = 0; k < n_nodes; ++k) {
            (*u_out)[k] = {sol.x[ui(k, 0)], sol.x[ui(k, 1)]};
        }
    }
    clarabel_DefaultSolver_free(solver);
    return solved;
}

}  // namespace
#endif  // POST2_HAVE_CLARABEL

bool solve_entry_burn(const EntryBurnProblem& p, EntryBurnSolution* solution)
{
    if (solution == nullptr) {
        return false;
    }
    *solution = {};
#ifdef POST2_HAVE_CLARABEL
    const int n_nodes = std::max(4, std::min(p.max_nodes, 400));

    // Ballistic reference: if it already respects the limits, no burn needed.
    Reference ref = predict(p, {}, n_nodes);
    const bool over_q = p.q_limit_pa > 0.0 && ref.peak_q_pa > p.q_limit_pa;
    const bool over_hf = p.heat_flux_limit_wpm2 > 0.0 && ref.peak_hf_wpm2 > p.heat_flux_limit_wpm2;
    if (!over_q && !over_hf) {
        solution->ok = true;
        solution->burn_needed = false;
        solution->predicted_peak_q_pa = ref.peak_q_pa;
        solution->predicted_peak_heat_flux_wpm2 = ref.peak_hf_wpm2;
        return true;
    }

    // Successive convexification: re-linearise the frozen atmosphere/drag around
    // the latest solved trajectory a few times.
    std::vector<Vec2> u_profile;
    bool any_solved = false;
    for (int it = 0; it < std::max(1, p.scvx_iterations); ++it) {
        std::vector<Vec2> u_new;
        if (!solve_socp(p, ref, &u_new)) {
            break;
        }
        any_solved = true;
        u_profile = u_new;
        ref = predict(p, u_profile, static_cast<int>(ref.nodes.size()));
    }
    if (!any_solved) {
        return false;
    }

    // Extract the burn: per-node thrust force and the contiguous window where it
    // is significant (> 1% of max thrust).
    const std::size_t nn = u_profile.size();
    solution->thrust_profile_n.assign(nn, 0.0);
    double max_thrust = 0.0;
    for (std::size_t k = 0; k < nn; ++k) {
        const double f = norm2(u_profile[k]) * ref.nodes[k].mass;
        solution->thrust_profile_n[k] = f;
        max_thrust = std::max(max_thrust, f);
    }
    const double on_threshold = std::max(1.0, 0.01 * p.max_thrust_n);
    int first = -1;
    int last = -1;
    for (std::size_t k = 0; k < nn; ++k) {
        if (solution->thrust_profile_n[k] > on_threshold) {
            if (first < 0) first = static_cast<int>(k);
            last = static_cast<int>(k);
        }
    }
    solution->ok = true;
    solution->burn_needed = true;
    solution->predicted_peak_q_pa = ref.peak_q_pa;
    solution->predicted_peak_heat_flux_wpm2 = ref.peak_hf_wpm2;
    if (first >= 0) {
        solution->ignition_delay_s = first * p.dt_s;
        solution->burn_duration_s = (last - first + 1) * p.dt_s;
        solution->thrust_n = std::min(p.max_thrust_n, max_thrust);
    }
    return true;
#else
    return false;
#endif
}

} // namespace post2::core
