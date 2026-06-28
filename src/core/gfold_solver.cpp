#include "post2/core/gfold_solver.hpp"

#ifdef POST2_HAVE_CLARABEL
// The vendored Clarabel C headers have no extern "C" guard (see reentry_solver).
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

bool gfold_available()
{
#ifdef POST2_HAVE_CLARABEL
    return true;
#else
    return false;
#endif
}

#ifdef POST2_HAVE_CLARABEL
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Triplet {
    int row = 0;
    int col = 0;
    double val = 0.0;
};

// Builds and solves the lossless-convexification min-fuel SOCP for a fixed tf.
// Decision vector per node k (stride 8): [rx, ry, vx, vy, z=ln(m), ux, uy, sigma].
GfoldSolution solve_fixed_tf(const GfoldProblem& p, double tf, int n_nodes)
{
    GfoldSolution out;
    out.tf_s = tf;
    const int N = n_nodes;
    if (N < 2 || tf <= 0.0 || p.t_max_n <= 0.0 || p.exhaust_velocity_mps <= 0.0 ||
        p.m_wet <= 0.0) {
        return out;
    }

    const double dt = tf / (N - 1);
    const double alpha = 1.0 / p.exhaust_velocity_mps;  // mass flow per thrust [s/m]
    const double rho1 = p.min_throttle * p.t_max_n;     // min thrust [N]
    const double rho2 = p.max_throttle * p.t_max_n;     // max thrust [N]
    const double cosT = std::cos(p.max_tilt_deg * kPi / 180.0);
    const double tanG = std::tan(p.glide_slope_deg * kPi / 180.0);
    const double gy = -p.g0;

    // The Taylor linearisation of the mass dynamics needs the min-mass arc to
    // stay positive over the whole horizon, else ln() is undefined and the tf is
    // infeasible for this propellant load.
    if (p.m_wet - alpha * rho2 * tf <= 1.0) {
        return out;
    }

    const int S = 8;
    const int n = S * N;
    auto RX = [&](int k) { return S * k + 0; };
    auto RY = [&](int k) { return S * k + 1; };
    auto VX = [&](int k) { return S * k + 2; };
    auto VY = [&](int k) { return S * k + 3; };
    auto Z = [&](int k) { return S * k + 4; };
    auto UX = [&](int k) { return S * k + 5; };
    auto UY = [&](int k) { return S * k + 6; };
    auto SG = [&](int k) { return S * k + 7; };

    auto z0_at = [&](double t) { return std::log(p.m_wet - alpha * rho2 * t); };
    auto zU_at = [&](double t) { return std::log(p.m_wet - alpha * rho1 * t); };

    std::vector<Triplet> trip;
    std::vector<double> bvec;
    std::vector<ClarabelSupportedConeT> cones;
    int row = 0;
    auto addRow = [&](std::initializer_list<std::pair<int, double>> coeffs, double b) {
        for (const auto& c : coeffs) {
            trip.push_back({row, c.first, c.second});
        }
        bvec.push_back(b);
        ++row;
    };

    // ---- Section 1: equality constraints (ZeroCone) ----
    const int row_eq_begin = row;
    addRow({{RX(0), 1}}, p.r0x);
    addRow({{RY(0), 1}}, p.r0y);
    addRow({{VX(0), 1}}, p.v0x);
    addRow({{VY(0), 1}}, p.v0y);
    addRow({{Z(0), 1}}, std::log(p.m_wet));
    // Terminal: always land on the ground (ry = rfy) with zero velocity. Free
    // landing leaves the downrange touchdown point (rx) free; a fixed target
    // also pins rx = rfx.
    if (!p.free_landing) {
        addRow({{RX(N - 1), 1}}, p.rfx);
    }
    addRow({{RY(N - 1), 1}}, p.rfy);
    addRow({{VX(N - 1), 1}}, 0.0);
    addRow({{VY(N - 1), 1}}, 0.0);
    for (int k = 0; k < N - 1; ++k) {
        addRow({{RX(k + 1), 1}, {RX(k), -1}, {VX(k), -dt / 2}, {VX(k + 1), -dt / 2}}, 0.0);
        addRow({{RY(k + 1), 1}, {RY(k), -1}, {VY(k), -dt / 2}, {VY(k + 1), -dt / 2}}, 0.0);
        addRow({{VX(k + 1), 1}, {VX(k), -1}, {UX(k), -dt / 2}, {UX(k + 1), -dt / 2}}, 0.0);
        addRow({{VY(k + 1), 1}, {VY(k), -1}, {UY(k), -dt / 2}, {UY(k + 1), -dt / 2}}, dt * gy);
        addRow({{Z(k + 1), 1}, {Z(k), -1}, {SG(k), alpha * dt / 2}, {SG(k + 1), alpha * dt / 2}}, 0.0);
    }
    {
        ClarabelSupportedConeT zero;
        zero.tag = ClarabelZeroConeT_Tag;
        zero.zero_cone_t = static_cast<std::uintptr_t>(row - row_eq_begin);
        cones.push_back(zero);
    }

    // ---- Section 2: linear inequalities (NonnegativeCone, s >= 0) ----
    const int row_ineq_begin = row;
    for (int k = 0; k < N; ++k) {
        const double tk = k * dt;
        const double z0k = z0_at(tk);
        const double zUk = zU_at(tk);
        const double c2k = rho2 * std::exp(-z0k);
        addRow({{Z(k), c2k}, {SG(k), 1.0}}, c2k * (1.0 + z0k));     // upper thrust bound
        addRow({{UY(k), -1.0}, {SG(k), cosT}}, 0.0);               // thrust pointing
        // Glide slope. A fixed target anchors the cone at (rfx, rfy); a free
        // downrange landing cannot (the apex would be a variable), so it reduces
        // to staying above the ground, ry >= rfy.
        if (p.free_landing) {
            addRow({{RY(k), -1.0}}, -p.rfy);                       // ry >= rfy
        } else {
            addRow({{RY(k), -1.0}, {RX(k), tanG}}, -p.rfy + tanG * p.rfx);   // glide slope +
            addRow({{RY(k), -1.0}, {RX(k), -tanG}}, -p.rfy - tanG * p.rfx);  // glide slope -
        }
        addRow({{Z(k), -1.0}}, -z0k);                              // z >= z0k
        addRow({{Z(k), 1.0}}, zUk);                                // z <= zUk
    }
    addRow({{Z(N - 1), -1.0}}, -std::log(p.m_dry > 0.0 ? p.m_dry : 1.0));  // dry-mass floor
    {
        ClarabelSupportedConeT nn;
        nn.tag = ClarabelNonnegativeConeT_Tag;
        nn.nonnegative_cone_t = static_cast<std::uintptr_t>(row - row_ineq_begin);
        cones.push_back(nn);
    }

    // ---- Section 3: second-order cones ----
    auto push_soc3 = [&]() {
        ClarabelSupportedConeT soc;
        soc.tag = ClarabelSecondOrderConeT_Tag;
        soc.second_order_cone_t = 3;
        cones.push_back(soc);
    };
    for (int k = 0; k < N; ++k) {
        // Thrust magnitude: (sigma, ux, uy) in SOC3.
        addRow({{SG(k), -1.0}}, 0.0);
        addRow({{UX(k), -1.0}}, 0.0);
        addRow({{UY(k), -1.0}}, 0.0);
        push_soc3();

        // Lower thrust bound (2nd-order Taylor), rotated cone -> SOC3.
        const double tk = k * dt;
        const double z0k = z0_at(tk);
        const double mu0 = rho1 * std::exp(-z0k);
        addRow({{SG(k), -1.0}}, -mu0 / 2.0 + 1.0 / mu0);                 // c0
        addRow({{SG(k), -1.0}}, -mu0 / 2.0 - 1.0 / mu0);                 // c1
        addRow({{Z(k), -std::sqrt(2.0)}}, -std::sqrt(2.0) * (z0k + 1.0)); // c2
        push_soc3();
    }

    const int m = row;

    // ---- Assemble A in CSC (column-major) from the triplets ----
    std::vector<std::uintptr_t> a_colptr(n + 1, 0);
    for (const auto& t : trip) {
        ++a_colptr[t.col + 1];
    }
    for (int c = 0; c < n; ++c) {
        a_colptr[c + 1] += a_colptr[c];
    }
    std::vector<std::uintptr_t> a_rowval(trip.size());
    std::vector<double> a_nzval(trip.size());
    std::vector<std::uintptr_t> fill(a_colptr.begin(), a_colptr.end());
    for (const auto& t : trip) {
        const std::uintptr_t dst = fill[t.col]++;
        a_rowval[dst] = static_cast<std::uintptr_t>(t.row);
        a_nzval[dst] = t.val;
    }
    ClarabelCscMatrix A;
    clarabel_CscMatrix_init(&A, m, n, a_colptr.data(), a_rowval.data(), a_nzval.data());

    // Zero quadratic cost; linear cost maximises z_N (== minimise fuel).
    std::vector<std::uintptr_t> p_colptr(n + 1, 0);
    ClarabelCscMatrix Pmat;
    clarabel_CscMatrix_init(&Pmat, n, n, p_colptr.data(), nullptr, nullptr);
    std::vector<double> q(n, 0.0);
    q[Z(N - 1)] = -1.0;

    ClarabelDefaultSettings settings = clarabel_DefaultSettings_default();
    settings.verbose = false;
    settings.max_iter = 200;

    ClarabelDefaultSolver* solver = clarabel_DefaultSolver_new(
        &Pmat, q.data(), &A, bvec.data(),
        static_cast<std::uintptr_t>(cones.size()), cones.data(), &settings);
    if (solver == nullptr) {
        return out;
    }
    clarabel_DefaultSolver_solve(solver);
    const ClarabelDefaultSolution sol = clarabel_DefaultSolver_solution(solver);
    const bool solved = (sol.status == ClarabelSolved || sol.status == ClarabelAlmostSolved) &&
                        sol.x != nullptr;
    if (solved) {
        out.feasible = true;
        const double* x = sol.x;
        out.nodes.resize(N);
        for (int k = 0; k < N; ++k) {
            const double* xk = x + S * k;
            const double mass = std::exp(xk[4]);
            const double un = std::hypot(xk[5], xk[6]);
            GfoldNode node;
            node.t_s = k * dt;
            node.rx = xk[0];
            node.ry = xk[1];
            node.vx = xk[2];
            node.vy = xk[3];
            node.mass_kg = mass;
            node.ux = xk[5];
            node.uy = xk[6];
            node.thrust_n = mass * un;
            node.throttle = p.t_max_n > 0.0 ? node.thrust_n / p.t_max_n : 0.0;
            out.nodes[k] = node;
        }
        out.final_mass_kg = std::exp(x[Z(N - 1)]);
        out.fuel_used_kg = p.m_wet - out.final_mass_kg;
    }
    clarabel_DefaultSolver_free(solver);
    return out;
}

}  // namespace
#endif  // POST2_HAVE_CLARABEL

GfoldSolution gfold_solve_fixed_tf(const GfoldProblem& problem, double tf_s, int num_nodes)
{
#ifdef POST2_HAVE_CLARABEL
    return solve_fixed_tf(problem, tf_s, num_nodes);
#else
    (void)problem;
    (void)tf_s;
    (void)num_nodes;
    return GfoldSolution{};
#endif
}

GfoldSolution gfold_solve_optimal(
    const GfoldProblem& problem,
    int num_nodes,
    double tf_min_s,
    double tf_max_s)
{
#ifdef POST2_HAVE_CLARABEL
    if (tf_max_s <= tf_min_s) {
        return GfoldSolution{};
    }
    const double inf = std::numeric_limits<double>::infinity();
    auto fuel_at = [&](double tf) {
        const GfoldSolution r = solve_fixed_tf(problem, tf, num_nodes);
        return r.feasible ? r.fuel_used_kg : inf;
    };

    // Feasibility in tf is an INTERVAL, not a half-line: too short cannot
    // decelerate, too long exhausts the propellant (the full-throttle min-mass
    // arc goes non-positive). So coarse-scan to locate a feasible sample and
    // bracket the (unimodal) fuel minimum before refining.
    const int kScan = 24;
    double best_tf = -1.0;
    double best_fuel = inf;
    for (int i = 0; i < kScan; ++i) {
        const double tf = tf_min_s + (tf_max_s - tf_min_s) * i / (kScan - 1);
        const double f = fuel_at(tf);
        if (f < best_fuel) {
            best_fuel = f;
            best_tf = tf;
        }
    }
    if (best_tf < 0.0 || !std::isfinite(best_fuel)) {
        return GfoldSolution{};  // no feasible landing anywhere in range
    }

    // Golden-section within one scan-spacing of the best sample (the true minimum
    // lies in that bracket). Keep the best feasible point seen, so an infeasible
    // bracket endpoint never discards a good interior solution.
    const double spacing = (tf_max_s - tf_min_s) / (kScan - 1);
    double a = std::max(tf_min_s, best_tf - spacing);
    double b = std::min(tf_max_s, best_tf + spacing);
    double keep_tf = best_tf;
    double keep_fuel = best_fuel;
    auto track = [&](double tf, double f) {
        if (f < keep_fuel) {
            keep_fuel = f;
            keep_tf = tf;
        }
    };
    const double gr = 0.6180339887498949;
    double c = b - gr * (b - a), d = a + gr * (b - a);
    double fc = fuel_at(c), fd = fuel_at(d);
    track(c, fc);
    track(d, fd);
    for (int i = 0; i < 12; ++i) {
        if (fc < fd) {
            b = d; d = c; fd = fc; c = b - gr * (b - a); fc = fuel_at(c); track(c, fc);
        } else {
            a = c; c = d; fc = fd; d = a + gr * (b - a); fd = fuel_at(d); track(d, fd);
        }
    }
    return solve_fixed_tf(problem, keep_tf, num_nodes);
#else
    (void)problem;
    (void)num_nodes;
    (void)tf_min_s;
    (void)tf_max_s;
    return GfoldSolution{};
#endif
}

}  // namespace post2::core
