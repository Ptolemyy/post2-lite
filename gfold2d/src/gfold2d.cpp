// =============================================================================
//  gfold2d  --  2D fuel-optimal powered-descent guidance (GFOLD) via Clarabel
// -----------------------------------------------------------------------------
//  Implements the lossless-convexification soft-landing problem of
//
//    B. Acikmese, J. M. Carson, L. Blackmore,
//    "Lossless Convexification of Nonconvex Control Bound and Pointing
//     Constraints of the Soft Landing Optimal Control Problem,"
//    IEEE Trans. Control Systems Technology, 2013.
//    http://www.larsblackmore.com/iee_tcst13.pdf
//
//  The non-convex thrust *lower* bound (rho1 <= ||T|| <= rho2) is convexified
//  with the slack variable sigma >= ||u||, and the mass dynamics are made
//  linear with the change of variable z = ln(m).  The resulting second-order
//  cone program (Problem 3 -- minimum fuel) is solved with Clarabel.
//
//  This is a stand-alone study tool (NOT wired into post2-lite).  It reports
//  the per-solve time so the SOCP cost can be characterised, and it runs a
//  small sweep over the time-of-flight t_f (the one remaining non-convex
//  parameter) to recover the global fuel optimum, as GFOLD does in practice.
//
//  Decision vector, per discretisation node k = 0 .. N-1 (stride 8):
//      [ rx, ry, vx, vy, z, ux, uy, sigma ]
//  with  r = position [m]  (ry = altitude, up positive)
//        v = velocity [m/s]
//        z = ln(mass)
//        u = thrust acceleration T/m [m/s^2]
//        sigma >= ||u||  (convexified throttle slack, = Gamma/m)
//
//  Clarabel solves   min 1/2 x'Px + q'x   s.t.  A x + s = b,  s in K.
//  Here P = 0 (the objective is linear: maximise z_N == minimise fuel).
// =============================================================================

#include <clarabel.hpp>
#include <Eigen/Eigen>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <vector>

using namespace clarabel;
using Eigen::SparseMatrix;
using Eigen::VectorXd;
using Triplet = Eigen::Triplet<double>;

// Global solve counters (for measuring search overhead).
static int    g_nsolve    = 0;     // number of SOCP solves
static double g_solve_sum = 0.0;   // accumulated Clarabel solve_time [s]
static void   reset_counters() { g_nsolve = 0; g_solve_sum = 0.0; }

// -----------------------------------------------------------------------------
//  Problem parameters (representative Mars soft-landing values, cf. the paper)
// -----------------------------------------------------------------------------
struct Params
{
    // Environment
    double g0 = 3.7114;          // gravitational acceleration [m/s^2] (Mars)

    // Vehicle / propulsion
    double m_wet = 2000.0;       // initial (wet) mass [kg]
    double m_dry = 1500.0;       // dry mass [kg]  (=> 500 kg usable propellant)
    double Isp = 225.0;          // specific impulse [s]
    double g_e = 9.807;          // standard gravity used in Isp [m/s^2]
    double Tmax = 24000.0;       // single-engine-cluster max thrust [N]
    double thr_min = 0.20;       // min throttle fraction  -> rho1 = thr_min*Tmax
    double thr_max = 0.80;       // max throttle fraction  -> rho2 = thr_max*Tmax

    // Constraints
    double theta_point_deg = 45.0;  // max thrust tilt from vertical [deg]
    double gamma_gs_deg    = 10.0;  // min glide-slope angle above ground [deg]

    // Boundary conditions
    double r0x = 2000.0, r0y = 1500.0;   // initial position [m]
    double v0x = -40.0,  v0y = -60.0;    // initial velocity [m/s]
    double rfx = 0.0,    rfy = 0.0;      // target (landing) position [m]
    // final velocity is zero (soft landing)

    // Derived
    double alpha() const { return 1.0 / (Isp * g_e); }   // mass flow per thrust [s/m]
    double rho1()  const { return thr_min * Tmax; }
    double rho2()  const { return thr_max * Tmax; }
};

// -----------------------------------------------------------------------------
//  Result of a single fixed-t_f solve
// -----------------------------------------------------------------------------
struct SolveResult
{
    SolverStatus status = SolverStatus::Unsolved;
    bool feasible = false;
    double tf = 0.0;
    int N = 0;
    double final_mass = 0.0;     // kg
    double fuel_used = 0.0;      // kg
    double solve_time = 0.0;     // Clarabel internal solve time [s]
    double wall_time = 0.0;      // wall-clock build+solve [s]
    uint32_t iters = 0;
    VectorXd x;                  // full decision vector (if feasible)
};

// -----------------------------------------------------------------------------
//  Build and solve the SOCP for a fixed time-of-flight t_f with N nodes.
// -----------------------------------------------------------------------------
SolveResult solve_gfold(const Params& P, double tf, int N, bool verbose,
                        bool free_landing = false)
{
    const double dt = tf / (N - 1);
    const double alpha = P.alpha();
    const double rho1 = P.rho1(), rho2 = P.rho2();
    const double cosT = std::cos(P.theta_point_deg * M_PI / 180.0);
    const double tanG = std::tan(P.gamma_gs_deg  * M_PI / 180.0);
    const double gy = -P.g0;

    const int S = 8;                 // variables per node
    const int n = S * N;             // total decision variables

    // Per-node variable index helpers
    auto RX = [&](int k){ return S*k + 0; };
    auto RY = [&](int k){ return S*k + 1; };
    auto VX = [&](int k){ return S*k + 2; };
    auto VY = [&](int k){ return S*k + 3; };
    auto Z  = [&](int k){ return S*k + 4; };
    auto UX = [&](int k){ return S*k + 5; };
    auto UY = [&](int k){ return S*k + 6; };
    auto SG = [&](int k){ return S*k + 7; };

    // Linearisation trajectory  z0(t) = ln(m_wet - alpha*rho2*t)  (min-mass arc)
    auto z0_at = [&](double t){ return std::log(P.m_wet - alpha * rho2 * t); };
    auto zU_at = [&](double t){ return std::log(P.m_wet - alpha * rho1 * t); };

    std::vector<Triplet> trip;
    std::vector<double>  bvec;
    std::vector<SupportedConeT<double>> cones;
    int row = 0;

    // Emit one scalar linear row:  s = b - A x   (cone determined by section)
    auto addRow = [&](std::initializer_list<std::pair<int,double>> coeffs, double b)
    {
        for (auto& c : coeffs) trip.emplace_back(row, c.first, c.second);
        bvec.push_back(b);
        ++row;
    };

    // ---- Section 1: equality constraints (ZeroCone) -------------------------
    const int row_eq_begin = row;

    // Initial state
    addRow({{RX(0),1}}, P.r0x);
    addRow({{RY(0),1}}, P.r0y);
    addRow({{VX(0),1}}, P.v0x);
    addRow({{VY(0),1}}, P.v0y);
    addRow({{Z(0), 1}}, std::log(P.m_wet));

    // Final state (z_N free -> maximised in the objective).
    // If free_landing, the terminal *position* is left unconstrained (the
    // vehicle may touch down anywhere the other constraints allow); only the
    // soft-landing zero-velocity condition is imposed.
    if (!free_landing)
    {
        addRow({{RX(N-1),1}}, P.rfx);
        addRow({{RY(N-1),1}}, P.rfy);
    }
    addRow({{VX(N-1),1}}, 0.0);
    addRow({{VY(N-1),1}}, 0.0);

    // Trapezoidal dynamics  x_{k+1} = x_k + dt/2 (f_k + f_{k+1})
    for (int k = 0; k < N - 1; ++k)
    {
        // position:  r_{k+1} - r_k - dt/2 (v_k + v_{k+1}) = 0
        addRow({{RX(k+1),1},{RX(k),-1},{VX(k),-dt/2},{VX(k+1),-dt/2}}, 0.0);
        addRow({{RY(k+1),1},{RY(k),-1},{VY(k),-dt/2},{VY(k+1),-dt/2}}, 0.0);
        // velocity:  v_{k+1} - v_k - dt/2 (u_k + u_{k+1}) = dt*g
        addRow({{VX(k+1),1},{VX(k),-1},{UX(k),-dt/2},{UX(k+1),-dt/2}}, 0.0);
        addRow({{VY(k+1),1},{VY(k),-1},{UY(k),-dt/2},{UY(k+1),-dt/2}}, dt*gy);
        // mass:  z_{k+1} - z_k + alpha*dt/2 (sigma_k + sigma_{k+1}) = 0
        addRow({{Z(k+1),1},{Z(k),-1},{SG(k),alpha*dt/2},{SG(k+1),alpha*dt/2}}, 0.0);
    }
    const int n_eq = row - row_eq_begin;
    cones.push_back(ZeroConeT<double>(n_eq));

    // ---- Section 2: linear inequalities (NonnegativeCone, s >= 0) -----------
    const int row_ineq_begin = row;
    for (int k = 0; k < N; ++k)
    {
        const double tk   = k * dt;
        const double z0k  = z0_at(tk);
        const double zUk  = zU_at(tk);
        const double c2k  = rho2 * std::exp(-z0k);   // = rho2 / (m_wet - alpha*rho2*tk)

        // (a) upper thrust bound (1st-order):  sigma <= c2k (1 - (z - z0k))
        //     => c2k(1+z0k) - c2k z - sigma >= 0
        addRow({{Z(k), c2k},{SG(k), 1.0}}, c2k * (1.0 + z0k));

        // (b) thrust pointing:  uy >= cos(theta) sigma   =>  uy - cosT sigma >= 0
        addRow({{UY(k), -1.0},{SG(k), cosT}}, 0.0);

        // (c) glide slope:  (ry - rfy) >= tan(gamma) |rx - rfx|   (two half-planes)
        addRow({{RY(k), -1.0},{RX(k),  tanG}}, -P.rfy + tanG * P.rfx);
        addRow({{RY(k), -1.0},{RX(k), -tanG}}, -P.rfy - tanG * P.rfx);

        // (d) mass lower / (e) upper bounds keep the Taylor expansion valid
        addRow({{Z(k), -1.0}}, -z0k);   // z >= z0k
        addRow({{Z(k),  1.0}},  zUk);   // z <= zUk
    }
    // (f) dry-mass limit:  z_N >= ln(m_dry)
    addRow({{Z(N-1), -1.0}}, -std::log(P.m_dry));
    const int n_ineq = row - row_ineq_begin;
    cones.push_back(NonnegativeConeT<double>(n_ineq));

    // ---- Section 3: second-order cones -------------------------------------
    for (int k = 0; k < N; ++k)
    {
        // Thrust magnitude cone:  sigma >= ||(ux, uy)||   ->  (sigma, ux, uy) in SOC3
        addRow({{SG(k), -1.0}}, 0.0);
        addRow({{UX(k), -1.0}}, 0.0);
        addRow({{UY(k), -1.0}}, 0.0);
        cones.push_back(SecondOrderConeT<double>(3));

        // Lower thrust bound (2nd-order):  sigma >= mu0 (1 - d + d^2/2), d = z - z0k.
        // Equivalent rotated-cone -> standard SOC3 with
        //   c0 = sigma - mu0/2 + 1/mu0,  c1 = sigma - mu0/2 - 1/mu0,
        //   c2 = sqrt(2)(z - z0k - 1),   and  c0 >= ||(c1, c2)||.
        const double tk  = k * dt;
        const double z0k = z0_at(tk);
        const double mu0 = rho1 * std::exp(-z0k);     // = rho1 / (m_wet - alpha*rho2*tk)
        addRow({{SG(k), -1.0}}, -mu0/2.0 + 1.0/mu0);              // c0
        addRow({{SG(k), -1.0}}, -mu0/2.0 - 1.0/mu0);              // c1
        addRow({{Z(k), -std::sqrt(2.0)}}, -std::sqrt(2.0)*(z0k + 1.0)); // c2
        cones.push_back(SecondOrderConeT<double>(3));
    }

    const int m = row;

    // ---- Assemble Clarabel inputs -----------------------------------------
    SparseMatrix<double> A(m, n);
    A.setFromTriplets(trip.begin(), trip.end());
    A.makeCompressed();

    SparseMatrix<double> Pmat(n, n);   // zero objective Hessian
    Pmat.makeCompressed();

    VectorXd q = VectorXd::Zero(n);
    q[Z(N-1)] = -1.0;                  // maximise z_N (== minimise fuel)

    VectorXd b = Eigen::Map<VectorXd>(bvec.data(), bvec.size());

    DefaultSettings<double> settings = DefaultSettings<double>::default_settings();
    settings.verbose = verbose;
    settings.max_iter = 200;

    // ---- Solve (timed) -----------------------------------------------------
    auto t0 = std::chrono::high_resolution_clock::now();
    DefaultSolver<double> solver(Pmat, q, A, b, cones, settings);
    solver.solve();
    auto t1 = std::chrono::high_resolution_clock::now();

    DefaultSolution<double> sol = solver.solution();
    ++g_nsolve;
    g_solve_sum += sol.solve_time;

    SolveResult R;
    R.status     = sol.status;
    R.tf         = tf;
    R.N          = N;
    R.solve_time = sol.solve_time;
    R.wall_time  = std::chrono::duration<double>(t1 - t0).count();
    R.iters      = sol.iterations;
    R.feasible   = (sol.status == SolverStatus::Solved ||
                    sol.status == SolverStatus::AlmostSolved);
    if (R.feasible)
    {
        R.x = VectorXd(sol.x);                 // copy out of the Eigen::Map
        R.final_mass = std::exp(R.x[Z(N-1)]);
        R.fuel_used  = P.m_wet - R.final_mass;
    }
    return R;
}

// -----------------------------------------------------------------------------
//  Dump the optimal trajectory of a result to CSV (for plotting).
// -----------------------------------------------------------------------------
void write_csv(const Params& P, const SolveResult& R, const char* path)
{
    if (!R.feasible) return;
    const int N = R.N, S = 8;
    const double dt = R.tf / (N - 1);
    std::ofstream f(path);
    f << "t,rx,ry,vx,vy,mass,ux,uy,sigma,thrust_N,throttle\n";
    for (int k = 0; k < N; ++k)
    {
        const double* x = R.x.data() + S*k;
        const double m = std::exp(x[4]);
        const double un = std::hypot(x[5], x[6]);
        const double thrust = m * un;          // ||T|| = m * ||u||  (= m*sigma at opt)
        f << k*dt << ',' << x[0] << ',' << x[1] << ',' << x[2] << ',' << x[3]
          << ',' << m << ',' << x[5] << ',' << x[6] << ',' << x[7]
          << ',' << thrust << ',' << thrust / P.Tmax << '\n';
    }
    std::printf("  trajectory written to %s\n", path);
}

// -----------------------------------------------------------------------------
//  Aggregate timing over a t_f sweep.
// -----------------------------------------------------------------------------
struct SweepStats
{
    int n_solves = 0, n_feasible = 0;
    double sum_solve = 0.0, sum_wall = 0.0;   // [s]
    double min_solve = 1e30, max_solve = 0.0; // [s] over feasible solves
    SolveResult best;
};

SweepStats run_sweep(const Params& P, int N, bool free_landing, bool print_rows)
{
    SweepStats S;
    if (print_rows)
        std::printf("   tf[s]   status        fuel[kg]   iters  solve[ms]  wall[ms]\n");

    for (double tf = 20.0; tf <= 80.0 + 1e-9; tf += 2.5)
    {
        SolveResult R = solve_gfold(P, tf, N, /*verbose=*/false, free_landing);
        ++S.n_solves;
        S.sum_solve += R.solve_time;
        S.sum_wall  += R.wall_time;

        const char* st = R.feasible ? "FEASIBLE" :
            (R.status == SolverStatus::PrimalInfeasible ? "infeasible" : "no-solve");
        if (print_rows)
            std::printf("   %5.1f   %-12s  %8.2f   %4u   %7.3f   %7.3f\n",
                        tf, st, R.feasible ? R.fuel_used
                                           : std::numeric_limits<double>::quiet_NaN(),
                        R.iters, R.solve_time*1e3, R.wall_time*1e3);

        if (R.feasible)
        {
            ++S.n_feasible;
            S.min_solve = std::min(S.min_solve, R.solve_time);
            S.max_solve = std::max(S.max_solve, R.solve_time);
            if (!S.best.feasible || R.fuel_used < S.best.fuel_used) S.best = R;
        }
    }
    return S;
}

void print_stats(const char* tag, const SweepStats& S)
{
    std::printf(" %-16s %2d solves: avg solve = %.3f ms  (min %.3f / max %.3f)  avg wall = %.3f ms\n",
                tag, S.n_solves, 1e3*S.sum_solve/S.n_solves,
                1e3*S.min_solve, 1e3*S.max_solve, 1e3*S.sum_wall/S.n_solves);
}

// =============================================================================
//  Efficient search:  ignition time  +  time-of-flight
// =============================================================================

// Ballistic coast (thrust off) for time tc under gravity only.  Returns the
// parameter set with the propagated initial state.
Params coast(const Params& P0, double tc)
{
    Params P = P0;
    P.r0x = P0.r0x + P0.v0x * tc;
    P.r0y = P0.r0y + P0.v0y * tc - 0.5 * P0.g0 * tc * tc;
    P.v0x = P0.v0x;
    P.v0y = P0.v0y - P0.g0 * tc;
    return P;
}

// Cheap analytic *necessary* feasibility screen (no SOCP):  can the available
// max deceleration null the vertical descent within the remaining altitude?
//   a_net = rho2/m_wet - g  ;  need  v_down^2 / (2 a_net) <= altitude.
// Rejects obviously hopeless states for free; never rejects a feasible one.
bool kinematic_possible(const Params& P)
{
    if (P.r0y <= 0.0) return false;
    const double a_net = P.rho2() / P.m_wet - P.g0;     // best-case net decel
    if (a_net <= 0.0) return false;
    const double v_down = (P.v0y < 0.0) ? -P.v0y : 0.0;
    return (v_down * v_down) <= 2.0 * a_net * P.r0y * 1.05; // 5% slack
}

// Does *any* feasible landing solution exist for this state?  Uses the fact
// that feasibility in tf is an interval; we probe a short bisection for the
// feasible floor.  Returns {exists, tf_floor}.  Counts solves globally.
bool landing_feasible(const Params& P, int N)
{
    if (!kinematic_possible(P)) return false;       // free analytic rejection
    // Feasibility in tf is an interval [tf_min, tf_max]; probe a coarse set and
    // accept as soon as one tf is feasible (existence test, not optimisation).
    for (double tf : {20.0, 32.0, 45.0, 60.0, 80.0})
        if (solve_gfold(P, tf, N, false).feasible) return true;
    return false;
}

// Fuel-optimal tf for a fixed state:  bisect the feasibility floor, then
// golden-section the (unimodal) fuel(tf) curve.  Returns best result.
SolveResult find_tf_optimal(const Params& P, int N,
                            double tf_min = 10.0, double tf_max = 100.0)
{
    SolveResult none;
    // --- 1) bisection for the minimum feasible tf (feasibility monotone up) --
    double lo = tf_min, hi = tf_max;
    if (!solve_gfold(P, hi, N, false).feasible) return none;   // no solution at all
    if (solve_gfold(P, lo, N, false).feasible) { hi = lo; }    // floor below range
    else {
        for (int i = 0; i < 7; ++i) {                          // ~0.5 s tolerance
            double mid = 0.5 * (lo + hi);
            if (solve_gfold(P, mid, N, false).feasible) hi = mid; else lo = mid;
        }
    }
    const double tf_floor = hi;

    // --- 2) golden-section minimise fuel on [tf_floor, tf_floor + window] ----
    const double gr = 0.6180339887;
    double a = tf_floor, b = std::min(tf_floor + 30.0, tf_max);
    auto fuel_at = [&](double tf)->double {
        SolveResult R = solve_gfold(P, tf, N, false);
        return R.feasible ? R.fuel_used : 1e30;
    };
    double c = b - gr * (b - a), d = a + gr * (b - a);
    double fc = fuel_at(c), fd = fuel_at(d);
    for (int i = 0; i < 8; ++i) {                              // ~0.5 s tolerance
        if (fc < fd) { b = d; d = c; fd = fc; c = b - gr*(b-a); fc = fuel_at(c); }
        else         { a = c; c = d; fc = fd; d = a + gr*(b-a); fd = fuel_at(d); }
    }
    const double tf_star = 0.5 * (a + b);
    return solve_gfold(P, tf_star, N, false);
}

// Latest ignition time along the ballistic coast for which a feasible landing
// still exists (feasibility monotone-decreasing along the coast).  Bisection.
double find_ignition_time(const Params& P0, int N, double tc_max)
{
    // If we cannot even land igniting *now*, there is no window.
    if (!landing_feasible(P0, N)) return -1.0;
    double lo = 0.0, hi = tc_max;            // lo feasible, hi (probably) not
    if (landing_feasible(coast(P0, hi), N)) return hi;   // feasible to the end
    for (int i = 0; i < 8; ++i) {            // ~tc_max/256 tolerance
        double mid = 0.5 * (lo + hi);
        if (landing_feasible(coast(P0, mid), N)) lo = mid; else hi = mid;
    }
    return lo;   // last feasible ignition time
}

// Fuel-optimal ignition time:  minimise fuel*(t_ig) over the coast window
// [0, t_ig_last].  Each evaluation is a full inner tf optimisation; the outer
// curve is (near-)unimodal so golden section converges in a handful of probes.
// Returns the best t_ig (and fills *out with the burn at that ignition).
double fuel_optimal_ignition(const Params& P0, int N, double t_ig_last,
                             SolveResult* out)
{
    const double gr = 0.6180339887;
    double a = 0.0, b = t_ig_last;
    auto fuel_at = [&](double tig)->double {
        SolveResult R = find_tf_optimal(coast(P0, tig), N);
        return R.feasible ? R.fuel_used : 1e30;
    };
    double c = b - gr*(b-a), d = a + gr*(b-a);
    double fc = fuel_at(c), fd = fuel_at(d);
    for (int i = 0; i < 6; ++i) {
        if (fc < fd) { b = d; d = c; fd = fc; c = b - gr*(b-a); fc = fuel_at(c); }
        else         { a = c; c = d; fc = fd; d = a + gr*(b-a); fd = fuel_at(d); }
    }
    double tig = 0.5*(a+b);
    // Golden section never evaluates the endpoints; the optimum may sit at
    // t_ig = 0 ("ignite now"), so test it explicitly and keep the better one.
    SolveResult Rmid = find_tf_optimal(coast(P0, tig), N);
    SolveResult Rnow = find_tf_optimal(P0, N);
    double fmid = Rmid.feasible ? Rmid.fuel_used : 1e30;
    double fnow = Rnow.feasible ? Rnow.fuel_used : 1e30;
    if (fnow <= fmid) { *out = Rnow; return 0.0; }
    *out = Rmid; return tig;
}

// -----------------------------------------------------------------------------
//  Empirical test of the premise behind the same authors' t_f optimisation
//  (Blackmore, Acikmese, Scharf, "Minimum-Landing-Error Powered-Descent
//   Guidance for Mars Landing", JGCD 2010; patent US 8,489,260): the minimum
//  fuel is a UNIMODAL function of t_f, so a golden-section line search finds
//  the global optimum.  Here we fine-grid fuel(t_f), check unimodality and
//  convexity, then confirm golden section lands on the grid minimum.
// -----------------------------------------------------------------------------
void run_tf_test(const Params& P, int N)
{
    std::printf("==============================================================\n");
    std::printf(" t_f optimisation test (Blackmore/Acikmese golden-search premise)\n");
    std::printf(" state r=(%.0f,%.0f) m  v=(%.0f,%.0f) m/s\n", P.r0x,P.r0y,P.v0x,P.v0y);
    std::printf("--------------------------------------------------------------\n");

    const double tf_lo = 25.0, tf_hi = 95.0, step = 0.5;
    std::ofstream f("gfold2d_tf_fuel.csv");
    f << "tf,feasible,fuel\n";

    std::vector<double> fuels;     // feasible fuel values (contiguous block)
    double best_tf = 0.0, best_fuel = 1e30;
    int argmin = 0, nfeas = 0;
    double first_feas_tf = -1.0;
    reset_counters();
    for (double tf = tf_lo; tf <= tf_hi + 1e-9; tf += step)
    {
        SolveResult R = solve_gfold(P, tf, N, false);
        f << tf << ',' << (R.feasible ? 1 : 0) << ',';
        if (R.feasible) {
            f << R.fuel_used;
            if (first_feas_tf < 0) first_feas_tf = tf;
            if (R.fuel_used < best_fuel) { best_fuel = R.fuel_used; best_tf = tf; argmin = nfeas; }
            fuels.push_back(R.fuel_used);
            ++nfeas;
        } else f << "nan";
        f << '\n';
    }
    const int grid_solves = g_nsolve;

    // Unimodality: non-increasing up to argmin, non-decreasing after (w/ tol).
    const double tol = 2e-2;   // kg, absorbs interior-point solve noise
    bool unimodal = true;
    for (int i = 1; i < argmin; ++i)
        if (fuels[i] > fuels[i-1] + tol) unimodal = false;
    for (int i = argmin + 1; i < (int)fuels.size(); ++i)
        if (fuels[i] < fuels[i-1] - tol) unimodal = false;

    // Convexity: discrete second difference should be >= -tol everywhere.
    int convex_viol = 0; double min_d2 = 1e30;
    for (int i = 1; i + 1 < (int)fuels.size(); ++i) {
        double d2 = fuels[i+1] - 2*fuels[i] + fuels[i-1];
        min_d2 = std::min(min_d2, d2);
        if (d2 < -tol) ++convex_viol;
    }

    std::printf(" fine grid: %d feasible pts (step %.2f s); feasible t_f >= %.1f s\n",
                nfeas, step, first_feas_tf);
    std::printf("   grid minimum   : t_f=%.2f s   fuel=%.3f kg   (%d solves)\n",
                best_tf, best_fuel, grid_solves);
    std::printf("   unimodal in t_f: %s\n", unimodal ? "YES" : "NO");
    std::printf("   convex in t_f  : %s   (min 2nd-diff=%.4f kg, %d viol > %.2f kg)\n",
                convex_viol == 0 ? "YES" : "approx (noise)", min_d2, convex_viol, tol);

    // Golden-section search (the authors' method) and its accuracy/cost.
    reset_counters();
    SolveResult G = find_tf_optimal(P, N);
    const int golden_solves = g_nsolve;
    std::printf("--------------------------------------------------------------\n");
    std::printf(" golden-section result: t_f*=%.3f s  fuel=%.3f kg  (%d solves)\n",
                G.tf, G.fuel_used, golden_solves);
    std::printf("   error vs grid min : dt_f=%+.3f s   dfuel=%+.4f kg  (%+.4f%%)\n",
                G.tf - best_tf, G.fuel_used - best_fuel,
                100.0*(G.fuel_used - best_fuel)/best_fuel);
    std::printf("   cost: %d golden solves vs %d fine-grid solves  (%.1fx fewer)\n",
                golden_solves, grid_solves, (double)grid_solves/golden_solves);
    std::printf(" wrote gfold2d_tf_fuel.csv\n");
    std::printf("--------------------------------------------------------------\n");
}

// -----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    Params P;
    // Modes:  (default) batch comparison;  "tf" = t_f unimodality/golden test.
    const bool tf_mode = (argc > 1 && std::strcmp(argv[1], "tf") == 0);
    const int N = tf_mode ? (argc > 2 ? std::atoi(argv[2]) : 60)
                          : (argc > 1 ? std::atoi(argv[1]) : 60);
    if (tf_mode) { run_tf_test(P, N); return 0; }


    std::printf("==============================================================\n");
    std::printf(" 2D GFOLD powered-descent solver (Clarabel SOCP)\n");
    std::printf("==============================================================\n");
    std::printf(" r0 = (%.0f, %.0f) m   v0 = (%.0f, %.0f) m/s\n",
                P.r0x, P.r0y, P.v0x, P.v0y);
    std::printf(" m_wet = %.0f kg  m_dry = %.0f kg  Tmax = %.0f N  throttle [%.2f,%.2f]\n",
                P.m_wet, P.m_dry, P.Tmax, P.thr_min, P.thr_max);
    std::printf(" g = %.4f m/s^2  Isp = %.0f s  glide >= %.0f deg  tilt <= %.0f deg\n",
                P.g0, P.Isp, P.gamma_gs_deg, P.theta_point_deg);
    std::printf(" N = %d nodes  ->  %d variables\n", N, 8*N);

    auto ms = [](double s){ return 1e3 * s; };

    // =========================================================================
    //  Batch comparison over several "high-speed, lateral-velocity" entry
    //  states.  For each we compute the landing fuel two ways:
    //    A) ignite-now : bisection feasible-floor + golden section on tf only
    //    B) joint-opt  : nested golden(t_ig) x golden(tf) -> fuel-optimal igniti.
    // =========================================================================
    struct Scenario { const char* name; double rx, ry, vx, vy; };
    const Scenario scen[] = {
        {"baseline",      2000, 1500,  -40,  -60},
        {"fast-lateral",  2500, 1800, -100,  -50},
        {"steep-fast",    1500, 1800,  -30, -130},
        {"high-slow",     2500, 3000,  -20,  -30},
        {"high-fast-lat", 3000, 2800, -120,  -60},
        {"low-fast",      1200, 1000,  -50, -110},
        {"far-downrange", 3500, 2000,  -80,  -70},
        {"near-vertical",  400, 2200,  -10, -100},
    };

    std::printf("--------------------------------------------------------------------------------\n");
    std::printf(" scenario        spd  | A ignite-now    | B joint-optimum            | dFuel(B-A)\n");
    std::printf("                 m/s  | tf*    fuel kg   | t_ig* tf*    fuel kg  spd  | kg     %%\n");
    std::printf("--------------------------------------------------------------------------------\n");

    int    nA = 0, nB = 0;          // total solves per method (timing)
    double sA = 0.0, sB = 0.0;      // total Clarabel solve-time per method
    for (const Scenario& s : scen)
    {
        Params Ps = P;
        Ps.r0x = s.rx; Ps.r0y = s.ry; Ps.v0x = s.vx; Ps.v0y = s.vy;
        const double spd0 = std::hypot(s.vx, s.vy);

        // --- A) ignite now: tf-only search -----------------------------------
        reset_counters();
        SolveResult A = find_tf_optimal(Ps, N);
        nA += g_nsolve; sA += g_solve_sum;

        if (!A.feasible) {
            std::printf(" %-14s %4.0f  | INFEASIBLE (cannot land even igniting now)\n",
                        s.name, spd0);
            continue;
        }

        // --- last-safe ignition window, then B) joint fuel-optimal ignition ---
        const double disc = s.vy*s.vy + 2.0*Ps.g0*s.ry;
        const double t_impact = (-s.vy + std::sqrt(disc)) / Ps.g0;
        double t_ig_last = find_ignition_time(Ps, N, t_impact);

        reset_counters();
        SolveResult B; double tig = 0.0;
        if (t_ig_last > 0.5)
            tig = fuel_optimal_ignition(Ps, N, t_ig_last, &B);
        else
            { B = A; tig = 0.0; }   // no usable coast window -> joint == now
        nB += g_nsolve; sB += g_solve_sum;

        Params Pb = coast(Ps, tig);
        const double spd_ig = std::hypot(Pb.v0x, Pb.v0y);
        const double dkg = B.fuel_used - A.fuel_used;
        const double dpc = 100.0 * dkg / A.fuel_used;

        std::printf(" %-14s %4.0f  | %5.1f %8.2f | %5.2f %5.1f %8.2f %4.0f | %+6.2f %+5.1f\n",
                    s.name, spd0,
                    A.tf, A.fuel_used,
                    tig, B.tf, B.fuel_used, spd_ig,
                    dkg, dpc);
    }
    std::printf("--------------------------------------------------------------------------------\n");
    std::printf(" TIMING:  A ignite-now total %d solves (%.0f ms) ; B joint total %d solves (%.0f ms)\n",
                nA, ms(sA), nB, ms(sB));
    std::printf("          (a naive 25x25 t_ig-tf grid per scenario would be %d solves)\n",
                625 * (int)(sizeof(scen)/sizeof(scen[0])));
    std::printf("--------------------------------------------------------------------------------\n");
    return 0;
}
