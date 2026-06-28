#include "post2/core/reentry_solver.hpp"

#include <cmath>
#include <iostream>

// Verifies the Clarabel integration gate. Works in both build configurations:
//   * POST2_USE_CLARABEL=ON  -> the reference QP (min x^2 s.t. x>=1) solves x*=1.
//   * default (Clarabel off) -> the solver reports unavailable, no crash.
int main()
{
    const bool available = post2::core::clarabel_available();
    double x = -123.0;
    const bool solved = post2::core::clarabel_selftest(&x);

    if (available) {
        if (!solved) {
            std::cerr << "FAIL: Clarabel available but selftest did not solve\n";
            return 1;
        }
        if (std::fabs(x - 1.0) > 1.0e-4) {
            std::cerr << "FAIL: Clarabel selftest x = " << x << ", expected 1\n";
            return 1;
        }
        std::cout << "reentry solver test passed (Clarabel enabled, x = " << x << ")\n";
    } else {
        if (solved) {
            std::cerr << "FAIL: Clarabel unavailable but selftest claimed success\n";
            return 1;
        }
        std::cout << "reentry solver test passed (Clarabel disabled; fallback OK)\n";
    }

    // Entry-burn guidance: only exercised when Clarabel is compiled in.
    if (available) {
        post2::core::EntryBurnProblem base;
        base.altitude_m = 60000.0;
        base.speed_mps = 2200.0;
        base.flight_path_angle_rad = 1.0;  // ~57 deg below horizontal (steep)
        base.mass_kg = 30000.0;
        base.max_thrust_n = 2.5e6;
        base.min_thrust_n = 0.4e6;
        base.exhaust_velocity_mps = 3000.0;
        base.drag_cd = 0.8;
        base.ref_area_m2 = 10.5;
        base.nose_radius_m = 1.0;
        base.dt_s = 1.0;
        base.max_nodes = 240;
        base.scvx_iterations = 4;

        // 1) Effectively no limit -> no burn; record the ballistic peak q.
        post2::core::EntryBurnProblem hi = base;
        hi.q_limit_pa = 1.0e12;
        post2::core::EntryBurnSolution s_hi;
        if (!post2::core::solve_entry_burn(hi, &s_hi) || !s_hi.ok) {
            std::cerr << "FAIL: entry-burn solve failed at huge limit\n";
            return 1;
        }
        if (s_hi.burn_needed) {
            std::cerr << "FAIL: burn flagged needed under an effectively infinite limit\n";
            return 1;
        }
        const double ballistic_q = s_hi.predicted_peak_q_pa;
        if (!(ballistic_q > 0.0)) {
            std::cerr << "FAIL: ballistic peak q non-positive (" << ballistic_q << ")\n";
            return 1;
        }

        // 2) Limit below the ballistic peak -> a burn is sized that reduces it.
        post2::core::EntryBurnProblem lo = base;
        lo.q_limit_pa = 0.6 * ballistic_q;
        post2::core::EntryBurnSolution s_lo;
        if (!post2::core::solve_entry_burn(lo, &s_lo) || !s_lo.ok) {
            std::cerr << "FAIL: entry-burn solve failed at reduced limit\n";
            return 1;
        }
        if (!s_lo.burn_needed) {
            std::cerr << "FAIL: expected a burn when limit < ballistic peak\n";
            return 1;
        }
        if (!(s_lo.burn_duration_s > 0.0) || !(s_lo.thrust_n > 0.0)) {
            std::cerr << "FAIL: burn window/thrust empty (dur=" << s_lo.burn_duration_s
                      << ", T=" << s_lo.thrust_n << ")\n";
            return 1;
        }
        if (!(s_lo.predicted_peak_q_pa < ballistic_q)) {
            std::cerr << "FAIL: burn did not reduce peak q (" << s_lo.predicted_peak_q_pa
                      << " vs ballistic " << ballistic_q << ")\n";
            return 1;
        }
        std::cout << "entry-burn test passed: ballistic q=" << ballistic_q
                  << " Pa -> with burn q=" << s_lo.predicted_peak_q_pa
                  << " Pa, ignition+" << s_lo.ignition_delay_s
                  << "s for " << s_lo.burn_duration_s << "s at "
                  << s_lo.thrust_n << " N\n";
    }
    return 0;
}
