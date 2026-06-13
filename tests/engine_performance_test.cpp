#include "post2/propagation/engine_performance.hpp"

#include <cmath>
#include <iostream>

namespace {

using post2::propagation::EnginePerformanceInputs;
using post2::propagation::EnginePerformanceOutputs;
using post2::propagation::evaluate_engine;
using post2::vehicle::EngineConfig;
using post2::vehicle::EngineThrottleCurvePoint;

constexpr double kStandardGravityMps2 = 9.80665;

int g_failures = 0;

void check_close(const char* what, double got, double expected, double tol)
{
    if (std::abs(got - expected) > tol) {
        std::cerr << what << ": got=" << got << " expected=" << expected
                  << " tol=" << tol << '\n';
        ++g_failures;
    }
}

EngineConfig make_vacuum_engine()
{
    EngineConfig e;
    e.enabled = true;
    e.thrust_vac_n = 1000.0;
    e.isp_vac_s = 300.0;
    e.engine_count = 1;
    e.max_throttle = 1.0;
    return e;
}

void test_vacuum_baseline()
{
    EngineConfig e = make_vacuum_engine();
    EnginePerformanceInputs in;
    in.throttle_command = 1.0;
    in.ambient_pressure_pa = 0.0;
    const EnginePerformanceOutputs out = evaluate_engine(e, in);

    check_close("vac thrust", out.thrust_n, 1000.0, 1.0e-9);
    const double expected_mdot = 1000.0 / (300.0 * kStandardGravityMps2);
    check_close("vac mdot", out.mdot_kgps, expected_mdot, 1.0e-12);
    check_close("vac isp", out.isp_s, 300.0, 1.0e-9);
    check_close("vac throttle_eff", out.effective_throttle, 1.0, 1.0e-12);
}

void test_pressure_correction()
{
    // F(p) = F_vac - p * Ae, mdot constant. Isp drops with p.
    EngineConfig e = make_vacuum_engine();
    e.nozzle_exit_area_m2 = 1.0;  // 1 m^2 exit area
    EnginePerformanceInputs in;
    in.throttle_command = 1.0;
    in.ambient_pressure_pa = 100.0;  // -> 100 N thrust loss

    const EnginePerformanceOutputs out = evaluate_engine(e, in);
    check_close("pressure-corrected thrust", out.thrust_n, 900.0, 1.0e-9);
    // mdot is set by chamber (vacuum design point), unchanged.
    const double expected_mdot = 1000.0 / (300.0 * kStandardGravityMps2);
    check_close("mdot constant under pressure", out.mdot_kgps, expected_mdot, 1.0e-12);
    // Isp = F / (mdot * g0) drops accordingly.
    const double expected_isp = 900.0 / (expected_mdot * kStandardGravityMps2);
    check_close("isp drops with pressure", out.isp_s, expected_isp, 1.0e-9);
}

void test_pressure_zero_at_high_pressure()
{
    // If p * Ae > F_vac the engine produces zero thrust (over-expanded floor).
    EngineConfig e = make_vacuum_engine();
    e.thrust_vac_n = 100.0;
    e.nozzle_exit_area_m2 = 1.0;
    EnginePerformanceInputs in;
    in.throttle_command = 1.0;
    in.ambient_pressure_pa = 200.0;
    const EnginePerformanceOutputs out = evaluate_engine(e, in);
    check_close("over-pressured thrust", out.thrust_n, 0.0, 1.0e-9);
}

void test_multi_engine_cluster()
{
    EngineConfig e = make_vacuum_engine();
    e.engine_count = 9;
    EnginePerformanceInputs in;
    in.throttle_command = 1.0;
    const EnginePerformanceOutputs out = evaluate_engine(e, in);
    check_close("cluster thrust = 9 x single", out.thrust_n, 9000.0, 1.0e-9);
    const double single_mdot = 1000.0 / (300.0 * kStandardGravityMps2);
    check_close("cluster mdot = 9 x single", out.mdot_kgps, 9.0 * single_mdot, 1.0e-12);
    check_close("cluster isp unchanged", out.isp_s, 300.0, 1.0e-9);
}

void test_min_throttle_clip_to_zero()
{
    EngineConfig e = make_vacuum_engine();
    e.min_throttle = 0.4;
    EnginePerformanceInputs in;
    in.throttle_command = 0.3;
    const EnginePerformanceOutputs out = evaluate_engine(e, in);
    check_close("sub-min command -> zero thrust", out.thrust_n, 0.0, 1.0e-9);
    check_close("sub-min command -> zero mdot", out.mdot_kgps, 0.0, 1.0e-9);
}

void test_max_throttle_clip()
{
    EngineConfig e = make_vacuum_engine();
    e.max_throttle = 0.7;
    EnginePerformanceInputs in;
    in.throttle_command = 1.0;
    const EnginePerformanceOutputs out = evaluate_engine(e, in);
    check_close("max-throttle clip thrust", out.thrust_n, 700.0, 1.0e-9);
}

void test_throttle_curve_lookup()
{
    EngineConfig e = make_vacuum_engine();
    e.throttle_curve = {
        {0.0, 0.0},
        {0.5, 0.3},   // throttle 0.5 -> mdot_ratio 0.3
        {1.0, 1.0},
    };
    EnginePerformanceInputs in;
    in.throttle_command = 0.5;
    const EnginePerformanceOutputs out = evaluate_engine(e, in);
    // mdot_ratio = 0.3 -> thrust = 1000 * 0.3 = 300, mdot = vac_mdot * 0.3
    check_close("throttle_curve lookup thrust", out.thrust_n, 300.0, 1.0e-9);
    const double vac_mdot = 1000.0 / (300.0 * kStandardGravityMps2);
    check_close("throttle_curve lookup mdot", out.mdot_kgps, 0.3 * vac_mdot, 1.0e-12);
}

void test_throttle_curve_linear_interp()
{
    EngineConfig e = make_vacuum_engine();
    e.throttle_curve = {
        {0.0, 0.0},
        {1.0, 1.0},
    };
    // Linear identity table; equivalent to no curve. Spot check at 0.7.
    EnginePerformanceInputs in;
    in.throttle_command = 0.7;
    const EnginePerformanceOutputs out = evaluate_engine(e, in);
    check_close("linear-table thrust at 0.7", out.thrust_n, 700.0, 1.0e-9);
}

void test_disabled_engine_returns_zero()
{
    EngineConfig e = make_vacuum_engine();
    e.enabled = false;
    EnginePerformanceInputs in;
    in.throttle_command = 1.0;
    const EnginePerformanceOutputs out = evaluate_engine(e, in);
    check_close("disabled thrust", out.thrust_n, 0.0, 1.0e-12);
    check_close("disabled mdot", out.mdot_kgps, 0.0, 1.0e-12);
}

void test_pressure_correction_with_count()
{
    // Per-engine F = F_vac - p*Ae; cluster aggregate = count x per-engine.
    EngineConfig e = make_vacuum_engine();
    e.thrust_vac_n = 1000.0;
    e.nozzle_exit_area_m2 = 1.0;
    e.engine_count = 4;
    EnginePerformanceInputs in;
    in.throttle_command = 1.0;
    in.ambient_pressure_pa = 100.0;
    const EnginePerformanceOutputs out = evaluate_engine(e, in);
    // Per-engine 900 N x 4 = 3600 N.
    check_close("cluster + pressure thrust", out.thrust_n, 3600.0, 1.0e-9);
}

} // namespace

int main()
{
    test_vacuum_baseline();
    test_pressure_correction();
    test_pressure_zero_at_high_pressure();
    test_multi_engine_cluster();
    test_min_throttle_clip_to_zero();
    test_max_throttle_clip();
    test_throttle_curve_lookup();
    test_throttle_curve_linear_interp();
    test_disabled_engine_returns_zero();
    test_pressure_correction_with_count();

    if (g_failures > 0) {
        std::cerr << g_failures << " engine_performance_test failure(s)\n";
        return 1;
    }
    std::cout << "engine_performance_test ok\n";
    return 0;
}
