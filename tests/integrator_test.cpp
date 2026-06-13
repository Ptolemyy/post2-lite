#include "post2/core/frames.hpp"
#include "post2/integrators/dopri5.hpp"
#include "post2/integrators/integrator.hpp"
#include "post2/propagation/builtin_events.hpp"

#include <cmath>
#include <iostream>

namespace {

using post2::integrators::Dopri5Integrator;
using post2::integrators::EventFunction;
using post2::integrators::ExtendedDerivative;
using post2::integrators::ExtendedState;
using post2::integrators::IntegratorTolerances;
using post2::integrators::Rk4IntegratorAdapter;
using post2::integrators::StepResult;
using post2::integrators::make_integrator;

int g_failures = 0;

void check_close(const char* what, double got, double expected, double tol)
{
    if (std::abs(got - expected) > tol) {
        std::cerr << what << ": got=" << got << " expected=" << expected
                  << " tol=" << tol << '\n';
        ++g_failures;
    }
}

void check_true(const char* what, bool ok)
{
    if (!ok) {
        std::cerr << what << ": condition false\n";
        ++g_failures;
    }
}

// Embed a scalar y(t) and y'(t) into ExtendedState by abusing
// position.x and velocity_mps.x. Other fields stay zero. tank_masses_kg
// is empty.
auto pack_scalar = [](double y) {
    ExtendedState s;
    s.motion.position_m = {y, 0.0, 0.0};
    s.motion.velocity_mps = {0.0, 0.0, 0.0};
    return s;
};
auto unpack_scalar = [](const ExtendedState& s) { return s.motion.position_m.x; };

// dy/dt = -y, y(0) = 1. Analytic solution y(t) = exp(-t).
ExtendedDerivative decay_dynamics(double /*t*/, const ExtendedState& s)
{
    ExtendedDerivative d;
    d.motion_dot.d_position_mps = {-s.motion.position_m.x, 0.0, 0.0};
    d.motion_dot.d_velocity_mps2 = {0.0, 0.0, 0.0};
    return d;
}

void test_dopri5_decay_accuracy()
{
    IntegratorTolerances tol;
    tol.rtol = 1.0e-10;
    tol.atol_position_m = 1.0e-12;
    Dopri5Integrator integ(tol);

    ExtendedState s = pack_scalar(1.0);
    double t = 0.0;
    double h = 0.1;
    const double t_final = 1.0;

    while (t < t_final - 1.0e-12) {
        const double h_attempt = std::min(h, t_final - t);
        const StepResult res = integ.step(s, t, h_attempt, decay_dynamics, {});
        if (!res.accepted) {
            std::cerr << "step rejected unexpectedly\n";
            ++g_failures;
            break;
        }
        s = res.state_end;
        t = res.t_end;
        h = res.h_next_suggested;
    }

    const double expected = std::exp(-1.0);
    check_close("dopri5 exp(-1) accuracy", unpack_scalar(s), expected, 1.0e-8);
}

// dy/dt = +1, y(0) = 0 -> y(t) = t. Event g(t,y) = y - 1.0 should hit t=1.0.
ExtendedDerivative linear_dynamics(double /*t*/, const ExtendedState& /*s*/)
{
    ExtendedDerivative d;
    d.motion_dot.d_position_mps = {1.0, 0.0, 0.0};
    d.motion_dot.d_velocity_mps2 = {0.0, 0.0, 0.0};
    return d;
}

void test_dopri5_event_detection()
{
    IntegratorTolerances tol;
    Dopri5Integrator integ(tol);

    std::vector<EventFunction> events;
    EventFunction ev;
    ev.name = "y == 1";
    ev.terminating = true;
    ev.g = [](double /*t*/, const ExtendedState& s) {
        return s.motion.position_m.x - 1.0;
    };
    events.push_back(ev);

    ExtendedState s = pack_scalar(0.0);
    const StepResult res = integ.step(s, 0.0, 2.0, linear_dynamics, events);

    check_true("event triggered", res.event.has_value());
    if (res.event.has_value()) {
        check_close("event time", res.event->t_s, 1.0, 1.0e-9);
        check_close("event state y", res.event->state.motion.position_m.x, 1.0, 1.0e-9);
        check_close("step h_used at event", res.h_used, 1.0, 1.0e-9);
    }
}

void test_rk4_adapter_event_detection()
{
    Rk4IntegratorAdapter integ;
    std::vector<EventFunction> events;
    EventFunction ev;
    ev.name = "y == 1";
    ev.g = [](double /*t*/, const ExtendedState& s) {
        return s.motion.position_m.x - 1.0;
    };
    events.push_back(ev);

    ExtendedState s = pack_scalar(0.0);
    // h=2.0 spans across the event; linear interp -> t_event = 1.0 exactly.
    const StepResult res = integ.step(s, 0.0, 2.0, linear_dynamics, events);

    check_true("rk4-adapter event triggered", res.event.has_value());
    if (res.event.has_value()) {
        check_close("rk4-adapter event time", res.event->t_s, 1.0, 1.0e-9);
    }
}

// Harmonic oscillator: dy/dt = v, dv/dt = -y. Energy E = 0.5*(y^2 + v^2)
// should be conserved.
ExtendedDerivative oscillator_dynamics(double /*t*/, const ExtendedState& s)
{
    ExtendedDerivative d;
    // y stored in position.x, v stored in velocity.x
    d.motion_dot.d_position_mps = {s.motion.velocity_mps.x, 0.0, 0.0};
    d.motion_dot.d_velocity_mps2 = {-s.motion.position_m.x, 0.0, 0.0};
    return d;
}

void test_dopri5_energy_conservation()
{
    IntegratorTolerances tol;
    tol.rtol = 1.0e-10;
    tol.atol_position_m = 1.0e-12;
    tol.atol_velocity_mps = 1.0e-12;
    Dopri5Integrator integ(tol);

    ExtendedState s;
    s.motion.position_m = {1.0, 0.0, 0.0};
    s.motion.velocity_mps = {0.0, 0.0, 0.0};
    const double e0 = 0.5 * (1.0 + 0.0);

    double t = 0.0;
    double h = 0.2;
    const double t_final = 100.0;  // many periods (2pi)
    while (t < t_final - 1.0e-9) {
        const double h_attempt = std::min(h, t_final - t);
        const StepResult res = integ.step(s, t, h_attempt, oscillator_dynamics, {});
        if (!res.accepted) {
            std::cerr << "oscillator step rejected unexpectedly\n";
            ++g_failures;
            break;
        }
        s = res.state_end;
        t = res.t_end;
        h = res.h_next_suggested;
    }
    const double y = s.motion.position_m.x;
    const double v = s.motion.velocity_mps.x;
    const double e_final = 0.5 * (y * y + v * v);
    check_close("oscillator energy conservation", e_final, e0, 1.0e-7);
}

void test_factory_dispatch()
{
    IntegratorTolerances tol;
    auto rk4 = make_integrator("rk4", 1.0, tol);
    auto dopri5 = make_integrator("dopri5", 1.0, tol);
    auto legacy = make_integrator("ode", 1.0, tol);
    check_true("factory rk4 not null", rk4 != nullptr);
    check_true("factory dopri5 not null", dopri5 != nullptr);
    check_true("factory ode-alias not null", legacy != nullptr);
}

void test_builtin_altitude_events()
{
    using post2::propagation::altitude_threshold_event;
    using post2::propagation::altitude_zero_event;
    const double a_m = post2::core::frames::Wgs84::a_m;

    // At equator surface, altitude_zero should evaluate to ~0.
    ExtendedState s;
    s.motion.position_m = {a_m, 0.0, 0.0};
    s.motion.velocity_mps = {0.0, 0.0, 0.0};
    const auto ev0 = altitude_zero_event();
    check_close("altitude_zero at surface", ev0.g(0.0, s), 0.0, 1.0e-3);

    // 200 km above equator surface, threshold at 100 km -> g = +100000.
    s.motion.position_m = {a_m + 200000.0, 0.0, 0.0};
    const auto ev_threshold = altitude_threshold_event(100000.0);
    check_close("altitude_threshold sign", ev_threshold.g(0.0, s), 100000.0, 1.0e-3);
}

void test_builtin_apsis_events()
{
    using post2::propagation::periapsis_event;
    using post2::propagation::apoapsis_event;

    ExtendedState s;
    // r=(1,0,0), v=(1,0,0): r.v = 1 (heading outward, just past periapsis).
    s.motion.position_m = {1.0, 0.0, 0.0};
    s.motion.velocity_mps = {1.0, 0.0, 0.0};
    const auto peri = periapsis_event();
    const auto apo = apoapsis_event();
    check_close("periapsis g r=(1,0,0) v=(1,0,0)", peri.g(0.0, s), 1.0, 1.0e-12);
    check_close("apoapsis g  r=(1,0,0) v=(1,0,0)", apo.g(0.0, s), 1.0, 1.0e-12);

    // r=(1,0,0), v=(-1,0,0): r.v = -1 (heading inward, past apoapsis).
    s.motion.velocity_mps = {-1.0, 0.0, 0.0};
    check_close("periapsis g v=(-1,0,0)", peri.g(0.0, s), -1.0, 1.0e-12);

    // Circular orbit: r=(1,0,0), v=(0,1,0). r.v = 0 (at periapsis == apoapsis).
    s.motion.velocity_mps = {0.0, 1.0, 0.0};
    check_close("periapsis g circular", peri.g(0.0, s), 0.0, 1.0e-12);
}

} // namespace

int main()
{
    test_dopri5_decay_accuracy();
    test_dopri5_event_detection();
    test_rk4_adapter_event_detection();
    test_dopri5_energy_conservation();
    test_factory_dispatch();
    test_builtin_altitude_events();
    test_builtin_apsis_events();

    if (g_failures > 0) {
        std::cerr << g_failures << " integrator_test failure(s)\n";
        return 1;
    }
    std::cout << "integrator_test ok\n";
    return 0;
}
