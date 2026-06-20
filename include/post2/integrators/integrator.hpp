#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "post2/integrators/ode_integrator.hpp"

namespace post2::integrators {

// Per-component tolerance groups for adaptive error control.
//
//  rtol               is unitless and applied multiplicatively to the current
//                     state magnitude (per component).
//  atol_position_m    is the absolute floor of the error norm for ECI
//                     position components (m).
//  atol_velocity_mps  same for velocity components (m/s).
//  atol_tank_mass_kg  same for per-tank propellant mass components (kg).
//
// The component-wise scale used in the embedded-error WRMS norm is:
//   scale_i = atol_group(i) + rtol * max(|y_old_i|, |y_new_i|).
struct IntegratorTolerances {
    double rtol = 1.0e-8;
    double atol_position_m = 1.0e-3;
    double atol_velocity_mps = 1.0e-6;
    double atol_tank_mass_kg = 1.0e-3;
};

// A continuous event: integrator watches the sign of g(t, state) along the
// trajectory and roots it as it crosses zero. `direction` filters crossings:
// 0 = either direction, +1 = negative-to-positive, -1 = positive-to-negative.
// terminating events halt the step on detection (subsequent events of higher
// t are ignored).
struct EventFunction {
    std::function<double(double t_s, const ExtendedState&)> g;
    bool terminating = true;
    int direction = 0;
    std::string name;
};

struct EventHit {
    std::size_t event_index = 0;
    std::string name;
    double t_s = 0.0;
    ExtendedState state;
};

struct StepResult {
    ExtendedState state_end;
    double t_end = 0.0;
    double h_used = 0.0;
    double h_next_suggested = 0.0;
    bool accepted = true;
    std::optional<EventHit> event;
};

class IIntegrator {
public:
    virtual ~IIntegrator() = default;

    // Advance one step. The integrator may shrink h_suggested adaptively
    // (and report h_used < h_suggested). If any event in `events` changes
    // sign between t and t+h_used, the integrator brackets it and returns
    // an EventHit with the trajectory truncated to that time.
    //
    // Fixed-step integrators ignore the suggestion (h_used == h_suggested)
    // and detect events via a single end-of-step sign check + linear root.
    // Adaptive integrators use dense output + Brent.
    virtual StepResult step(
        const ExtendedState& state,
        double t_s,
        double h_suggested,
        const DynamicsFunction& dynamics,
        const std::vector<EventFunction>& events) = 0;
};

// Factory. Recognized types: "rk4" (fixed step), "dopri5" (adaptive 5/4),
// and "dop853" (adaptive 8th-order). "ode" is accepted as a legacy alias
// for "rk4".
std::unique_ptr<IIntegrator> make_integrator(
    const std::string& type,
    double step_s,
    const IntegratorTolerances& tolerances);

} // namespace post2::integrators
