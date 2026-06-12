#include "post2/integrators/ode_integrator.hpp"

#include "post2/vehicle/runtime_state.hpp"

#include <algorithm>
#include <stdexcept>

namespace post2::integrators {

namespace {

using post2::vehicle::CartesianState6D;
using post2::vehicle::CartesianStateDerivative6D;

ExtendedState add_scaled(const ExtendedState& state, const ExtendedDerivative& derivative, double scale)
{
    ExtendedState out;
    out.motion = {
        state.motion.position_m + derivative.motion_dot.d_position_mps * scale,
        state.motion.velocity_mps + derivative.motion_dot.d_velocity_mps2 * scale,
    };
    out.tank_masses_kg.resize(state.tank_masses_kg.size());
    const std::size_t n = std::min(state.tank_masses_kg.size(), derivative.tank_mass_dots_kgps.size());
    for (std::size_t i = 0; i < n; ++i) {
        out.tank_masses_kg[i] = state.tank_masses_kg[i] + derivative.tank_mass_dots_kgps[i] * scale;
    }
    for (std::size_t i = n; i < state.tank_masses_kg.size(); ++i) {
        out.tank_masses_kg[i] = state.tank_masses_kg[i];
    }
    return out;
}

ExtendedState build_initial_extended(const post2::vehicle::VehicleRuntimeState& runtime)
{
    ExtendedState state;
    state.motion = runtime.vehicle.motion;
    state.tank_masses_kg = post2::vehicle::read_tank_masses_flat(runtime);
    return state;
}

} // namespace

Rk4OdeIntegrator::Rk4OdeIntegrator(OdeIntegratorOptions options)
    : options_(options)
{
}

ExtendedState Rk4OdeIntegrator::step(
    const ExtendedState& state,
    double time_s,
    double step_s,
    const DynamicsFunction& dynamics) const
{
    const ExtendedDerivative k1 = dynamics(time_s, state);
    const ExtendedDerivative k2 = dynamics(time_s + step_s * 0.5, add_scaled(state, k1, step_s * 0.5));
    const ExtendedDerivative k3 = dynamics(time_s + step_s * 0.5, add_scaled(state, k2, step_s * 0.5));
    const ExtendedDerivative k4 = dynamics(time_s + step_s, add_scaled(state, k3, step_s));

    const auto d_position =
        (k1.motion_dot.d_position_mps + (k2.motion_dot.d_position_mps + k3.motion_dot.d_position_mps) * 2.0 + k4.motion_dot.d_position_mps) / 6.0;
    const auto d_velocity =
        (k1.motion_dot.d_velocity_mps2 + (k2.motion_dot.d_velocity_mps2 + k3.motion_dot.d_velocity_mps2) * 2.0 + k4.motion_dot.d_velocity_mps2) / 6.0;

    ExtendedState out;
    out.motion = {
        state.motion.position_m + d_position * step_s,
        state.motion.velocity_mps + d_velocity * step_s,
    };
    out.tank_masses_kg.assign(state.tank_masses_kg.size(), 0.0);
    for (std::size_t i = 0; i < state.tank_masses_kg.size(); ++i) {
        const double d1 = i < k1.tank_mass_dots_kgps.size() ? k1.tank_mass_dots_kgps[i] : 0.0;
        const double d2 = i < k2.tank_mass_dots_kgps.size() ? k2.tank_mass_dots_kgps[i] : 0.0;
        const double d3 = i < k3.tank_mass_dots_kgps.size() ? k3.tank_mass_dots_kgps[i] : 0.0;
        const double d4 = i < k4.tank_mass_dots_kgps.size() ? k4.tank_mass_dots_kgps[i] : 0.0;
        const double d_mass = (d1 + 2.0 * (d2 + d3) + d4) / 6.0;
        out.tank_masses_kg[i] = std::max(0.0, state.tank_masses_kg[i] + d_mass * step_s);
    }
    return out;
}

post2::core::StateLog Rk4OdeIntegrator::integrate(
    const post2::core::StateLog& initial_state_log,
    const MaxTimeTerminationCondition& termination,
    const DynamicsFunction& dynamics,
    const RuntimeUpdateFunction& runtime_update) const
{
    if (initial_state_log.empty()) {
        throw std::invalid_argument("initial StateLog must contain at least one state");
    }
    if (termination.max_time_s < initial_state_log.front().time_s) {
        throw std::invalid_argument("max_time_s is earlier than the initial StateLog");
    }
    if (options_.step_s <= 0.0) {
        throw std::invalid_argument("integrator step_s must be positive");
    }

    post2::core::StateLog state_log = initial_state_log;
    state_log.truncate_after(termination.max_time_s);
    if (state_log.empty()) {
        throw std::invalid_argument("StateLog truncation removed every state");
    }
    if (state_log.back().time_s >= termination.max_time_s) {
        return state_log;
    }

    auto runtime = state_log.back().runtime;
    ExtendedState state = build_initial_extended(runtime);
    double time_s = state_log.back().time_s;
    while (time_s < termination.max_time_s) {
        const double h = std::min(options_.step_s, termination.max_time_s - time_s);
        state = step(state, time_s, h, dynamics);
        time_s += h;
        runtime = runtime_update(runtime, state, time_s, h);
        state.motion = runtime.vehicle.motion;
        state.tank_masses_kg = post2::vehicle::read_tank_masses_flat(runtime);
        state_log.append(runtime);
    }

    return state_log;
}

} // namespace post2::integrators
