#include "post2/core/simulation_driver.hpp"

#include "post2/core/control_models.hpp"
#include "post2/core/coordinates.hpp"
#include "post2/integrators/ode_integrator.hpp"
#include "post2/propagation/force_models.hpp"
#include "post2/propagation/vehicle_consumption.hpp"
#include "post2/vehicle/vehicle_config_io.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace post2::core {

namespace {

State make_hold_down_clamp_state(const SimulationConfig& config, double time_s)
{
    return launch_site_inertial_state(config, time_s);
}

void set_hold_down_clamp_state(
    post2::vehicle::VehicleRuntimeState* runtime,
    const SimulationConfig& config,
    bool active)
{
    runtime->hold_down_clamp.active = active;
    runtime->hold_down_clamp.planet_fixed_position_m = launch_site_planet_fixed_position_m(config);
}

post2::integrators::ExtendedDerivative inertial_extended_dynamics(
    const SimulationConfig& config,
    const post2::integrators::ExtendedState& state,
    const Vec3& thrust_acceleration_mps2,
    std::vector<double> tank_mass_dots_kgps)
{
    Vec3 acceleration_mps2 =
        post2::propagation::gravity_acceleration_mps2(config, state.motion.position_m) +
        thrust_acceleration_mps2;
    acceleration_mps2 = acceleration_mps2 +
        post2::propagation::surface_normal_acceleration_mps2(config, state.motion);

    return {
        {state.motion.velocity_mps, acceleration_mps2},
        std::move(tank_mass_dots_kgps),
    };
}

bool validate_config(const SimulationConfig& config, std::string* error)
{
    if (config.earth_radius_m <= 0.0) {
        *error = "earth_radius_m must be positive";
        return false;
    }
    if (config.earth_mu_m3s2 <= 0.0) {
        *error = "earth_mu_m3s2 must be positive";
        return false;
    }
    if (config.earth_rotation_rad_per_s < 0.0) {
        *error = "earth_rotation_rad_per_s cannot be negative";
        return false;
    }
    if (config.initial_altitude_m <= 0.0) {
        *error = "initial_altitude_m must be positive";
        return false;
    }
    if (config.duration_s <= 0.0) {
        *error = "duration_s must be positive";
        return false;
    }
    if (config.step_s <= 0.0) {
        *error = "step_s must be positive";
        return false;
    }
    if (config.launch_site.latitude_deg < -90.0 || config.launch_site.latitude_deg > 90.0) {
        *error = "launch_site.latitude_deg must be in [-90, 90]";
        return false;
    }
    if (config.launch_site.altitude_m <= -config.earth_radius_m) {
        *error = "launch_site.altitude_m is below the planet center";
        return false;
    }
    if (config.hold_down_clamp.release_time_s < 0.0) {
        *error = "hold_down_clamp.release_time_s cannot be negative";
        return false;
    }
    if (!post2::vehicle::validate_vehicle_config(config.vehicle, error)) {
        return false;
    }
    return true;
}

bool vehicle_impacted_earth(const StateLog& state_log, const SimulationConfig& config)
{
    constexpr double tolerance_m = 1.0;
    for (const auto& entry : state_log.entries()) {
        if (entry.radius_m < config.earth_radius_m - tolerance_m) {
            return true;
        }
        if (entry.radius_m <= config.earth_radius_m &&
            !entry.hold_down_clamp_active &&
            !config.normal_force.enabled) {
            return true;
        }
    }
    return false;
}

SimulationConfig make_phase_simulation_config(
    const CaseConfig& case_config,
    const PhaseConfig& phase,
    double phase_end_time_s)
{
    SimulationConfig config;
    config.earth_radius_m = case_config.earth_radius_m;
    config.earth_mu_m3s2 = case_config.earth_mu_m3s2;
    config.earth_rotation_rad_per_s = case_config.earth_rotation_rad_per_s;
    config.duration_s = phase_end_time_s;
    config.step_s = case_config.step_s;
    config.launch_site = case_config.launch_site;
    config.normal_force.enabled = phase.force_models.normal_force;
    config.vehicle = case_config.vehicle;
    return config;
}

bool action_type_is_supported(const std::string& type)
{
    return type == "set_engine_enabled" ||
        type == "set_hold_down_clamp_active" ||
        type == "set_stage_active" ||
        type == "set_stage_attached";
}

bool validate_case_config(const CaseConfig& config, std::string* error)
{
    if (config.earth_radius_m <= 0.0) {
        *error = "earth_radius_m must be positive";
        return false;
    }
    if (config.earth_mu_m3s2 <= 0.0) {
        *error = "earth_mu_m3s2 must be positive";
        return false;
    }
    if (config.earth_rotation_rad_per_s < 0.0) {
        *error = "earth_rotation_rad_per_s cannot be negative";
        return false;
    }
    if (config.step_s <= 0.0) {
        *error = "step_s must be positive";
        return false;
    }
    if (config.launch_site.latitude_deg < -90.0 || config.launch_site.latitude_deg > 90.0) {
        *error = "launch_site.latitude_deg must be in [-90, 90]";
        return false;
    }
    if (config.launch_site.altitude_m <= -config.earth_radius_m) {
        *error = "launch_site.altitude_m is below the planet center";
        return false;
    }
    if (config.phases.empty()) {
        *error = "case must contain at least one phase";
        return false;
    }
    if (!post2::vehicle::validate_vehicle_config(config.vehicle, error)) {
        return false;
    }

    for (std::size_t i = 0; i < config.phases.size(); ++i) {
        const PhaseConfig& phase = config.phases[i];
        if (phase.duration_s <= 0.0) {
            *error = "phase " + std::to_string(i) + " duration_s must be positive";
            return false;
        }
        if (phase.integrator != "ode") {
            *error = "phase " + std::to_string(i) + " integrator must be \"ode\"";
            return false;
        }
        if (i > 0 && !phase.inherit_initial_state && !phase.initial_state_eci.has_value()) {
            *error = "phase " + std::to_string(i) + " needs inherit_initial_state or initial_state_eci";
            return false;
        }
        for (const auto& action : phase.actions) {
            if (action.time_s < 0.0 || action.time_s > phase.duration_s) {
                *error = "phase " + std::to_string(i) + " action time_s is outside phase duration";
                return false;
            }
            if (!action_type_is_supported(action.type)) {
                *error = "unsupported phase action: " + action.type;
                return false;
            }
            if ((action.type == "set_stage_active" || action.type == "set_stage_attached") &&
                action.stage_index < 0 &&
                action.stage_name.empty()) {
                *error = action.type + " action needs stage_index or stage_name";
                return false;
            }
        }
    }
    return true;
}

bool case_vehicle_impacted_earth(const StateLog& state_log, const CaseConfig& config)
{
    constexpr double tolerance_m = 1.0;
    for (const auto& entry : state_log.entries()) {
        if (entry.radius_m < config.earth_radius_m - tolerance_m) {
            return true;
        }
        const bool phase_normal_force_enabled =
            entry.phase_index >= 0 &&
            static_cast<std::size_t>(entry.phase_index) < config.phases.size()
                ? config.phases[static_cast<std::size_t>(entry.phase_index)].force_models.normal_force
                : true;
        if (entry.radius_m <= config.earth_radius_m &&
            !entry.hold_down_clamp_active &&
            !phase_normal_force_enabled) {
            return true;
        }
    }
    return false;
}

struct RuntimeControl {
    bool engine_enabled = false;
    bool hold_down_clamp_active = false;
    std::size_t next_action_index = 0;
};

struct TimedAction {
    PhaseAction action;
};

std::vector<TimedAction> sorted_actions(const PhaseConfig& phase)
{
    std::vector<TimedAction> actions;
    actions.reserve(phase.actions.size());
    for (const auto& action : phase.actions) {
        actions.push_back({action});
    }
    std::sort(actions.begin(), actions.end(), [](const TimedAction& lhs, const TimedAction& rhs) {
        return lhs.action.time_s < rhs.action.time_s;
    });
    return actions;
}

void apply_due_actions(
    const std::vector<TimedAction>& actions,
    double phase_time_s,
    const SimulationConfig& simulation_config,
    RuntimeControl* control,
    post2::vehicle::VehicleRuntimeState* runtime)
{
    constexpr double epsilon_s = 1.0e-9;
    while (control->next_action_index < actions.size() &&
           actions[control->next_action_index].action.time_s <= phase_time_s + epsilon_s) {
        const PhaseAction& action = actions[control->next_action_index].action;
        if (action.type == "set_engine_enabled") {
            control->engine_enabled = action.value;
            runtime->engine.enabled = action.value;
        } else if (action.type == "set_hold_down_clamp_active") {
            control->hold_down_clamp_active = action.value;
            set_hold_down_clamp_state(runtime, simulation_config, action.value);
        } else if (action.type == "set_stage_active" || action.type == "set_stage_attached") {
            std::size_t stage_index = 0;
            bool found_stage = false;
            if (action.stage_index >= 0 &&
                static_cast<std::size_t>(action.stage_index) < runtime->stages.size()) {
                stage_index = static_cast<std::size_t>(action.stage_index);
                found_stage = true;
            } else if (!action.stage_name.empty()) {
                for (std::size_t i = 0; i < runtime->stages.size(); ++i) {
                    if (runtime->stages[i].name == action.stage_name) {
                        stage_index = i;
                        found_stage = true;
                        break;
                    }
                }
            }
            if (found_stage) {
                if (action.type == "set_stage_attached") {
                    post2::vehicle::set_stage_attached(runtime, stage_index, action.value);
                } else {
                    post2::vehicle::set_stage_active(runtime, stage_index, action.value);
                }
            }
        }
        ++control->next_action_index;
    }
}

double next_action_time_s(const std::vector<TimedAction>& actions, const RuntimeControl& control)
{
    if (control.next_action_index >= actions.size()) {
        return std::numeric_limits<double>::infinity();
    }
    return actions[control.next_action_index].action.time_s;
}

post2::propagation::EngineCommand make_engine_command(
    const PhaseConfig& phase,
    const PhaseContext& context,
    const IThrottleModel& throttle_model,
    const ISteeringModel& steering_model,
    const post2::vehicle::VehicleRuntimeState& runtime,
    const State& state,
    double absolute_time_s,
    bool engine_enabled)
{
    if (!phase.force_models.thrust) {
        return {};
    }

    post2::vehicle::VehicleRuntimeState runtime_for_model = runtime;
    runtime_for_model.engine.enabled = engine_enabled;
    const double phase_time_s = absolute_time_s - context.phase_start_time_s;
    return {
        engine_enabled,
        throttle_model.throttle(phase_time_s, runtime_for_model, context),
        steering_model.thrust_direction_eci(phase_time_s, state, runtime_for_model, context),
    };
}

post2::integrators::ExtendedDerivative phase_extended_dynamics(
    const SimulationConfig& simulation_config,
    const PhaseConfig& phase,
    const post2::integrators::ExtendedState& state,
    const Vec3& thrust_acceleration_mps2,
    std::vector<double> tank_mass_dots_kgps)
{
    Vec3 acceleration_mps2;
    if (phase.force_models.gravity) {
        acceleration_mps2 = acceleration_mps2 +
            post2::propagation::gravity_acceleration_mps2(simulation_config, state.motion.position_m);
    }
    if (phase.force_models.thrust) {
        acceleration_mps2 = acceleration_mps2 + thrust_acceleration_mps2;
    }
    if (phase.force_models.normal_force) {
        acceleration_mps2 = acceleration_mps2 +
            post2::propagation::surface_normal_acceleration_mps2(simulation_config, state.motion);
    }
    return {
        {state.motion.velocity_mps, acceleration_mps2},
        std::move(tank_mass_dots_kgps),
    };
}

void merge_phase_log(StateLog* merged, const StateLog& phase_log)
{
    constexpr double duplicate_epsilon_s = 1.0e-9;
    for (const auto& entry : phase_log.entries()) {
        if (!merged->empty() && std::abs(entry.time_s - merged->back().time_s) <= duplicate_epsilon_s) {
            continue;
        }
        merged->append(entry);
    }
}

StateLog propagate_phase(
    const CaseConfig& case_config,
    const PhaseConfig& phase,
    std::size_t phase_index,
    double phase_start_time_s,
    const post2::vehicle::VehicleRuntimeState& initial_runtime)
{
    const double phase_end_time_s = phase_start_time_s + phase.duration_s;
    const SimulationConfig simulation_config =
        make_phase_simulation_config(case_config, phase, phase_end_time_s);
    post2::propagation::VehicleConsumptionPropagator vehicle_propagator(case_config.vehicle);
    post2::integrators::Rk4OdeIntegrator integrator({case_config.step_s});
    const auto throttle_model = make_throttle_model(phase.throttle_model);
    const auto steering_model = make_steering_model(phase.steering_model);
    const PhaseContext context{&case_config, &phase, phase_index, phase_start_time_s};
    const std::vector<TimedAction> actions = sorted_actions(phase);

    RuntimeControl control;
    control.engine_enabled = initial_runtime.engine.enabled;
    control.hold_down_clamp_active = phase.hold_down_clamp_initial_active;

    auto runtime = initial_runtime;
    runtime.time_s = phase_start_time_s;
    set_hold_down_clamp_state(&runtime, simulation_config, control.hold_down_clamp_active);
    apply_due_actions(actions, 0.0, simulation_config, &control, &runtime);

    StateLog state_log(case_config.earth_radius_m, case_config.vehicle);
    state_log.set_phase_metadata(static_cast<int>(phase_index), phase.name);
    state_log.append(runtime);

    double time_s = phase_start_time_s;
    while (time_s < phase_end_time_s) {
        const double phase_time_s = time_s - phase_start_time_s;
        apply_due_actions(actions, phase_time_s, simulation_config, &control, &runtime);

        const double next_action_absolute_s = phase_start_time_s + next_action_time_s(actions, control);
        double step_s = std::min(case_config.step_s, phase_end_time_s - time_s);
        if (next_action_absolute_s > time_s && next_action_absolute_s < time_s + step_s) {
            step_s = next_action_absolute_s - time_s;
        }
        if (step_s <= 1.0e-12) {
            step_s = std::min(case_config.step_s, phase_end_time_s - time_s);
        }

        const State current_state = runtime.vehicle.motion;
        const auto command = make_engine_command(
            phase,
            context,
            *throttle_model,
            *steering_model,
            runtime,
            current_state,
            time_s,
            control.engine_enabled);

        post2::integrators::ExtendedState current_extended{
            current_state,
            post2::vehicle::read_tank_masses_flat(runtime),
        };

        post2::propagation::DerivativeResult last_eval;
        post2::integrators::ExtendedState integrated;
        if (control.hold_down_clamp_active) {
            // Engine still burns while clamped: integrate tank masses with
            // forward Euler (motion pinned to launch site rotates with Earth
            // and is overridden at the end of the step regardless).
            last_eval = vehicle_propagator.compute_derivatives(
                time_s, runtime, current_extended.tank_masses_kg, current_state, command);
            integrated.motion = make_hold_down_clamp_state(simulation_config, time_s + step_s);
            integrated.tank_masses_kg.resize(current_extended.tank_masses_kg.size());
            for (std::size_t i = 0; i < integrated.tank_masses_kg.size(); ++i) {
                const double dot = i < last_eval.tank_mass_dots_kgps.size()
                    ? last_eval.tank_mass_dots_kgps[i]
                    : 0.0;
                integrated.tank_masses_kg[i] =
                    std::max(0.0, current_extended.tank_masses_kg[i] + dot * step_s);
            }
        } else {
            integrated = integrator.step(
                current_extended,
                time_s,
                step_s,
                [&](double dynamics_time_s, const post2::integrators::ExtendedState& dynamics_state) {
                    const auto dynamics_command = make_engine_command(
                        phase,
                        context,
                        *throttle_model,
                        *steering_model,
                        runtime,
                        dynamics_state.motion,
                        dynamics_time_s,
                        control.engine_enabled);
                    auto eval = vehicle_propagator.compute_derivatives(
                        dynamics_time_s,
                        runtime,
                        dynamics_state.tank_masses_kg,
                        dynamics_state.motion,
                        dynamics_command);
                    auto deriv = phase_extended_dynamics(
                        simulation_config,
                        phase,
                        dynamics_state,
                        eval.thrust_acceleration_mps2,
                        eval.tank_mass_dots_kgps);
                    last_eval = std::move(eval);
                    return deriv;
                });
            if (phase.force_models.normal_force) {
                integrated.motion = post2::propagation::apply_surface_contact_constraint(
                    simulation_config, integrated.motion);
            }
        }

        const double next_time_s = time_s + step_s;
        runtime = vehicle_propagator.commit(runtime, integrated, next_time_s, last_eval, command);
        set_hold_down_clamp_state(&runtime, simulation_config, control.hold_down_clamp_active);
        apply_due_actions(actions, next_time_s - phase_start_time_s, simulation_config, &control, &runtime);
        state_log.append(runtime);
        time_s = next_time_s;
    }

    return state_log;
}

StateLog propagate_hold_down_clamp(
    const SimulationConfig& config,
    const post2::propagation::VehicleConsumptionPropagator& vehicle_propagator,
    const StateLog& initial_state_log)
{
    StateLog state_log = initial_state_log;
    if (!config.hold_down_clamp.enabled ||
        state_log.empty() ||
        state_log.back().time_s >= config.duration_s ||
        state_log.back().time_s >= config.hold_down_clamp.release_time_s) {
        return state_log;
    }

    double time_s = state_log.back().time_s;
    auto runtime = state_log.back().runtime;
    set_hold_down_clamp_state(&runtime, config, true);

    const double clamp_end_s = std::min(config.duration_s, config.hold_down_clamp.release_time_s);
    while (time_s < clamp_end_s) {
        const double next_time_s = std::min(time_s + config.step_s, clamp_end_s);
        const double step_s = next_time_s - time_s;
        const post2::propagation::EngineCommand cmd{
            runtime.engine.enabled,
            vehicle_propagator.throttle_at(time_s),
            runtime.engine.direction_body,
        };
        auto tank_masses = post2::vehicle::read_tank_masses_flat(runtime);
        auto eval = vehicle_propagator.compute_derivatives(
            time_s, runtime, tank_masses, runtime.vehicle.motion, cmd);
        // Forward Euler for tank masses while clamped; motion is overridden.
        for (std::size_t i = 0; i < tank_masses.size(); ++i) {
            const double dot = i < eval.tank_mass_dots_kgps.size()
                ? eval.tank_mass_dots_kgps[i] : 0.0;
            tank_masses[i] = std::max(0.0, tank_masses[i] + dot * step_s);
        }
        const post2::integrators::ExtendedState integrated{
            make_hold_down_clamp_state(config, next_time_s),
            std::move(tank_masses),
        };
        runtime = vehicle_propagator.commit(runtime, integrated, next_time_s, eval, cmd);
        set_hold_down_clamp_state(&runtime, config, next_time_s < config.hold_down_clamp.release_time_s);
        state_log.append(runtime);
        time_s = next_time_s;
    }

    return state_log;
}

} // namespace

StateLog GravityPropagator::propagate(const SimulationConfig& config, const StateLog& initial_state_log) const
{
    const post2::integrators::MaxTimeTerminationCondition termination{config.duration_s};
    const post2::integrators::Rk4OdeIntegrator integrator({config.step_s});
    post2::propagation::VehicleConsumptionPropagator vehicle_propagator(config.vehicle);
    StateLog state_log = propagate_hold_down_clamp(config, vehicle_propagator, initial_state_log);
    if (state_log.back().time_s >= config.duration_s || state_log.back().runtime.hold_down_clamp.active) {
        return state_log;
    }

    auto current_runtime = state_log.back().runtime;
    set_hold_down_clamp_state(&current_runtime, config, false);

    auto last_eval = std::make_shared<post2::propagation::DerivativeResult>();
    auto last_cmd = std::make_shared<post2::propagation::EngineCommand>();

    return integrator.integrate(
        state_log,
        termination,
        [&config, &vehicle_propagator, &current_runtime, last_eval, last_cmd](
            double time_s, const post2::integrators::ExtendedState& state) {
            const post2::propagation::EngineCommand cmd{
                current_runtime.engine.enabled,
                vehicle_propagator.throttle_at(time_s),
                current_runtime.engine.direction_body,
            };
            *last_cmd = cmd;
            auto eval = vehicle_propagator.compute_derivatives(
                time_s, current_runtime, state.tank_masses_kg, state.motion, cmd);
            auto deriv = inertial_extended_dynamics(
                config, state, eval.thrust_acceleration_mps2, eval.tank_mass_dots_kgps);
            *last_eval = std::move(eval);
            return deriv;
        },
        [&config, &vehicle_propagator, &current_runtime, last_eval, last_cmd](
            const post2::vehicle::VehicleRuntimeState& previous_runtime,
            const post2::integrators::ExtendedState& integrated,
            double next_time_s,
            double step_s) {
            (void)step_s;
            post2::integrators::ExtendedState constrained = integrated;
            constrained.motion = post2::propagation::apply_surface_contact_constraint(
                config, constrained.motion);
            current_runtime = vehicle_propagator.commit(
                previous_runtime, constrained, next_time_s, *last_eval, *last_cmd);
            set_hold_down_clamp_state(&current_runtime, config, false);
            return current_runtime;
        });
}

bool LaunchVehicleEvent::has_actions_before_propagation() const
{
    return false;
}

bool LaunchVehicleEvent::has_actions_after_propagation() const
{
    return false;
}

void LaunchVehicleEvent::cleanupEvent(StateLog&) const
{
}

void LaunchVehicleEvent::run_actions_after_propagation(StateLog&) const
{
}

StateLog LaunchVehicleEvent::executeEvent(const SimulationConfig& config, const StateLog& state_log) const
{
    GravityPropagator propagator;
    return propagator.propagate(config, state_log);
}

SimulationResult SimulationDriver::run(const SimulationConfig& config) const
{
    std::string error;
    if (!validate_config(config, &error)) {
        return {false, error, {}};
    }

    return run(case_from_simulation_config(config));
}

SimulationResult SimulationDriver::run(const CaseConfig& config) const
{
    std::string error;
    if (!validate_case_config(config, &error)) {
        return {false, error, {}};
    }

    try {
        StateLog state_log(config.earth_radius_m, config.vehicle);
        post2::propagation::VehicleConsumptionPropagator vehicle_propagator(config.vehicle);

        double phase_start_time_s = 0.0;
        for (std::size_t phase_index = 0; phase_index < config.phases.size(); ++phase_index) {
            const PhaseConfig& phase = config.phases[phase_index];
            const SimulationConfig simulation_config =
                make_phase_simulation_config(config, phase, phase_start_time_s + phase.duration_s);

            post2::vehicle::VehicleRuntimeState initial_runtime;
            if (phase.inherit_initial_state && !state_log.empty()) {
                initial_runtime = state_log.back().runtime;
            } else if (phase.initial_state_eci.has_value()) {
                initial_runtime = vehicle_propagator.make_initial_state(*phase.initial_state_eci, phase_start_time_s);
            } else if (phase_index == 0) {
                const State initial_state = phase.hold_down_clamp_initial_active
                    ? make_hold_down_clamp_state(simulation_config, phase_start_time_s)
                    : make_default_leo_state(simulation_config);
                initial_runtime = vehicle_propagator.make_initial_state(initial_state, phase_start_time_s);
            } else {
                return {false, "phase " + std::to_string(phase_index) + " has no initial state", state_log};
            }

            const StateLog phase_log =
                propagate_phase(config, phase, phase_index, phase_start_time_s, initial_runtime);
            merge_phase_log(&state_log, phase_log);
            phase_start_time_s += phase.duration_s;
        }

        if (case_vehicle_impacted_earth(state_log, config)) {
            return {false, "vehicle impacted Earth during propagation", state_log};
        }

        return {true, "", state_log};
    } catch (const std::exception& ex) {
        return {false, ex.what(), {}};
    }
}

StateLog SimulationDriver::initialize_or_truncate_state_log(const SimulationConfig& config) const
{
    post2::propagation::VehicleConsumptionPropagator vehicle_propagator(config.vehicle);
    StateLog state_log(config.earth_radius_m, config.vehicle);
    const State initial_state = config.hold_down_clamp.enabled
        ? make_hold_down_clamp_state(config, 0.0)
        : make_default_leo_state(config);
    auto runtime = vehicle_propagator.make_initial_state(initial_state, 0.0);
    set_hold_down_clamp_state(
        &runtime,
        config,
        config.hold_down_clamp.enabled && config.hold_down_clamp.release_time_s > 0.0);
    state_log.append(runtime);
    state_log.truncate_after(config.duration_s);
    return state_log;
}

StateLog SimulationDriver::integrateOneEvent(
    const LaunchVehicleEvent& event,
    const SimulationConfig& config,
    const StateLog& state_log) const
{
    return event.executeEvent(config, state_log);
}

} // namespace post2::core
