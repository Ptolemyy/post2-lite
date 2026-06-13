#include "post2/core/simulation_driver.hpp"

#include "post2/core/control_models.hpp"
#include "post2/core/coordinates.hpp"
#include "post2/core/frames.hpp"
#include "post2/environment/atmosphere.hpp"
#include "post2/integrators/dopri5.hpp"
#include "post2/integrators/integrator.hpp"
#include "post2/integrators/ode_integrator.hpp"
#include "post2/propagation/builtin_events.hpp"
#include "post2/propagation/force_model_set.hpp"
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

double minimum_effective_step_s(double nominal_step_s)
{
    const double scaled = (std::isfinite(nominal_step_s) && nominal_step_s > 0.0)
        ? nominal_step_s * 1.0e-6
        : 1.0e-6;
    return std::max(1.0e-9, std::min(1.0e-6, scaled));
}

int max_phase_integration_steps(double duration_s, double nominal_step_s)
{
    if (!std::isfinite(duration_s) || duration_s <= 0.0) {
        return 1000;
    }
    const double nominal = (std::isfinite(nominal_step_s) && nominal_step_s > 0.0)
        ? nominal_step_s
        : 1.0;
    const double expected_steps = std::ceil(duration_s / nominal);
    const double budget = 100.0 + 50.0 * expected_steps;
    return static_cast<int>(std::clamp(budget, 1000.0, 100000.0));
}

Vec3 cross_product(const Vec3& lhs, const Vec3& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

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

post2::propagation::EnvironmentState make_environment_state(
    const SimulationConfig& config,
    const PhaseConfig& phase,
    double time_s,
    const State& motion)
{
    post2::propagation::EnvironmentState environment;
    environment.time_s = time_s;
    environment.position_eci_m = motion.position_m;
    environment.velocity_eci_mps = motion.velocity_mps;
    const double theta_rad = frames::earth_rotation_angle_rad(
        config.earth_rotation_at_epoch_rad,
        config.earth_rotation_rad_per_s,
        time_s);
    const frames::EcefState ecef_state = frames::eci_to_ecef_state(
        motion.position_m,
        motion.velocity_mps,
        theta_rad,
        config.earth_rotation_rad_per_s);
    environment.position_ecef_m = ecef_state.position_m;
    environment.velocity_ecef_mps = ecef_state.velocity_mps;
    environment.radius_m = post2::vehicle::norm(motion.position_m);
    const frames::Geodetic geo = frames::ecef_to_geodetic(environment.position_ecef_m);
    environment.altitude_m = geo.altitude_m;
    environment.latitude_rad = geo.latitude_rad;
    environment.longitude_rad = geo.longitude_rad;
    if (phase.force_models.atmosphere_model.type == "exponential") {
        const post2::environment::ExponentialAtmosphereModel atmosphere(config.earth_radius_m);
        const post2::environment::AtmosphereSample sample =
            atmosphere.sample(time_s, motion.position_m, motion.velocity_mps);
        environment.density_kgpm3 = sample.density_kgpm3;
        environment.pressure_pa = sample.pressure_pa;
        environment.temperature_k = sample.temperature_k;
        environment.speed_of_sound_mps = sample.speed_of_sound_mps;
        environment.wind_ecef_mps = sample.wind_ecef_mps;
    }
    environment.wind_eci_mps = frames::ecef_to_eci_position(environment.wind_ecef_mps, theta_rad);
    return environment;
}

void append_entry_with_environment(
    StateLog* state_log,
    const post2::vehicle::VehicleRuntimeState& runtime,
    const post2::propagation::EnvironmentState& env,
    double earth_rotation_rad_per_s,
    const Vec3& acceleration_eci_mps2 = {0.0, 0.0, 0.0})
{
    LaunchVehicleStateLogEntry entry = state_log->build_entry(runtime);
    entry.acceleration_eci_mps2 = acceleration_eci_mps2;
    entry.acceleration_mps2 = post2::vehicle::norm(acceleration_eci_mps2);
    entry.ambient_pressure_pa = env.pressure_pa;
    entry.atmosphere_density_kgpm3 = env.density_kgpm3;
    // Atmosphere rotates with Earth: v_rel = v_eci - omega x r - wind_eci.
    const Vec3 atmosphere_v_eci =
        cross_product({0.0, 0.0, earth_rotation_rad_per_s}, runtime.vehicle.motion.position_m);
    const Vec3 v_rel = runtime.vehicle.motion.velocity_mps - atmosphere_v_eci - env.wind_eci_mps;
    const double v_rel_mag = post2::vehicle::norm(v_rel);
    entry.dynamic_pressure_pa = 0.5 * env.density_kgpm3 * v_rel_mag * v_rel_mag;
    entry.mach_number = env.speed_of_sound_mps > 0.0
        ? v_rel_mag / env.speed_of_sound_mps : 0.0;
    state_log->append(entry);
}

post2::integrators::ExtendedDerivative phase_extended_dynamics(
    const post2::propagation::ForceModelSet& force_models,
    const post2::propagation::ForceModelContext& force_context,
    const post2::integrators::ExtendedState& state,
    std::vector<double> tank_mass_dots_kgps)
{
    const post2::propagation::ForceModelOutput force_output =
        force_models.evaluate_all(force_context, state);

    return {
        {state.motion.velocity_mps, force_output.acceleration_eci_mps2},
        std::move(tank_mass_dots_kgps),
    };
}

bool supported_gravity_model_type(const std::string& type)
{
    return type == "point_mass" || type == "j2" || type == "spherical_harmonic";
}

bool supported_atmosphere_model_type(const std::string& type)
{
    return type == "none" || type == "exponential";
}

bool validate_gravity_model_config(
    const GravityModelConfig& gravity_model,
    const std::string& prefix,
    std::string* error)
{
    if (!supported_gravity_model_type(gravity_model.type)) {
        *error = prefix + ".type must be \"point_mass\", \"j2\", or \"spherical_harmonic\"";
        return false;
    }
    if (gravity_model.j2 < 0.0) {
        *error = prefix + ".j2 cannot be negative";
        return false;
    }
    if (gravity_model.degree < 0 || gravity_model.order < 0) {
        *error = prefix + ".degree and .order cannot be negative";
        return false;
    }
    if (gravity_model.order > gravity_model.degree) {
        *error = prefix + ".order cannot exceed .degree";
        return false;
    }
    return true;
}

bool validate_atmosphere_model_config(
    const AtmosphereModelConfig& atmosphere_model,
    const std::string& prefix,
    std::string* error)
{
    if (!supported_atmosphere_model_type(atmosphere_model.type)) {
        *error = prefix + ".type must be \"none\" or \"exponential\"";
        return false;
    }
    return true;
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
    if (config.earth_j2 < 0.0) {
        *error = "earth_j2 cannot be negative";
        return false;
    }
    if (config.earth_rotation_rad_per_s < 0.0) {
        *error = "earth_rotation_rad_per_s cannot be negative";
        return false;
    }
    if (!validate_gravity_model_config(config.gravity_model, "gravity_model", error)) {
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
        // altitude_m is WGS84 geodetic; below ellipsoid means below ground.
        if (entry.altitude_m < -tolerance_m) {
            return true;
        }
        if (entry.altitude_m <= 0.0 &&
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
    config.epoch_utc = case_config.epoch_utc;
    config.earth_rotation_at_epoch_rad = case_config.earth_rotation_at_epoch_rad;
    config.duration_s = phase_end_time_s;
    config.step_s = case_config.step_s;
    config.launch_site = case_config.launch_site;
    config.earth_j2 = case_config.earth_j2;
    config.gravity_model = phase.force_models.gravity_model;
    if (config.gravity_model.j2 == kEarthJ2 && config.earth_j2 != kEarthJ2) {
        config.gravity_model.j2 = config.earth_j2;
    }
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
    if (config.earth_j2 < 0.0) {
        *error = "earth_j2 cannot be negative";
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
        if (phase.integrator != "ode" &&
            phase.integrator != "rk4" &&
            phase.integrator != "dopri5") {
            *error = "phase " + std::to_string(i) +
                " integrator must be \"rk4\", \"dopri5\", or legacy \"ode\"";
            return false;
        }
        if (phase.tolerances.rtol <= 0.0 ||
            phase.tolerances.atol_position_m <= 0.0 ||
            phase.tolerances.atol_velocity_mps <= 0.0 ||
            phase.tolerances.atol_tank_mass_kg <= 0.0) {
            *error = "phase " + std::to_string(i) + " tolerances must be positive";
            return false;
        }
        if (!validate_gravity_model_config(
                phase.force_models.gravity_model,
                "phase " + std::to_string(i) + " force_models.gravity_model",
                error)) {
            return false;
        }
        if (!validate_atmosphere_model_config(
                phase.force_models.atmosphere_model,
                "phase " + std::to_string(i) + " force_models.atmosphere_model",
                error)) {
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
        if (entry.altitude_m < -tolerance_m) {
            return true;
        }
        const bool phase_normal_force_enabled =
            entry.phase_index >= 0 &&
            static_cast<std::size_t>(entry.phase_index) < config.phases.size()
                ? config.phases[static_cast<std::size_t>(entry.phase_index)].force_models.normal_force
                : true;
        if (entry.altitude_m <= 0.0 &&
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
    auto integrator = post2::integrators::make_integrator(
        phase.integrator, case_config.step_s, phase.tolerances);
    const auto throttle_model = make_throttle_model(phase.throttle_model);
    const auto steering_model = make_steering_model(phase.steering_model);
    const PhaseContext context{&case_config, &phase, phase_index, phase_start_time_s};
    const post2::propagation::ForceModelSet force_models =
        post2::propagation::make_force_model_set(case_config, phase);
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
    {
        const auto env_initial = make_environment_state(
            simulation_config, phase, runtime.time_s, runtime.vehicle.motion);
        append_entry_with_environment(
            &state_log, runtime, env_initial, simulation_config.earth_rotation_rad_per_s);
    }

    const bool use_adaptive_step_suggestions = phase.integrator == "dopri5";
    const double min_effective_step_s = minimum_effective_step_s(case_config.step_s);
    const int max_step_count =
        max_phase_integration_steps(phase.duration_s, case_config.step_s);
    int step_count = 0;
    double time_s = phase_start_time_s;
    double suggested_step_s = case_config.step_s;
    while (time_s < phase_end_time_s) {
        ++step_count;
        if (step_count > max_step_count) {
            throw std::runtime_error("integrator exceeded phase step budget");
        }

        const double phase_time_s = time_s - phase_start_time_s;
        apply_due_actions(actions, phase_time_s, simulation_config, &control, &runtime);

        const double next_action_absolute_s = phase_start_time_s + next_action_time_s(actions, control);
        const double requested_step_s =
            (std::isfinite(suggested_step_s) && suggested_step_s > 1.0e-12)
                ? suggested_step_s
                : case_config.step_s;
        double step_s = std::min(requested_step_s, phase_end_time_s - time_s);
        if (next_action_absolute_s > time_s && next_action_absolute_s < time_s + step_s) {
            step_s = next_action_absolute_s - time_s;
        }
        if (step_s <= 1.0e-12) {
            step_s = std::min(case_config.step_s, phase_end_time_s - time_s);
        }

        const State current_state = runtime.vehicle.motion;
        auto command = make_engine_command(
            phase,
            context,
            *throttle_model,
            *steering_model,
            runtime,
            current_state,
            time_s,
            control.engine_enabled);
        // Pre-sample environment at the start-of-step state so engine
        // performance (pressure correction) and downstream force models see a
        // consistent ambient pressure.
        {
            const post2::propagation::EnvironmentState env_at_start =
                make_environment_state(simulation_config, phase, time_s, current_state);
            command.ambient_pressure_pa = env_at_start.pressure_pa;
        }

        post2::integrators::ExtendedState current_extended{
            current_state,
            post2::vehicle::read_tank_masses_flat(runtime),
        };

        post2::propagation::DerivativeResult last_eval;
        Vec3 last_acceleration_eci_mps2{0.0, 0.0, 0.0};
        post2::integrators::ExtendedState integrated;
        bool altitude_zero_event_hit = false;
        if (control.hold_down_clamp_active) {
            // Engine still burns while clamped: integrate tank masses with
            // forward Euler (motion pinned to launch site rotates with Earth
            // and is overridden at the end of the step regardless).
            last_eval = vehicle_propagator.compute_derivatives(
                time_s, runtime, current_extended.tank_masses_kg, current_state, command);
            last_acceleration_eci_mps2 = {0.0, 0.0, 0.0};
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
            // Tank-empty events: any tank currently above the threshold
            // gets a g(t,state) = mass - eps watcher. When the integrator
            // detects a crossing it truncates the step at the event time so
            // we can hard-clamp the tank to 0 below.
            constexpr double kTankEpsKg = 1.0e-3;
            std::vector<post2::integrators::EventFunction> events;
            std::vector<std::size_t> event_tank_indices;
            events.reserve(current_extended.tank_masses_kg.size());
            event_tank_indices.reserve(current_extended.tank_masses_kg.size());
            for (std::size_t i = 0; i < current_extended.tank_masses_kg.size(); ++i) {
                if (current_extended.tank_masses_kg[i] > kTankEpsKg) {
                    post2::integrators::EventFunction ev;
                    ev.name = "tank_empty_" + std::to_string(i);
                    ev.terminating = true;
                    const std::size_t flat = i;
                    ev.g = [flat](double, const post2::integrators::ExtendedState& s) {
                        return (flat < s.tank_masses_kg.size() ? s.tank_masses_kg[flat] : 0.0)
                            - kTankEpsKg;
                    };
                    events.push_back(std::move(ev));
                    event_tank_indices.push_back(i);
                }
            }
            const post2::propagation::EnvironmentState env_for_events =
                make_environment_state(simulation_config, phase, time_s, current_state);
            if (!phase.force_models.normal_force && env_for_events.altitude_m > 1.0) {
                events.push_back(post2::propagation::altitude_zero_event(true));
            }

            const post2::integrators::StepResult step_result = integrator->step(
                current_extended,
                time_s,
                step_s,
                [&](double dynamics_time_s, const post2::integrators::ExtendedState& dynamics_state) {
                    // Environment first so engine performance can read ambient
                    // pressure for the pressure-correction term.
                    const post2::propagation::EnvironmentState environment =
                        make_environment_state(
                            simulation_config,
                            phase,
                            dynamics_time_s,
                            dynamics_state.motion);
                    auto dynamics_command = make_engine_command(
                        phase,
                        context,
                        *throttle_model,
                        *steering_model,
                        runtime,
                        dynamics_state.motion,
                        dynamics_time_s,
                        control.engine_enabled);
                    dynamics_command.ambient_pressure_pa = environment.pressure_pa;
                    auto eval = vehicle_propagator.compute_derivatives(
                        dynamics_time_s,
                        runtime,
                        dynamics_state.tank_masses_kg,
                        dynamics_state.motion,
                        dynamics_command);
                    const post2::propagation::ForceModelContext force_context{
                        &case_config,
                        &phase,
                        &runtime,
                        &environment,
                        eval.thrust_acceleration_mps2,
                    };
                    auto deriv = phase_extended_dynamics(
                        force_models,
                        force_context,
                        dynamics_state,
                        eval.tank_mass_dots_kgps);
                    last_acceleration_eci_mps2 = deriv.motion_dot.d_velocity_mps2;
                    last_eval = std::move(eval);
                    return deriv;
                },
                events);
            if (!step_result.accepted || step_result.h_used <= 1.0e-12) {
                throw std::runtime_error("integrator failed to make progress");
            }
            if (!step_result.event.has_value() &&
                step_result.h_used < min_effective_step_s &&
                phase_end_time_s - (time_s + step_result.h_used) > min_effective_step_s) {
                throw std::runtime_error("integrator step size below minimum progress");
            }
            integrated = step_result.state_end;
            // Reflect the (possibly shortened) step used by the integrator.
            step_s = step_result.h_used;
            suggested_step_s =
                (use_adaptive_step_suggestions &&
                 std::isfinite(step_result.h_next_suggested) &&
                 step_result.h_next_suggested > 1.0e-12)
                    ? step_result.h_next_suggested
                    : case_config.step_s;
            // If a tank-empty event fired, clamp that tank to exactly zero
            // so the next step's gating logic sees it as drained.
            if (step_result.event.has_value()) {
                altitude_zero_event_hit = step_result.event->name == "altitude_zero";
                const std::size_t event_idx = step_result.event->event_index;
                if (event_idx < event_tank_indices.size()) {
                    const std::size_t tank_idx = event_tank_indices[event_idx];
                    if (tank_idx < integrated.tank_masses_kg.size()) {
                        integrated.tank_masses_kg[tank_idx] = 0.0;
                    }
                }
            }
            if (phase.force_models.normal_force) {
                integrated.motion = post2::propagation::apply_surface_contact_constraint(
                    simulation_config, integrated.motion);
            }
        }

        const double next_time_s = time_s + step_s;
        runtime = vehicle_propagator.commit(runtime, integrated, next_time_s, last_eval, command);
        set_hold_down_clamp_state(&runtime, simulation_config, control.hold_down_clamp_active);
        apply_due_actions(actions, next_time_s - phase_start_time_s, simulation_config, &control, &runtime);
        {
            const auto env_after_step = make_environment_state(
                simulation_config, phase, next_time_s, runtime.vehicle.motion);
            append_entry_with_environment(
                &state_log,
                runtime,
                env_after_step,
                simulation_config.earth_rotation_rad_per_s,
                last_acceleration_eci_mps2);
        }
        time_s = next_time_s;
        if (altitude_zero_event_hit) {
            break;
        }
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
    PhaseConfig hdc_phase;
    while (time_s < clamp_end_s) {
        const double next_time_s = std::min(time_s + config.step_s, clamp_end_s);
        const double step_s = next_time_s - time_s;
        const post2::propagation::EnvironmentState env =
            make_environment_state(config, hdc_phase, time_s, runtime.vehicle.motion);
        post2::propagation::EngineCommand cmd{
            runtime.engine.enabled,
            vehicle_propagator.throttle_at(time_s),
            runtime.engine.direction_body,
        };
        cmd.ambient_pressure_pa = env.pressure_pa;
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
        {
            const auto env_after = make_environment_state(
                config, hdc_phase, next_time_s, runtime.vehicle.motion);
            append_entry_with_environment(
                &state_log, runtime, env_after, config.earth_rotation_rad_per_s);
        }
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
    const CaseConfig case_config = case_from_simulation_config(config);
    const PhaseConfig& phase = case_config.phases.front();
    const post2::propagation::ForceModelSet force_models =
        post2::propagation::make_force_model_set(case_config, phase);

    return integrator.integrate(
        state_log,
        termination,
        [&config, &case_config, &phase, &force_models, &vehicle_propagator, &current_runtime, last_eval, last_cmd](
            double time_s, const post2::integrators::ExtendedState& state) {
            const post2::propagation::EnvironmentState environment =
                make_environment_state(config, phase, time_s, state.motion);
            post2::propagation::EngineCommand cmd{
                current_runtime.engine.enabled,
                vehicle_propagator.throttle_at(time_s),
                current_runtime.engine.direction_body,
            };
            cmd.ambient_pressure_pa = environment.pressure_pa;
            *last_cmd = cmd;
            auto eval = vehicle_propagator.compute_derivatives(
                time_s, current_runtime, state.tank_masses_kg, state.motion, cmd);
            const post2::propagation::ForceModelContext force_context{
                &case_config,
                &phase,
                &current_runtime,
                &environment,
                eval.thrust_acceleration_mps2,
            };
            auto deriv = phase_extended_dynamics(
                force_models,
                force_context,
                state,
                eval.tank_mass_dots_kgps);
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
            if (case_vehicle_impacted_earth(state_log, config)) {
                return {false, "vehicle impacted Earth during propagation", state_log};
            }
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
