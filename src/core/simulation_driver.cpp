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
#include <cctype>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace post2::core {

namespace {

std::string lowercase(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

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

// Upper-bound time horizon for non-time terminations (24h). Phases whose
// termination event has not fired by then are stopped by the budget guard.
constexpr double kMaxPhaseTimeS = 86400.0;
constexpr double kStandardGravityMps2 = 9.80665;

// Returns true iff the trigger condition's quantity is one that depends only
// on integrator-state (i.e. can be evaluated by an EventFunction).
bool trigger_uses_event_path(const TriggerCondition& trigger)
{
    return trigger.type == "altitude_m" ||
        trigger.type == "velocity_mps" ||
        trigger.type == "total_mass_kg" ||
        trigger.type == "propellant_mass_kg" ||
        trigger.type == "apoapsis_altitude_m" ||
        trigger.type == "periapsis_altitude_m" ||
        trigger.type == "orbital_energy" ||
        trigger.type == "sma_m" ||
        trigger.type == "thrust_fraction" ||
        trigger.type == "time";
}

// Specific orbital quantity from an inertial state, used by the orbital-element
// phase/event triggers. Mirrors orbital_elements_for_entry in nlp_evaluator.cpp
// but evaluable mid-integration from (r, v) alone so it can drive an
// EventFunction. apoapsis on an unbound (e >= 1) osculating orbit is reported as
// +infinity (any finite ">=" target is already exceeded).
double orbital_trigger_quantity(
    const std::string& type,
    const Vec3& r,
    const Vec3& v,
    double mu_m3s2,
    double earth_radius_m)
{
    const double r_norm = post2::vehicle::norm(r);
    const double v_norm = post2::vehicle::norm(v);
    if (r_norm <= 0.0 || mu_m3s2 <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (type == "orbital_energy") {
        return 0.5 * v_norm * v_norm - mu_m3s2 / r_norm;
    }
    if (type == "sma_m") {
        const double denom = 2.0 / r_norm - v_norm * v_norm / mu_m3s2;
        return std::abs(denom) > 0.0 ? 1.0 / denom
                                     : std::numeric_limits<double>::quiet_NaN();
    }
    const Vec3 h{
        r.y * v.z - r.z * v.y,
        r.z * v.x - r.x * v.z,
        r.x * v.y - r.y * v.x,
    };
    const double h_norm = post2::vehicle::norm(h);
    if (h_norm <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double rv = post2::vehicle::dot(r, v);
    const Vec3 e_vec = (r * (v_norm * v_norm - mu_m3s2 / r_norm) - v * rv) / mu_m3s2;
    const double e = post2::vehicle::norm(e_vec);
    const double p = h_norm * h_norm / mu_m3s2;
    if (type == "periapsis_altitude_m") {
        return (1.0 + e > 0.0) ? p / (1.0 + e) - earth_radius_m
                               : std::numeric_limits<double>::quiet_NaN();
    }
    // apoapsis_altitude_m
    if (e >= 1.0) {
        return std::numeric_limits<double>::infinity();
    }
    return p / (1.0 - e) - earth_radius_m;
}

// Builds an EventFunction whose g changes sign at the trigger's rising edge
// (relative to the comparison direction). For mass-related triggers,
// `runtime_snapshot` captures the attached / active stage set at the moment
// the event is registered; re-register fresh after any staging change.
//
// `time_base_s` is subtracted from t for type=="time" triggers — pass the
// phase start time for phase-relative semantics or 0 for mission-absolute.
// `mu_m3s2` / `earth_radius_m` are only consulted by the orbital-element
// trigger types (apoapsis/periapsis/energy/sma).
post2::integrators::EventFunction make_trigger_event(
    const TriggerCondition& trigger,
    const post2::vehicle::VehicleRuntimeState& runtime_snapshot,
    double mu_m3s2,
    double earth_radius_m,
    double time_base_s,
    bool terminating,
    const std::string& name)
{
    const bool comparison_ge = trigger.comparison == ">=";
    const double threshold = trigger.value;
    auto wrap = [comparison_ge, threshold](double quantity) {
        return comparison_ge ? (quantity - threshold) : (threshold - quantity);
    };

    post2::integrators::EventFunction ev;
    ev.name = name;
    ev.terminating = terminating;
    ev.direction = +1;

    if (trigger.type == "time") {
        ev.g = [wrap, time_base_s](double t, const post2::integrators::ExtendedState&) {
            return wrap(t - time_base_s);
        };
        return ev;
    }
    if (trigger.type == "altitude_m") {
        ev.g = [wrap](double, const post2::integrators::ExtendedState& s) {
            return wrap(post2::core::frames::ecef_to_geodetic(s.motion.position_m).altitude_m);
        };
        return ev;
    }
    if (trigger.type == "velocity_mps") {
        ev.g = [wrap](double, const post2::integrators::ExtendedState& s) {
            return wrap(post2::vehicle::norm(s.motion.velocity_mps));
        };
        return ev;
    }
    if (trigger.type == "total_mass_kg") {
        double dry_mass_attached_kg = 0.0;
        std::vector<std::size_t> attached_tank_flat_indices;
        for (std::size_t i = 0; i < runtime_snapshot.stages.size(); ++i) {
            if (!runtime_snapshot.stages[i].attached) {
                continue;
            }
            dry_mass_attached_kg += runtime_snapshot.stages[i].dry_mass_kg;
            for (std::size_t j = 0; j < runtime_snapshot.stages[i].tanks.size(); ++j) {
                attached_tank_flat_indices.push_back(
                    post2::vehicle::flat_tank_index(runtime_snapshot, i, j));
            }
        }
        ev.g = [wrap, dry_mass_attached_kg, idx = std::move(attached_tank_flat_indices)](
                   double, const post2::integrators::ExtendedState& s) {
            double total = dry_mass_attached_kg;
            for (std::size_t i : idx) {
                if (i < s.tank_masses_kg.size()) {
                    total += std::max(0.0, s.tank_masses_kg[i]);
                }
            }
            return wrap(total);
        };
        return ev;
    }
    if (trigger.type == "propellant_mass_kg") {
        std::vector<std::size_t> indices;
        for (std::size_t i = 0; i < runtime_snapshot.stages.size(); ++i) {
            if (!runtime_snapshot.stages[i].attached || !runtime_snapshot.stages[i].active) {
                continue;
            }
            for (std::size_t j = 0; j < runtime_snapshot.stages[i].tanks.size(); ++j) {
                indices.push_back(post2::vehicle::flat_tank_index(runtime_snapshot, i, j));
            }
        }
        ev.g = [wrap, idx = std::move(indices)](
                   double, const post2::integrators::ExtendedState& s) {
            double total = 0.0;
            for (std::size_t i : idx) {
                if (i < s.tank_masses_kg.size()) {
                    total += std::max(0.0, s.tank_masses_kg[i]);
                }
            }
            return wrap(total);
        };
        return ev;
    }

    if (trigger.type == "apoapsis_altitude_m" ||
        trigger.type == "periapsis_altitude_m" ||
        trigger.type == "orbital_energy" ||
        trigger.type == "sma_m") {
        ev.g = [wrap, type = trigger.type, mu_m3s2, earth_radius_m](
                   double, const post2::integrators::ExtendedState& s) {
            return wrap(orbital_trigger_quantity(
                type, s.motion.position_m, s.motion.velocity_mps, mu_m3s2, earth_radius_m));
        };
        return ev;
    }

    // "thrust_fraction" depends on engine spool state, which is not part of the
    // integrated ExtendedState, so it cannot drive an integrator event. It is
    // handled out-of-band by trigger_condition_is_satisfied at step boundaries;
    // here we degenerate to a never-firing event (same as unknown types).
    ev.g = [](double, const post2::integrators::ExtendedState&) { return -1.0; };
    return ev;
}

// Current thrust-establishment fraction: aggregate actual (spooled) thrust over
// aggregate steady (commanded) thrust. 0 when nothing is commanded to fire.
// Used by the "thrust_fraction" termination, which the integrator cannot
// evaluate (it depends on engine spool state, not the integrated motion/mass).
double current_thrust_fraction(const post2::vehicle::VehicleRuntimeState& runtime)
{
    const double commanded = runtime.engine.commanded_thrust_n;
    if (commanded <= 0.0) {
        return 0.0;
    }
    return std::max(0.0, runtime.engine.actual_thrust_n) / commanded;
}

bool trigger_condition_is_satisfied(
    const TriggerCondition& trigger,
    const post2::vehicle::VehicleRuntimeState& runtime,
    double mu_m3s2,
    double earth_radius_m,
    double time_base_s)
{
    if (trigger.type == "thrust_fraction") {
        const double fraction = current_thrust_fraction(runtime);
        return trigger.comparison == ">=" ? fraction >= trigger.value
                                          : fraction <= trigger.value;
    }

    post2::integrators::EventFunction probe = make_trigger_event(
        trigger,
        runtime,
        mu_m3s2,
        earth_radius_m,
        time_base_s,
        /*terminating=*/false,
        "trigger_probe");
    if (!probe.g) {
        return false;
    }

    const post2::integrators::ExtendedState state{
        runtime.vehicle.motion,
        post2::vehicle::read_tank_masses_flat(runtime),
    };
    const double g = probe.g(runtime.time_s, state);
    return g >= 0.0;
}

// Per-mission-event runtime tracking. prev_g_sign is the sign of g at the
// previous step boundary (post-action). Initialised to a negative baseline
// so a trigger that is already satisfied at t=0 does not spuriously refire
// on its first evaluation.
struct MissionEventRuntime {
    double prev_g = -1.0;
    int fired_count = 0;
};

struct MissionEventsState {
    std::vector<MissionEventRuntime> runtimes;
};

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
    {
        const std::string& atmo_type = phase.force_models.atmosphere_model.type;
        post2::environment::AtmosphereSample sample;
        bool sampled = false;
        if (atmo_type == "us_standard_1976") {
            const post2::environment::UsStandardAtmosphere1976Model atmosphere(config.earth_radius_m);
            sample = atmosphere.sample(time_s, motion.position_m, motion.velocity_mps);
            sampled = true;
        } else if (atmo_type == "table") {
            // Load once and cache by path (this runs every integration step).
            static std::string cached_path;
            static post2::environment::TabulatedAtmosphereModel cached_model(
                config.earth_radius_m, {}, {});
            static bool cached_ok = false;
            const std::string& path = phase.force_models.atmosphere_model.table_path;
            if (path != cached_path) {
                cached_path = path;
                std::string err;
                cached_ok = post2::environment::TabulatedAtmosphereModel::load_csv(
                    path, config.earth_radius_m, &cached_model, &err);
            }
            if (cached_ok) {
                sample = cached_model.sample(time_s, motion.position_m, motion.velocity_mps);
                sampled = true;
            }
        }
        if (!sampled && (atmo_type == "exponential" || atmo_type == "us_standard_1976" ||
                         atmo_type == "table")) {
            const post2::environment::ExponentialAtmosphereModel atmosphere(config.earth_radius_m);
            sample = atmosphere.sample(time_s, motion.position_m, motion.velocity_mps);
            sampled = true;
        }
        if (sampled) {
            environment.density_kgpm3 = sample.density_kgpm3;
            environment.pressure_pa = sample.pressure_pa;
            environment.temperature_k = sample.temperature_k;
            environment.speed_of_sound_mps = sample.speed_of_sound_mps;
            environment.wind_ecef_mps = sample.wind_ecef_mps;
        }
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
    return type == "none" || type == "exponential" ||
           type == "us_standard_1976" || type == "table";
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
        *error = prefix +
            ".type must be \"none\", \"exponential\", \"us_standard_1976\", or \"table\"";
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
        if (phase.termination.type == "time" && phase.termination.value <= 0.0) {
            *error = "phase " + std::to_string(i) + " termination.value must be positive for time-based termination";
            return false;
        }
        if (phase.integrator != "ode" &&
            phase.integrator != "rk4" &&
            phase.integrator != "dopri5" &&
            phase.integrator != "dop853") {
            *error = "phase " + std::to_string(i) +
                " integrator must be \"rk4\", \"dopri5\", \"dop853\", or legacy \"ode\"";
            return false;
        }
        if (phase.tolerances.rtol <= 0.0 ||
            phase.tolerances.atol_position_m <= 0.0 ||
            phase.tolerances.atol_velocity_mps <= 0.0 ||
            phase.tolerances.atol_tank_mass_kg <= 0.0) {
            *error = "phase " + std::to_string(i) + " tolerances must be positive";
            return false;
        }
        if (!trigger_uses_event_path(phase.termination)) {
            *error = "phase " + std::to_string(i) +
                " termination.type unrecognised: " + phase.termination.type;
            return false;
        }
        if (phase.termination.comparison != ">=" && phase.termination.comparison != "<=") {
            *error = "phase " + std::to_string(i) +
                " termination.comparison must be \">=\" or \"<=\"";
            return false;
        }
        if (phase.termination.type == "thrust_fraction" &&
            (phase.termination.value <= 0.0 || phase.termination.value > 1.0)) {
            *error = "phase " + std::to_string(i) +
                " termination.value must be in (0, 1] for thrust_fraction";
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
        const bool time_terminated = phase.termination.type == "time";
        for (const auto& action : phase.actions) {
            if (action.time_s < 0.0) {
                *error = "phase " + std::to_string(i) + " action time_s must be non-negative";
                return false;
            }
            if (time_terminated && action.time_s > phase.termination.value) {
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
    for (std::size_t i = 0; i < config.events.size(); ++i) {
        const EventConfig& event = config.events[i];
        if (!trigger_uses_event_path(event.trigger)) {
            *error = "event " + std::to_string(i) +
                " trigger.type unrecognised: " + event.trigger.type;
            return false;
        }
        if (event.trigger.comparison != ">=" && event.trigger.comparison != "<=") {
            *error = "event " + std::to_string(i) +
                " trigger.comparison must be \">=\" or \"<=\"";
            return false;
        }
        for (const auto& action : event.actions) {
            if (!action_type_is_supported(action.type)) {
                *error = "unsupported event action: " + action.type;
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

bool is_powered_upfg_phase(
    const PhaseConfig& phase,
    const RuntimeControl& control,
    const std::vector<TimedAction>& actions)
{
    return lowercase(phase.steering_model.type) == "upfg" &&
        phase.force_models.thrust &&
        control.engine_enabled &&
        control.next_action_index >= actions.size();
}

double active_propulsive_burn_time_s(
    const CaseConfig& case_config,
    const post2::vehicle::VehicleRuntimeState& runtime)
{
    const std::vector<post2::vehicle::StageConfig> stage_configs =
        post2::vehicle::effective_stage_configs(case_config.vehicle);
    double propellant_kg = 0.0;
    double mdot_kgps = 0.0;
    for (std::size_t i = 0; i < stage_configs.size() && i < runtime.stages.size(); ++i) {
        const auto& stage_rt = runtime.stages[i];
        const auto& stage_cfg = stage_configs[i];
        if (!stage_rt.attached ||
            !stage_rt.active ||
            !stage_rt.engine.enabled ||
            !stage_cfg.engine.enabled ||
            stage_cfg.engine.thrust_vac_n <= 0.0 ||
            stage_cfg.engine.isp_vac_s <= 0.0) {
            continue;
        }
        const double thrust_n =
            std::max(0, stage_cfg.engine.engine_count) * stage_cfg.engine.thrust_vac_n;
        const double stage_mdot_kgps =
            thrust_n / (stage_cfg.engine.isp_vac_s * kStandardGravityMps2);
        if (stage_mdot_kgps <= 0.0) {
            continue;
        }
        propellant_kg += post2::vehicle::stage_propellant_kg(stage_rt);
        mdot_kgps += stage_mdot_kgps;
    }

    if (propellant_kg <= 0.0 || mdot_kgps <= 0.0) {
        return 0.0;
    }
    return propellant_kg / mdot_kgps;
}

bool is_segmented_poly_type(const std::string& type)
{
    const std::string normalized = lowercase(type);
    return normalized == "segmented_poly" ||
        normalized == "generic_segmented_poly" ||
        normalized == "piecewise_poly";
}

void add_control_switch_action(std::vector<TimedAction>* actions, double phase_time_s)
{
    if (!actions || !std::isfinite(phase_time_s) || phase_time_s <= 0.0) {
        return;
    }
    PhaseAction action;
    action.time_s = phase_time_s;
    action.type = "control_switch";
    actions->push_back({action});
}

std::vector<TimedAction> sorted_actions(const PhaseConfig& phase)
{
    std::vector<TimedAction> actions;
    actions.reserve(
        phase.actions.size() +
        phase.throttle_model.segmented_poly.segments.size() +
        phase.steering_model.segmented_poly.segments.size());
    for (const auto& action : phase.actions) {
        actions.push_back({action});
    }
    if (is_segmented_poly_type(phase.throttle_model.type)) {
        for (const auto& segment : phase.throttle_model.segmented_poly.segments) {
            add_control_switch_action(&actions, segment.start_time_s);
        }
    }
    if (is_segmented_poly_type(phase.steering_model.type)) {
        for (const auto& segment : phase.steering_model.segmented_poly.segments) {
            add_control_switch_action(&actions, segment.start_time_s);
        }
    }
    std::sort(actions.begin(), actions.end(), [](const TimedAction& lhs, const TimedAction& rhs) {
        return lhs.action.time_s < rhs.action.time_s;
    });
    return actions;
}

void apply_single_action(
    const PhaseAction& action,
    const SimulationConfig& simulation_config,
    RuntimeControl* control,
    post2::vehicle::VehicleRuntimeState* runtime)
{
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
        apply_single_action(
            actions[control->next_action_index].action,
            simulation_config,
            control,
            runtime);
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
    const post2::vehicle::VehicleRuntimeState& initial_runtime,
    MissionEventsState* mission_events)
{
    const bool time_terminated = phase.termination.type == "time";
    const double phase_horizon_s = time_terminated ? phase.termination.value : kMaxPhaseTimeS;
    const double phase_end_time_s = phase_start_time_s + phase_horizon_s;
    const SimulationConfig simulation_config =
        make_phase_simulation_config(case_config, phase, phase_end_time_s);
    post2::propagation::VehicleConsumptionPropagator vehicle_propagator(case_config.vehicle);
    auto integrator = post2::integrators::make_integrator(
        phase.integrator, case_config.step_s, phase.tolerances);
    ThrottleModelConfig effective_throttle = phase.throttle_model;
    if (lowercase(phase.steering_model.type) == "upfg") {
        effective_throttle = ThrottleModelConfig{};
        effective_throttle.type = "poly";
        effective_throttle.c0 = 1.0;
        effective_throttle.c1 = 0.0;
        effective_throttle.c2 = 0.0;
        effective_throttle.target_t2w = 1.0;
        effective_throttle.continuity = false;
    }
    const auto throttle_model = make_throttle_model(effective_throttle);
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

    const bool use_adaptive_step_suggestions =
        phase.integrator == "dopri5" || phase.integrator == "dop853";
    const double min_effective_step_s = minimum_effective_step_s(case_config.step_s);
    const int max_step_count =
        max_phase_integration_steps(phase_horizon_s, case_config.step_s);
    int step_count = 0;
    double time_s = phase_start_time_s;
    double suggested_step_s = case_config.step_s;
    while (time_s < phase_end_time_s) {
        ++step_count;
        if (step_count > max_step_count) {
            std::ostringstream msg;
            msg << "integrator exceeded phase step budget in phase " << phase_index
                << " (\"" << phase.name << "\") at t=" << time_s
                << " s, phase_t=" << (time_s - phase_start_time_s) << " s";
            throw std::runtime_error(msg.str());
        }

        const double phase_time_s = time_s - phase_start_time_s;
        const std::size_t action_index_before_step = control.next_action_index;
        apply_due_actions(actions, phase_time_s, simulation_config, &control, &runtime);
        if (control.next_action_index != action_index_before_step) {
            suggested_step_s = case_config.step_s;
        }
        // The integrator events localize crossings inside the next step; this
        // covers phase/action boundaries where the condition is already true.
        if (!time_terminated &&
            trigger_condition_is_satisfied(
                phase.termination,
                runtime,
                case_config.earth_mu_m3s2,
                case_config.earth_radius_m,
                phase_start_time_s)) {
            break;
        }
        const bool powered_upfg =
            !time_terminated && is_powered_upfg_phase(phase, control, actions);
        const double burnout_margin_s = std::max(1.0e-3, min_effective_step_s);
        const double burn_time_remaining_s = powered_upfg
            ? active_propulsive_burn_time_s(case_config, runtime)
            : std::numeric_limits<double>::infinity();
        if (powered_upfg && burn_time_remaining_s <= burnout_margin_s) {
            break;
        }

        const double next_action_absolute_s = phase_start_time_s + next_action_time_s(actions, control);
        const double requested_step_s =
            (std::isfinite(suggested_step_s) && suggested_step_s > 1.0e-12)
                ? suggested_step_s
                : case_config.step_s;
        double step_s = std::min(requested_step_s, phase_end_time_s - time_s);
        if (next_action_absolute_s > time_s && next_action_absolute_s < time_s + step_s) {
            step_s = next_action_absolute_s - time_s;
        }
        if (powered_upfg && burn_time_remaining_s < step_s + burnout_margin_s) {
            step_s = burn_time_remaining_s - burnout_margin_s;
        }
        if (step_s <= 1.0e-12) {
            step_s = std::min(case_config.step_s, phase_end_time_s - time_s);
        }
        if (powered_upfg && step_s > burn_time_remaining_s - burnout_margin_s) {
            break;
        }
        // While clamped, if an engine is still spooling up toward its commanded
        // thrust (the ignition transient), cap the step so the forward-Euler
        // clamp branch resolves the buildup and the "thrust established" crossing
        // finely. Engines with no transient reach commanded thrust immediately
        // (actual == commanded) and keep their normal step, as do clamp holds
        // once thrust is fully established.
        if (control.hold_down_clamp_active &&
            runtime.engine.commanded_thrust_n > 0.0 &&
            runtime.engine.actual_thrust_n < 0.999 * runtime.engine.commanded_thrust_n) {
            constexpr double kClampSpoolSubstepS = 0.05;
            step_s = std::min(step_s, kClampSpoolSubstepS);
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
        bool phase_terminated_by_event = false;
        std::size_t fired_mission_event_index = std::numeric_limits<std::size_t>::max();
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
                    ev.direction = -1;
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

            // Phase termination event (non-time terminations only — time
            // termination is handled by the outer while-loop guard).
            const std::size_t termination_event_index =
                time_terminated ? std::numeric_limits<std::size_t>::max() : events.size();
            if (!time_terminated) {
                events.push_back(make_trigger_event(
                    phase.termination,
                    runtime,
                    case_config.earth_mu_m3s2,
                    case_config.earth_radius_m,
                    phase_start_time_s,
                    /*terminating=*/true,
                    "phase_termination"));
            }

            // Mission events. Re-registered fresh each step (so the
            // mass-trigger snapshots reflect the current attached/active
            // stage set). Only events whose previous-g is non-positive are
            // armed — others are waiting for the next rising edge.
            const std::size_t mission_event_base = events.size();
            std::vector<std::size_t> mission_event_armed_indices;
            if (mission_events) {
                for (std::size_t k = 0; k < case_config.events.size(); ++k) {
                    if (!case_config.events[k].enabled) {
                        continue;
                    }
                    const MissionEventRuntime& rt = mission_events->runtimes[k];
                    if (rt.prev_g > 0.0) {
                        // Already past the rising edge; not armed.
                        continue;
                    }
                    events.push_back(make_trigger_event(
                        case_config.events[k].trigger,
                        runtime,
                        case_config.earth_mu_m3s2,
                        case_config.earth_radius_m,
                        /*time_base_s=*/0.0,
                        /*terminating=*/true,
                        "mission_event_" + std::to_string(k)));
                    mission_event_armed_indices.push_back(k);
                }
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
                std::ostringstream message;
                message << "integrator failed to make progress"
                        << " in phase " << phase_index
                        << " (\"" << phase.name << "\")"
                        << " at t=" << time_s << " s"
                        << ", phase_t=" << phase_time_s << " s"
                        << ", requested_h=" << step_s << " s"
                        << ", used_h=" << step_result.h_used << " s"
                        << ", accepted=" << (step_result.accepted ? "true" : "false");
                if (step_result.event.has_value()) {
                    message << ", event=\"" << step_result.event->name << "\""
                            << ", event_t=" << step_result.event->t_s << " s";
                }
                throw std::runtime_error(message.str());
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
            // Dispatch by event class. tank_empty: clamp that tank to zero.
            // altitude_zero: break the outer loop. phase_termination: break.
            // mission_event_K: applied later, after vehicle_propagator.commit.
            if (step_result.event.has_value()) {
                const std::size_t event_idx = step_result.event->event_index;
                altitude_zero_event_hit = step_result.event->name == "altitude_zero";
                if (event_idx < event_tank_indices.size()) {
                    // tank_empty_K
                    const std::size_t tank_idx = event_tank_indices[event_idx];
                    if (tank_idx < integrated.tank_masses_kg.size()) {
                        integrated.tank_masses_kg[tank_idx] = 0.0;
                    }
                } else if (!time_terminated && event_idx == termination_event_index) {
                    phase_terminated_by_event = true;
                } else if (event_idx >= mission_event_base &&
                           (event_idx - mission_event_base) < mission_event_armed_indices.size()) {
                    fired_mission_event_index =
                        mission_event_armed_indices[event_idx - mission_event_base];
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
        const std::size_t action_index_before_commit_actions = control.next_action_index;
        apply_due_actions(actions, next_time_s - phase_start_time_s, simulation_config, &control, &runtime);
        if (control.next_action_index != action_index_before_commit_actions) {
            suggested_step_s = case_config.step_s;
        }

        // Mission events: apply the fired event's actions atomically, then
        // re-evaluate prev_g for every enabled event so the next step arms
        // correctly. Evaluating against runtime *after* actions means a
        // trigger that the action invalidates (e.g. detached stage drops
        // total mass below threshold) does not immediately refire.
        if (fired_mission_event_index != std::numeric_limits<std::size_t>::max() &&
            mission_events &&
            fired_mission_event_index < case_config.events.size()) {
            const EventConfig& fired_event = case_config.events[fired_mission_event_index];
            for (const auto& action : fired_event.actions) {
                apply_single_action(action, simulation_config, &control, &runtime);
            }
            mission_events->runtimes[fired_mission_event_index].fired_count++;
        }
        if (mission_events) {
            const post2::integrators::ExtendedState eval_state{
                runtime.vehicle.motion,
                post2::vehicle::read_tank_masses_flat(runtime),
            };
            for (std::size_t k = 0; k < case_config.events.size(); ++k) {
                if (!case_config.events[k].enabled) {
                    continue;
                }
                post2::integrators::EventFunction probe = make_trigger_event(
                    case_config.events[k].trigger,
                    runtime,
                    case_config.earth_mu_m3s2,
                    case_config.earth_radius_m,
                    /*time_base_s=*/0.0,
                    /*terminating=*/false,
                    "mission_event_probe");
                double g = probe.g(next_time_s, eval_state);
                if (k == fired_mission_event_index && g == 0.0) {
                    g = 1.0e-12;
                }
                mission_events->runtimes[k].prev_g = g;
            }
        }
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
        if (altitude_zero_event_hit || phase_terminated_by_event) {
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

        // Mission events state lives across all phases. prev_g is initialised
        // to a small negative value so a trigger already satisfied at t=0
        // does not spuriously refire on its first evaluation.
        MissionEventsState mission_events;
        mission_events.runtimes.assign(config.events.size(), MissionEventRuntime{});

        double phase_start_time_s = 0.0;
        for (std::size_t phase_index = 0; phase_index < config.phases.size(); ++phase_index) {
            // Mutable copy: phase-boundary continuity may re-anchor the throttle
            // and steering constants before this phase propagates.
            PhaseConfig phase = config.phases[phase_index];
            const double phase_horizon_for_sim_config =
                (phase.termination.type == "time") ? phase.termination.value : kMaxPhaseTimeS;
            const SimulationConfig simulation_config =
                make_phase_simulation_config(
                    config, phase, phase_start_time_s + phase_horizon_for_sim_config);

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

            // At a phase switch, re-anchor any continuity-enabled throttle/
            // steering constants to the previous phase's final state.
            if (!state_log.empty()) {
                apply_phase_start_continuity(
                    &phase,
                    config,
                    initial_runtime,
                    state_log.back().throttle,
                    state_log.back().engine_direction_eci);
            }

            const StateLog phase_log = propagate_phase(
                config, phase, phase_index, phase_start_time_s, initial_runtime, &mission_events);
            merge_phase_log(&state_log, phase_log);
            if (case_vehicle_impacted_earth(state_log, config)) {
                return {false, "vehicle impacted Earth during propagation", state_log};
            }
            // The phase may have terminated by event before exhausting its
            // time horizon; advance the mission clock to wherever the phase
            // actually ended.
            if (!phase_log.empty()) {
                phase_start_time_s = phase_log.back().time_s;
            } else if (phase.termination.type == "time") {
                phase_start_time_s += phase.termination.value;
            }
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

StateLog CoastPropagator::propagate(
    const CaseConfig& case_config,
    const StateLog& source_log,
    double horizon_s,
    int sample_count) const
{
    StateLog out(case_config.earth_radius_m, case_config.vehicle);
    if (source_log.empty() || horizon_s <= 0.0 || sample_count <= 0) {
        return out;
    }

    const LaunchVehicleStateLogEntry& last = source_log.back();
    PhaseConfig phase;
    phase.name = "predicted orbit";
    phase.inherit_initial_state = false;
    phase.optimize_enabled = false;
    phase.hold_down_clamp_initial_active = false;
    phase.integrator = "dop853";
    phase.tolerances = case_config.phases.empty()
        ? post2::integrators::IntegratorTolerances{}
        : case_config.phases.back().tolerances;
    phase.force_models = case_config.phases.empty()
        ? ForceModelSwitches{}
        : case_config.phases.back().force_models;
    phase.force_models.gravity = true;
    phase.force_models.thrust = false;
    phase.force_models.normal_force = false;
    phase.throttle_model = ThrottleModelConfig{};
    phase.throttle_model.c0 = 0.0;
    phase.steering_model = SteeringModelConfig{};
    phase.termination = TriggerCondition{"time", ">=", horizon_s};

    const SimulationConfig simulation_config =
        make_phase_simulation_config(case_config, phase, last.time_s + horizon_s);
    const post2::propagation::ForceModelSet force_models =
        post2::propagation::make_force_model_set(case_config, phase);

    post2::vehicle::VehicleRuntimeState coast_runtime = last.runtime;
    coast_runtime.time_s = last.time_s;
    coast_runtime.vehicle.motion = last.state;
    coast_runtime.engine.enabled = false;
    coast_runtime.engine.firing = false;
    coast_runtime.engine.throttle = 0.0;
    coast_runtime.engine.commanded_thrust_n = 0.0;
    coast_runtime.engine.actual_thrust_n = 0.0;
    coast_runtime.engine.mass_flow_kgps = 0.0;
    for (auto& stage : coast_runtime.stages) {
        stage.engine.firing = false;
        stage.engine.throttle = 0.0;
        stage.engine.commanded_thrust_n = 0.0;
        stage.engine.actual_thrust_n = 0.0;
        stage.engine.mass_flow_kgps = 0.0;
    }

    out.set_phase_metadata(last.phase_index + 1, "predicted orbit");
    {
        const auto env = make_environment_state(
            simulation_config, phase, coast_runtime.time_s, coast_runtime.vehicle.motion);
        append_entry_with_environment(
            &out, coast_runtime, env, simulation_config.earth_rotation_rad_per_s);
    }

    post2::integrators::ExtendedState state{
        coast_runtime.vehicle.motion,
        post2::vehicle::read_tank_masses_flat(coast_runtime),
    };
    post2::integrators::Dop853Integrator integrator(phase.tolerances);

    auto derivative = [&](double t_s, const post2::integrators::ExtendedState& eval_state) {
        post2::vehicle::VehicleRuntimeState eval_runtime = coast_runtime;
        eval_runtime.time_s = t_s;
        eval_runtime.vehicle.motion = eval_state.motion;
        const post2::propagation::EnvironmentState environment =
            make_environment_state(simulation_config, phase, t_s, eval_state.motion);
        const post2::propagation::ForceModelContext force_context{
            &case_config,
            &phase,
            &eval_runtime,
            &environment,
            {0.0, 0.0, 0.0},
        };
        const post2::propagation::ForceModelOutput force_output =
            force_models.evaluate_all(force_context, eval_state);
        post2::integrators::ExtendedDerivative d;
        d.motion_dot = {eval_state.motion.velocity_mps, force_output.acceleration_eci_mps2};
        d.tank_mass_dots_kgps.assign(eval_state.tank_masses_kg.size(), 0.0);
        return d;
    };

    const int samples = std::max(2, sample_count);
    double t_s = last.time_s;
    double suggested_h = horizon_s / static_cast<double>(samples);
    const double end_t_s = last.time_s + horizon_s;
    for (int sample = 1; sample <= samples; ++sample) {
        const double target_t_s = last.time_s + horizon_s * static_cast<double>(sample) /
            static_cast<double>(samples);
        while (t_s < target_t_s - 1.0e-10) {
            const double h = std::min(
                (std::isfinite(suggested_h) && suggested_h > 1.0e-12) ? suggested_h : target_t_s - t_s,
                target_t_s - t_s);
            const post2::integrators::StepResult step =
                integrator.step(state, t_s, h, derivative, {});
            if (!step.accepted || step.h_used <= 1.0e-12) {
                throw std::runtime_error("coast DOP853 integrator failed to make progress");
            }
            state = step.state_end;
            t_s = step.t_end;
            suggested_h =
                (std::isfinite(step.h_next_suggested) && step.h_next_suggested > 1.0e-12)
                    ? step.h_next_suggested
                    : h;
            if (t_s > end_t_s + 1.0e-9) {
                break;
            }
        }

        coast_runtime.time_s = t_s;
        coast_runtime.vehicle.motion = state.motion;
        const auto env = make_environment_state(
            simulation_config, phase, t_s, coast_runtime.vehicle.motion);
        if (env.altitude_m < 0.0) {
            throw std::runtime_error("coast trajectory impacted Earth");
        }
        const auto d = derivative(t_s, state);
        append_entry_with_environment(
            &out,
            coast_runtime,
            env,
            simulation_config.earth_rotation_rad_per_s,
            d.motion_dot.d_velocity_mps2);
    }

    return out;
}

StateLog predict_orbit_path(
    const CaseConfig& case_config,
    const StateLog& source_log,
    int sample_count)
{
    StateLog empty(case_config.earth_radius_m, case_config.vehicle);
    if (source_log.empty()) {
        return empty;
    }
    const LaunchVehicleStateLogEntry& last = source_log.back();
    const Vec3& r = last.state.position_m;
    const Vec3& v = last.state.velocity_mps;
    const double r_norm = post2::vehicle::norm(r);
    const double v_norm = post2::vehicle::norm(v);
    const double mu = case_config.earth_mu_m3s2;
    if (r_norm <= 0.0 || mu <= 0.0) {
        return empty;
    }
    // Semi-major axis from vis-viva; only closed (bound) orbits get a path.
    const double specific_energy = 0.5 * v_norm * v_norm - mu / r_norm;
    if (specific_energy >= 0.0) {
        return empty;
    }
    const double a = -mu / (2.0 * specific_energy);
    if (a <= 0.0) {
        return empty;
    }
    constexpr double kPi = 3.141592653589793238462643383279502884;
    const double period_s = 2.0 * kPi * std::sqrt((a * a * a) / mu);
    const double horizon_s = period_s * 1.02;

    const int samples = std::max(60, sample_count);
    try {
        return CoastPropagator{}.propagate(case_config, source_log, horizon_s, samples);
    } catch (const std::exception&) {
        return empty;
    }
}

} // namespace post2::core
