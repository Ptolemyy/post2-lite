#include "post2/core/simulation_driver.hpp"

#include "post2/aero/aero_model.hpp"
#include "post2/core/control_models.hpp"
#include "post2/core/coordinates.hpp"
#include "post2/core/frames.hpp"
#include "post2/core/gfold_solver.hpp"
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
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace post2::core {

namespace {

constexpr ColorRgb kRootStackPredictionColor{13, 148, 136};
constexpr ColorRgb kControllerPredictionColors[] = {
    {37, 99, 235},
    {219, 39, 119},
    {245, 158, 11},
    {22, 163, 74},
    {124, 58, 237},
    {234, 88, 12},
    {14, 165, 233},
    {190, 24, 93},
};

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

int phase_controller_stage_index(const CaseConfig& case_config, const PhaseConfig& phase)
{
    const std::vector<post2::vehicle::StageConfig> stages =
        post2::vehicle::effective_stage_configs(case_config.vehicle);
    if (stages.empty()) {
        return -1;
    }
    if (phase.controller_stage_index >= 0 &&
        static_cast<std::size_t>(phase.controller_stage_index) < stages.size()) {
        return phase.controller_stage_index;
    }
    if (!phase.controller_stage_name.empty()) {
        for (std::size_t i = 0; i < stages.size(); ++i) {
            if (stages[i].name == phase.controller_stage_name) {
                return static_cast<int>(i);
            }
        }
    }
    return static_cast<int>(stages.size() - 1);
}

int phase_controlled_stage_index(const CaseConfig& case_config, const PhaseConfig& phase)
{
    return phase.controller_detached_stage
        ? phase_controller_stage_index(case_config, phase)
        : -1;
}

std::string prediction_controller_name(
    const CaseConfig& case_config,
    const PhaseConfig& phase,
    int controller_stage_index)
{
    if (!phase.controller_detached_stage) {
        return "root stack";
    }
    const std::vector<post2::vehicle::StageConfig> stages =
        post2::vehicle::effective_stage_configs(case_config.vehicle);
    if (controller_stage_index >= 0 &&
        static_cast<std::size_t>(controller_stage_index) < stages.size()) {
        return stages[static_cast<std::size_t>(controller_stage_index)].name;
    }
    if (!phase.controller_stage_name.empty()) {
        return phase.controller_stage_name;
    }
    return "detached controller";
}

ColorRgb prediction_color_for_controller(
    bool detached_controller,
    int controller_stage_index)
{
    if (!detached_controller) {
        return kRootStackPredictionColor;
    }
    const int index = std::max(0, controller_stage_index);
    return kControllerPredictionColors[
        static_cast<std::size_t>(index) %
        (sizeof(kControllerPredictionColors) / sizeof(kControllerPredictionColors[0]))];
}

void sync_engine_from_mounted_stage(
    post2::vehicle::VehicleRuntimeState* runtime,
    int preferred_stage_index = -1)
{
    if (!runtime || runtime->stages.empty()) {
        return;
    }
    if (preferred_stage_index >= 0 &&
        static_cast<std::size_t>(preferred_stage_index) < runtime->stages.size()) {
        runtime->engine = runtime->stages[static_cast<std::size_t>(preferred_stage_index)].engine;
        return;
    }
    for (const auto& stage : runtime->stages) {
        if (stage.attached && stage.active) {
            runtime->engine = stage.engine;
            return;
        }
    }
    for (const auto& stage : runtime->stages) {
        if (stage.attached) {
            runtime->engine = stage.engine;
            return;
        }
    }
    runtime->engine = runtime->stages.front().engine;
}

void mount_phase_controller_vehicle(
    const PhaseConfig& phase,
    const CaseConfig& case_config,
    post2::vehicle::VehicleRuntimeState* runtime)
{
    if (!runtime || runtime->stages.empty()) {
        return;
    }

    if (!phase.controller_detached_stage) {
        post2::vehicle::refresh_vehicle_masses(runtime);
        sync_engine_from_mounted_stage(runtime);
        return;
    }

    const int controlled_stage_index = phase_controller_stage_index(case_config, phase);
    if (controlled_stage_index < 0 ||
        static_cast<std::size_t>(controlled_stage_index) >= runtime->stages.size()) {
        return;
    }

    const std::size_t controlled = static_cast<std::size_t>(controlled_stage_index);
    for (std::size_t i = 0; i < runtime->stages.size(); ++i) {
        auto& stage = runtime->stages[i];
        const bool mounted = i == controlled;
        stage.attached = mounted;
        if (!mounted) {
            stage.active = false;
            stage.engine.firing = false;
            stage.engine.throttle = 0.0;
            stage.engine.commanded_thrust_n = 0.0;
            stage.engine.actual_thrust_n = 0.0;
            stage.engine.mass_flow_kgps = 0.0;
            stage.engine.spool_throttle = 0.0;
            stage.engine.ignition_time_s = -1.0;
        }
    }
    post2::vehicle::refresh_vehicle_masses(runtime);
    sync_engine_from_mounted_stage(runtime, controlled_stage_index);
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
        runtime.vehicle.rigid_body,
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

Vec3 normalized_or(const Vec3& value, const Vec3& fallback)
{
    const double length = post2::vehicle::norm(value);
    if (length <= 1.0e-12) {
        return fallback;
    }
    return value / length;
}

struct DynamicsPlane {
    Vec3 normal{0.0, 0.0, 1.0};
    Vec3 axis_u{1.0, 0.0, 0.0};
    Vec3 axis_v{0.0, 1.0, 0.0};
    bool enabled = false;
};

bool two_point_five_dof(DynamicsDof dof)
{
    return dof == DynamicsDof::TwoPointFiveDof;
}

Vec3 project_onto_plane(const Vec3& value, const DynamicsPlane& plane)
{
    if (!plane.enabled) {
        return value;
    }
    return value - plane.normal * post2::vehicle::dot(value, plane.normal);
}

void set_two_point_five_dof_plane_basis(
    DynamicsPlane* plane,
    const State& state,
    const Vec3& direction_hint_eci)
{
    if (!plane || !plane->enabled) {
        return;
    }

    Vec3 axis_u = project_onto_plane(state.position_m, *plane);
    if (post2::vehicle::norm(axis_u) <= 1.0e-12) {
        axis_u = project_onto_plane(direction_hint_eci, *plane);
    }
    if (post2::vehicle::norm(axis_u) <= 1.0e-12) {
        const Vec3 reference = std::abs(plane->normal.z) < 0.9
            ? Vec3{0.0, 0.0, 1.0}
            : Vec3{0.0, 1.0, 0.0};
        axis_u = cross_product(reference, plane->normal);
    }
    plane->axis_u = normalized_or(axis_u, {1.0, 0.0, 0.0});
    plane->axis_v = normalized_or(
        cross_product(plane->normal, plane->axis_u),
        {0.0, 1.0, 0.0});

    const Vec3 projected_hint = project_onto_plane(direction_hint_eci, *plane);
    if (post2::vehicle::norm(projected_hint) > 1.0e-12 &&
        post2::vehicle::dot(projected_hint, plane->axis_v) < 0.0) {
        plane->normal = plane->normal * -1.0;
        plane->axis_v = plane->axis_v * -1.0;
    }
}

State apply_two_point_five_dof_constraint(const State& state, const DynamicsPlane& plane)
{
    if (!plane.enabled) {
        return state;
    }
    return {
        project_onto_plane(state.position_m, plane),
        project_onto_plane(state.velocity_mps, plane),
    };
}

// Geometric gravity vector used to orient the 2.5-DOF plane. Uses the phase's
// gravity model where one is defined (point mass / J2), independent of whether
// the gravity *force* is enabled, so the plane always follows the true local
// "down". Falls back to the radial direction (-position) when no usable model
// is configured, which reproduces the orbital plane.
Vec3 plane_gravity_vector(const SimulationConfig& config, const Vec3& position_m)
{
    const std::string& type = config.gravity_model.type;
    if (type == "point_mass" || type == "j2") {
        return post2::propagation::gravity_acceleration_mps2(config, position_m);
    }
    return position_m * -1.0;
}

DynamicsPlane make_two_point_five_dof_plane(
    const State& state,
    const Vec3& gravity_mps2,
    const Vec3& direction_hint_eci)
{
    DynamicsPlane plane;

    auto use_normal_if_valid = [&plane](const Vec3& candidate, double threshold) {
        const double n = post2::vehicle::norm(candidate);
        if (n <= threshold) {
            return false;
        }
        plane.normal = candidate / n;
        plane.enabled = true;
        return true;
    };

    const double r_norm = post2::vehicle::norm(state.position_m);
    const double v_norm = post2::vehicle::norm(state.velocity_mps);
    const double g_norm = post2::vehicle::norm(gravity_mps2);
    const Vec3 hint_unit = normalized_or(direction_hint_eci, {0.0, 0.0, 0.0});

    // Primary: the plane spanned by the velocity and gravity vectors -- the true
    // "vertical plane" of motion, its normal perpendicular to both. For central
    // gravity this coincides with the orbital plane; under J2 it follows the
    // actual local down direction rather than the pure radial.
    if (use_normal_if_valid(
            cross_product(gravity_mps2, state.velocity_mps),
            std::max(1.0, g_norm) * std::max(1.0, v_norm) * 1.0e-9)) {
        set_two_point_five_dof_plane_basis(&plane, state, direction_hint_eci);
        return plane;
    }

    // Velocity (anti)parallel to gravity -- e.g. vertical ascent. Pick the plane
    // azimuth from the steering/heading hint crossed with gravity.
    if (use_normal_if_valid(
            cross_product(gravity_mps2, hint_unit),
            std::max(1.0, g_norm) * 1.0e-9)) {
        set_two_point_five_dof_plane_basis(&plane, state, direction_hint_eci);
        return plane;
    }

    // Gravity degenerate / unavailable: fall back to the orbital-plane geometry.
    if (use_normal_if_valid(cross_product(state.position_m, hint_unit), r_norm * 1.0e-9)) {
        set_two_point_five_dof_plane_basis(&plane, state, direction_hint_eci);
        return plane;
    }
    if (use_normal_if_valid(
            cross_product(state.position_m, state.velocity_mps),
            r_norm * std::max(1.0, v_norm) * 1.0e-12)) {
        set_two_point_five_dof_plane_basis(&plane, state, direction_hint_eci);
        return plane;
    }

    // Last resort: a reference plane that still contains the radial direction.
    const Vec3 anchor = r_norm > 1.0e-12 ? state.position_m : hint_unit;
    const Vec3 anchor_unit = normalized_or(anchor, {1.0, 0.0, 0.0});
    const Vec3 reference = std::abs(anchor_unit.z) < 0.9
        ? Vec3{0.0, 0.0, 1.0}
        : Vec3{0.0, 1.0, 0.0};
    use_normal_if_valid(cross_product(anchor, reference), 1.0e-12);
    plane.enabled = true;
    set_two_point_five_dof_plane_basis(&plane, state, direction_hint_eci);
    return plane;
}

Vec3 two_point_five_dof_body_axis_eci(
    const post2::vehicle::RigidBodyState& rigid_body,
    const DynamicsPlane& plane)
{
    if (!plane.enabled) {
        return {1.0, 0.0, 0.0};
    }
    const double c = std::cos(rigid_body.attitude_rad);
    const double s = std::sin(rigid_body.attitude_rad);
    return normalized_or(plane.axis_u * c + plane.axis_v * s, plane.axis_u);
}

Vec3 two_point_five_dof_engine_direction_eci(
    const post2::vehicle::RigidBodyState& rigid_body,
    const DynamicsPlane& plane,
    const post2::vehicle::EngineConfig& engine)
{
    const Vec3 body_x = two_point_five_dof_body_axis_eci(rigid_body, plane);
    const Vec3 body_y = normalized_or(cross_product(plane.normal, body_x), plane.axis_v);
    const Vec3 direction_body = normalized_or(engine.direction_body, {1.0, 0.0, 0.0});
    return normalized_or(
        body_x * direction_body.x + body_y * direction_body.y,
        body_x);
}

post2::vehicle::EngineConfig selected_engine_config_for_command(
    const CaseConfig& case_config,
    const post2::vehicle::VehicleRuntimeState& runtime,
    int controlled_stage_index)
{
    const std::vector<post2::vehicle::StageConfig> stage_configs =
        post2::vehicle::effective_stage_configs(case_config.vehicle);
    auto stage_config_at = [&](std::size_t index) -> const post2::vehicle::StageConfig* {
        return index < stage_configs.size() ? &stage_configs[index] : nullptr;
    };
    if (controlled_stage_index >= 0 &&
        static_cast<std::size_t>(controlled_stage_index) < runtime.stages.size()) {
        if (const auto* stage_config = stage_config_at(static_cast<std::size_t>(controlled_stage_index))) {
            return stage_config->engine;
        }
    }
    for (std::size_t i = 0; i < runtime.stages.size(); ++i) {
        if (runtime.stages[i].attached && runtime.stages[i].active) {
            if (const auto* stage_config = stage_config_at(i)) {
                return stage_config->engine;
            }
        }
    }
    for (std::size_t i = 0; i < runtime.stages.size(); ++i) {
        if (runtime.stages[i].attached) {
            if (const auto* stage_config = stage_config_at(i)) {
                return stage_config->engine;
            }
        }
    }
    return case_config.vehicle.engine;
}

double two_point_five_dof_angular_acceleration_radps2(
    const CaseConfig& case_config,
    const post2::integrators::ExtendedState& state,
    const post2::propagation::DerivativeResult& eval)
{
    const double inertia = state.rigid_body.moment_of_inertia_kgm2;
    const double moment_arm_m = case_config.vehicle.rigid_body.engine_moment_arm_m;
    if (inertia <= 0.0 || moment_arm_m <= 0.0) {
        return 0.0;
    }

    const std::vector<post2::vehicle::StageConfig> stage_configs =
        post2::vehicle::effective_stage_configs(case_config.vehicle);
    double torque_nm = 0.0;
    for (std::size_t i = 0; i < eval.per_stage_engine.size() && i < stage_configs.size(); ++i) {
        const double thrust_n = eval.per_stage_engine[i].actual_thrust_n;
        if (thrust_n <= 0.0) {
            continue;
        }
        const Vec3 direction_body = normalized_or(
            stage_configs[i].engine.direction_body,
            {1.0, 0.0, 0.0});
        torque_nm += -moment_arm_m * thrust_n * direction_body.y;
    }

    if (eval.per_stage_engine.empty()) {
        const Vec3 direction_body = normalized_or(
            case_config.vehicle.engine.direction_body,
            {1.0, 0.0, 0.0});
        torque_nm += -moment_arm_m * eval.total_actual_thrust_n * direction_body.y;
    }
    return torque_nm / inertia;
}

post2::propagation::EngineCommand constrain_engine_command_to_plane(
    post2::propagation::EngineCommand command,
    const DynamicsPlane& plane,
    const State& state)
{
    if (!plane.enabled) {
        return command;
    }
    const Vec3 radial_fallback = normalized_or(project_onto_plane(state.position_m, plane), {1.0, 0.0, 0.0});
    command.direction_eci =
        normalized_or(project_onto_plane(command.direction_eci, plane), radial_fallback);
    return command;
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
    // Heat-flux nose radius follows the currently-attached vehicle configuration
    // (same selection the force model uses for the CD/CL table), so a separated
    // booster flying alone heats per its own geometry rather than the full stack.
    const post2::vehicle::AeroConfig& aero = state_log->vehicle_config().aero;
    double nose_radius_input = aero.nose_radius_m;
    double ref_diameter_m = aero.ref_diameter_m;
    if (const post2::vehicle::AeroStageTable* active =
            post2::vehicle::select_active_aero_stage_table(aero, runtime)) {
        nose_radius_input = active->nose_radius_m;
        ref_diameter_m = active->ref_diameter_m;
    }
    entry.heat_flux_wpm2 = post2::aero::stagnation_heat_flux_wpm2(
        env.density_kgpm3, v_rel_mag,
        post2::aero::effective_nose_radius_m(nose_radius_input, ref_diameter_m));
    state_log->append(entry);
}

// 3-DOF translational derivative: d(position) = velocity, d(velocity) = sum of
// all force-model accelerations.
post2::integrators::ExtendedDerivative three_dof_extended_dynamics(
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

// 2.5-DOF derivative: keep the existing 6D state representation but constrain
// translational motion and acceleration to the phase plane. The commanded pitch
// attitude enters through the in-plane steering/thrust direction.
post2::integrators::ExtendedDerivative two_point_five_dof_extended_dynamics(
    const post2::propagation::ForceModelSet& force_models,
    const post2::propagation::ForceModelContext& force_context,
    const post2::integrators::ExtendedState& state,
    const DynamicsPlane& plane,
    double angular_acceleration_radps2,
    std::vector<double> tank_mass_dots_kgps)
{
    const post2::propagation::ForceModelOutput force_output =
        force_models.evaluate_all(force_context, state);

    post2::integrators::ExtendedDerivative derivative{
        {
            project_onto_plane(state.motion.velocity_mps, plane),
            project_onto_plane(force_output.acceleration_eci_mps2, plane),
        },
        std::move(tank_mass_dots_kgps),
    };
    derivative.rigid_body_dot.attitude_radps = state.rigid_body.angular_velocity_radps;
    derivative.rigid_body_dot.angular_acceleration_radps2 = angular_acceleration_radps2;
    return derivative;
}

// Dispatch seam for per-phase dynamics models.
post2::integrators::ExtendedDerivative phase_extended_dynamics(
    DynamicsDof dof,
    const DynamicsPlane& dynamics_plane,
    double angular_acceleration_radps2,
    const post2::propagation::ForceModelSet& force_models,
    const post2::propagation::ForceModelContext& force_context,
    const post2::integrators::ExtendedState& state,
    std::vector<double> tank_mass_dots_kgps)
{
    switch (dof) {
        case DynamicsDof::TwoPointFiveDof:
            return two_point_five_dof_extended_dynamics(
                force_models,
                force_context,
                state,
                dynamics_plane,
                angular_acceleration_radps2,
                std::move(tank_mass_dots_kgps));
        case DynamicsDof::ThreeDof:
        default:
            return three_dof_extended_dynamics(
                force_models, force_context, state, std::move(tank_mass_dots_kgps));
    }
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

bool poly_is_default_zero(const Poly2Config& poly)
{
    return poly.c0 == 0.0 &&
        poly.c1 == 0.0 &&
        poly.c2 == 0.0 &&
        !poly.continuity;
}

bool linear_tangent_is_default(const LinearTangentConfig& tangent)
{
    return tangent.a == 0.0 &&
        tangent.a_dot == 0.0 &&
        tangent.b == 0.0 &&
        tangent.b_dot == 0.0 &&
        tangent.t_offset_s == 0.0 &&
        !tangent.continuity;
}

bool upfg_is_default(const UpfgConfig& upfg)
{
    return upfg.periapsis_km == 200.0 &&
        upfg.apoapsis_km == 200.0 &&
        upfg.inclination_deg == kDefaultInclinationDeg;
}

bool vec3_equal(const Vec3& lhs, const Vec3& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool quaternion_points_empty(const std::vector<QuaternionPoint>& points)
{
    return points.empty();
}

bool selectable_segments_empty(const std::vector<SelectableSteeringSegment>& segments)
{
    return segments.empty();
}

bool segmented_steering_poly_is_default(const SegmentedSteeringPolyConfig& poly)
{
    return poly.order == 1 && poly.continuity && poly.segments.empty();
}

bool steering_model_is_default_for_two_point_five_dof(const SteeringModelConfig& steering)
{
    return lowercase(steering.type) == "generic_poly" &&
        poly_is_default_zero(steering.roll_deg) &&
        poly_is_default_zero(steering.pitch_deg) &&
        poly_is_default_zero(steering.yaw_deg) &&
        poly_is_default_zero(steering.azimuth_deg) &&
        poly_is_default_zero(steering.elevation_deg) &&
        linear_tangent_is_default(steering.tangent) &&
        upfg_is_default(steering.upfg) &&
        vec3_equal(steering.fixed_direction_eci, {1.0, 0.0, 0.0}) &&
        quaternion_points_empty(steering.points) &&
        selectable_segments_empty(steering.segments) &&
        segmented_steering_poly_is_default(steering.segmented_poly);
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
    const std::vector<post2::vehicle::StageConfig> stage_configs =
        post2::vehicle::effective_stage_configs(config.vehicle);

    for (std::size_t i = 0; i < config.phases.size(); ++i) {
        const PhaseConfig& phase = config.phases[i];
        if (phase.controller_stage_index < -1) {
            *error = "phase " + std::to_string(i) + " controller_stage_index must be -1 or a stage index";
            return false;
        }
        if (phase.controller_detached_stage &&
            phase.controller_stage_index < 0 &&
            phase.controller_stage_name.empty()) {
            *error = "phase " + std::to_string(i) +
                " detached controller needs controller_stage_index or controller_stage_name";
            return false;
        }
        if (phase.controller_stage_index >= 0 &&
            static_cast<std::size_t>(phase.controller_stage_index) >= stage_configs.size()) {
            *error = "phase " + std::to_string(i) + " controller_stage_index is out of range";
            return false;
        }
        if (!phase.controller_stage_name.empty()) {
            bool found_controller_stage = false;
            for (const auto& stage : stage_configs) {
                if (stage.name == phase.controller_stage_name) {
                    found_controller_stage = true;
                    break;
                }
            }
            if (!found_controller_stage) {
                *error = "phase " + std::to_string(i) +
                    " controller_stage_name does not match any vehicle stage";
                return false;
            }
        }
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
        DynamicsDof phase_dof = DynamicsDof::ThreeDof;
        if (!dynamics_dof_from_string(phase.dynamics_dof, &phase_dof)) {
            *error = "phase " + std::to_string(i) +
                " dynamics_dof must be \"3dof\" or \"2.5dof\"";
            return false;
        }
        const bool is_gfold_steering = lowercase(phase.steering_model.type) == "gfold";
        if (is_gfold_steering && phase_dof != DynamicsDof::TwoPointFiveDof) {
            *error = "phase " + std::to_string(i) +
                " steering_model \"gfold\" requires dynamics_dof \"2.5dof\"";
            return false;
        }
        if (phase_dof == DynamicsDof::TwoPointFiveDof) {
            // The G-FOLD landing owns its whole phase (it solves and plays back a
            // discrete trajectory rather than integrating attitude dynamics), so
            // it is exempt from the rigid-body / default-steering requirements.
            if (!is_gfold_steering &&
                config.vehicle.rigid_body.moment_of_inertia_kgm2 <= 0.0) {
                *error = "phase " + std::to_string(i) +
                    " dynamics_dof \"2.5dof\" requires vehicle.rigid_body.moment_of_inertia_kgm2 > 0";
                return false;
            }
            if (!is_gfold_steering &&
                !steering_model_is_default_for_two_point_five_dof(phase.steering_model)) {
                *error = "phase " + std::to_string(i) +
                    " dynamics_dof \"2.5dof\" does not support steering_model; use vehicle.rigid_body initial attitude/omega and engine.direction_body";
                return false;
            }
        }
        if (phase.tolerances.rtol <= 0.0 ||
            phase.tolerances.atol_position_m <= 0.0 ||
            phase.tolerances.atol_velocity_mps <= 0.0 ||
            phase.tolerances.atol_tank_mass_kg <= 0.0 ||
            phase.tolerances.atol_attitude_rad <= 0.0 ||
            phase.tolerances.atol_angular_velocity_radps <= 0.0) {
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
    int controlled_stage_index = -1;
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
    const post2::vehicle::VehicleRuntimeState& runtime,
    int controlled_stage_index)
{
    const std::vector<post2::vehicle::StageConfig> stage_configs =
        post2::vehicle::effective_stage_configs(case_config.vehicle);
    double propellant_kg = 0.0;
    double mdot_kgps = 0.0;
    for (std::size_t i = 0; i < stage_configs.size() && i < runtime.stages.size(); ++i) {
        if (controlled_stage_index >= 0 && i != static_cast<std::size_t>(controlled_stage_index)) {
            continue;
        }
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
        if (found_stage && control) {
            if (control->controlled_stage_index >= 0) {
                if (stage_index != static_cast<std::size_t>(control->controlled_stage_index)) {
                    found_stage = false;
                }
            } else if (stage_index >= runtime->stages.size() ||
                       !runtime->stages[stage_index].attached) {
                found_stage = false;
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
    post2::propagation::EngineCommand command;
    command.enabled = engine_enabled;
    command.throttle = throttle_model.throttle(phase_time_s, runtime_for_model, context);
    command.ignited_engine_count =
        throttle_model.ignited_engine_count(phase_time_s, runtime_for_model, context);
    command.direction_eci = steering_model.thrust_direction_eci(
        phase_time_s,
        state,
        runtime_for_model,
        context);
    if (context.case_config) {
        command.controlled_stage_index = phase_controlled_stage_index(*context.case_config, phase);
    }
    return command;
}

post2::propagation::EngineCommand make_two_point_five_dof_engine_command(
    const PhaseConfig& phase,
    const PhaseContext& context,
    const IThrottleModel& throttle_model,
    const post2::vehicle::VehicleRuntimeState& runtime,
    const State&,
    const DynamicsPlane& plane,
    double absolute_time_s,
    bool engine_enabled)
{
    if (!phase.force_models.thrust) {
        return {};
    }

    post2::vehicle::VehicleRuntimeState runtime_for_model = runtime;
    runtime_for_model.engine.enabled = engine_enabled;
    const double phase_time_s = absolute_time_s - context.phase_start_time_s;
    post2::propagation::EngineCommand command;
    command.enabled = engine_enabled;
    command.throttle = throttle_model.throttle(phase_time_s, runtime_for_model, context);
    command.ignited_engine_count =
        throttle_model.ignited_engine_count(phase_time_s, runtime_for_model, context);
    if (context.case_config) {
        command.controlled_stage_index =
            phase_controlled_stage_index(*context.case_config, phase);
        const post2::vehicle::EngineConfig engine =
            selected_engine_config_for_command(
                *context.case_config,
                runtime_for_model,
                command.controlled_stage_index);
        command.direction_eci =
            two_point_five_dof_engine_direction_eci(
                runtime_for_model.vehicle.rigid_body,
                plane,
                engine);
    } else {
        command.direction_eci = two_point_five_dof_body_axis_eci(
            runtime_for_model.vehicle.rigid_body,
            plane);
    }
    return command;
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

// A G-FOLD landing phase does NOT integrate. It solves the 2D fuel-optimal
// powered-descent SOCP (golden-section over time-of-flight) in the vehicle's
// 2.5-DOF vertical plane, lifts each discrete node back to 3D ECI, and writes
// the trajectory straight into the state log. Mass/thrust come from the runtime
// landing stack; the convex-problem knobs come from steering_model.gfold. On
// solver failure it records only the inherited handoff state (no landing).
StateLog propagate_gfold_landing_phase(
    const CaseConfig& case_config,
    const PhaseConfig& phase,
    std::size_t phase_index,
    double phase_start_time_s,
    const post2::vehicle::VehicleRuntimeState& initial_runtime)
{
    const SimulationConfig simulation_config = make_phase_simulation_config(
        case_config, phase, phase_start_time_s + kMaxPhaseTimeS);

    StateLog state_log(case_config.earth_radius_m, case_config.vehicle);
    state_log.set_phase_metadata(static_cast<int>(phase_index), phase.name);

    auto runtime = initial_runtime;
    runtime.time_s = phase_start_time_s;
    mount_phase_controller_vehicle(phase, case_config, &runtime);

    const State motion0 = runtime.vehicle.motion;
    const Vec3 gravity0 = plane_gravity_vector(simulation_config, motion0.position_m);
    const DynamicsPlane plane =
        make_two_point_five_dof_plane(motion0, gravity0, motion0.velocity_mps);
    const Vec3 up = plane.axis_u;        // local vertical (radial) in ECI
    const Vec3 down = plane.axis_v;      // downrange / horizontal in ECI

    // Records one state-log entry from explicit 3D values (no integrator).
    auto write_entry = [&](double t_s, const Vec3& pos, const Vec3& vel,
                           const Vec3& thrust_dir_eci, double throttle, double thrust_n,
                           double mass_kg, double m_dry_kg, const Vec3& accel_eci) {
        runtime.time_s = t_s;
        runtime.vehicle.motion = {pos, vel};
        runtime.vehicle.total_mass_kg = mass_kg;
        runtime.vehicle.propellant_mass_kg = std::max(0.0, mass_kg - m_dry_kg);
        runtime.vehicle.rigid_body.attitude_rad = std::atan2(
            post2::vehicle::dot(thrust_dir_eci, down),
            post2::vehicle::dot(thrust_dir_eci, up));
        runtime.engine.enabled = thrust_n > 0.0;
        runtime.engine.firing = thrust_n > 0.0;
        runtime.engine.throttle = throttle;
        runtime.engine.actual_thrust_n = thrust_n;
        // build_entry copies engine.direction_body into engine_direction_eci.
        runtime.engine.direction_body = thrust_dir_eci;
        const auto env = make_environment_state(
            simulation_config, phase, t_s, runtime.vehicle.motion);
        append_entry_with_environment(
            &state_log, runtime, env, simulation_config.earth_rotation_rad_per_s, accel_eci);
    };

    const double m_wet = runtime.vehicle.total_mass_kg;
    const double m_dry = std::max(1.0, m_wet - runtime.vehicle.propellant_mass_kg);

    const int controlled = phase_controlled_stage_index(case_config, phase);
    const post2::vehicle::EngineConfig engine =
        selected_engine_config_for_command(case_config, runtime, controlled);
    const auto env0 = make_environment_state(
        simulation_config, phase, phase_start_time_s, motion0);
    double per_engine_n = engine.thrust_vac_n;
    if (engine.nozzle_exit_area_m2 > 0.0) {
        per_engine_n = engine.thrust_vac_n - env0.pressure_pa * engine.nozzle_exit_area_m2;
    }
    if (per_engine_n <= 0.0) {
        per_engine_n = engine.thrust_vac_n;
    }

    const GfoldConfig& g = phase.steering_model.gfold;
    GfoldProblem problem;
    problem.g0 = post2::vehicle::norm(gravity0);
    problem.m_wet = m_wet;
    problem.m_dry = m_dry;
    problem.exhaust_velocity_mps = engine.isp_vac_s * 9.80665;
    problem.t_max_n = std::max(1, g.engine_count) * per_engine_n;
    problem.min_throttle = g.min_throttle;
    problem.max_throttle = g.max_throttle;
    problem.max_tilt_deg = g.max_tilt_deg;
    problem.glide_slope_deg = g.glide_slope_deg;
    problem.r0x = 0.0;
    problem.r0y = post2::core::frames::ecef_to_geodetic(motion0.position_m).altitude_m;
    problem.v0x = post2::vehicle::dot(motion0.velocity_mps, down);
    problem.v0y = post2::vehicle::dot(motion0.velocity_mps, up);
    problem.rfx = 0.0;
    problem.rfy = 0.0;
    problem.free_landing = g.free_landing;

    const GfoldSolution solution = gfold_solve_optimal(
        problem, std::max(4, g.num_nodes), g.tf_min_s, g.tf_max_s);

    if (!solution.feasible || solution.nodes.empty()) {
        // No feasible landing: discard the phase entirely (return no entries).
        // The driver then continues from the previous phase's final state -- a
        // following phase inherits it, or, if this was the last phase, the
        // trajectory-prediction pass extrapolates the ballistic descent.
        return StateLog(case_config.earth_radius_m, case_config.vehicle);
    }

    const double alt0 = problem.r0y;
    for (const auto& nd : solution.nodes) {
        // Lift the planar node into 3D ECI on the tangent plane through the
        // handoff point (flat-ground / constant-gravity GFOLD approximation):
        // downrange along axis_v, altitude change along axis_u.
        const Vec3 pos = motion0.position_m + down * nd.rx + up * (nd.ry - alt0);
        const Vec3 vel = down * nd.vx + up * nd.vy;
        const Vec3 thrust_accel = down * nd.ux + up * nd.uy;
        const Vec3 thrust_dir = normalized_or(thrust_accel, up);
        const Vec3 accel = thrust_accel + gravity0;
        write_entry(phase_start_time_s + nd.t_s, pos, vel, thrust_dir,
                    nd.throttle, nd.thrust_n, nd.mass_kg, m_dry, accel);
    }
    return state_log;
}

StateLog propagate_phase(
    const CaseConfig& case_config,
    const PhaseConfig& phase,
    std::size_t phase_index,
    double phase_start_time_s,
    const post2::vehicle::VehicleRuntimeState& initial_runtime,
    MissionEventsState* mission_events,
    const std::optional<Vec3>& inherited_orientation_eci = std::nullopt)
{
    // A G-FOLD landing phase is solved and played back, not integrated.
    if (lowercase(phase.steering_model.type) == "gfold") {
        return propagate_gfold_landing_phase(
            case_config, phase, phase_index, phase_start_time_s, initial_runtime);
    }
    const bool time_terminated = phase.termination.type == "time";
    const double phase_horizon_s = time_terminated ? phase.termination.value : kMaxPhaseTimeS;
    const double phase_end_time_s = phase_start_time_s + phase_horizon_s;
    // Resolve the dynamics model once; validate_case_config has already
    // guaranteed the token is recognised.
    DynamicsDof phase_dof = DynamicsDof::ThreeDof;
    dynamics_dof_from_string(phase.dynamics_dof, &phase_dof);
    const SimulationConfig simulation_config =
        make_phase_simulation_config(case_config, phase, phase_end_time_s);
    post2::propagation::VehicleConsumptionPropagator vehicle_propagator(case_config.vehicle);
    auto integrator = post2::integrators::make_integrator(
        phase.integrator, case_config.step_s, phase.tolerances);
    const bool is_two_point_five_dof_phase = two_point_five_dof(phase_dof);
    ThrottleModelConfig effective_throttle = phase.throttle_model;
    if (!is_two_point_five_dof_phase && lowercase(phase.steering_model.type) == "upfg") {
        effective_throttle = ThrottleModelConfig{};
        effective_throttle.type = "poly";
        effective_throttle.c0 = 1.0;
        effective_throttle.c1 = 0.0;
        effective_throttle.c2 = 0.0;
        effective_throttle.target_t2w = 1.0;
        effective_throttle.continuity = false;
    }
    const auto throttle_model = make_throttle_model(effective_throttle);
    std::unique_ptr<ISteeringModel> steering_model;
    if (!is_two_point_five_dof_phase) {
        steering_model = make_steering_model(phase.steering_model);
    }
    const PhaseContext context{&case_config, &phase, phase_index, phase_start_time_s};
    const post2::propagation::ForceModelSet force_models =
        post2::propagation::make_force_model_set(case_config, phase);
    const std::vector<TimedAction> actions = sorted_actions(phase);

    auto runtime = initial_runtime;
    runtime.time_s = phase_start_time_s;
    mount_phase_controller_vehicle(phase, case_config, &runtime);

    RuntimeControl control;
    control.engine_enabled = runtime.engine.enabled;
    control.hold_down_clamp_active = phase.hold_down_clamp_initial_active;
    control.controlled_stage_index = phase_controlled_stage_index(case_config, phase);

    set_hold_down_clamp_state(&runtime, simulation_config, control.hold_down_clamp_active);
    apply_due_actions(actions, 0.0, simulation_config, &control, &runtime);

    double time_s = phase_start_time_s;
    DynamicsPlane dynamics_plane;
    auto ensure_two_point_five_dof_plane = [&]() {
        if (!is_two_point_five_dof_phase ||
            control.hold_down_clamp_active ||
            dynamics_plane.enabled) {
            return;
        }
        // The 2.5-DOF vertical plane is spanned by the local radial direction and
        // the in-plane heading; building it from velocity keeps the orbital plane
        // (so projecting position/velocity onto it is loss-free and the trajectory
        // stays continuous). On a 3-DOF -> 2.5-DOF switch the inherited orientation
        // (the previous phase's 3-D thrust direction) only seeds the plane azimuth
        // when the vehicle is essentially at rest and velocity carries no heading.
        Vec3 plane_hint = runtime.vehicle.motion.velocity_mps;
        if (inherited_orientation_eci &&
            post2::vehicle::norm(*inherited_orientation_eci) > 1.0e-12 &&
            post2::vehicle::norm(runtime.vehicle.motion.velocity_mps) <= 1.0e-3) {
            plane_hint = *inherited_orientation_eci;
        }
        const Vec3 gravity_mps2 =
            plane_gravity_vector(simulation_config, runtime.vehicle.motion.position_m);
        dynamics_plane = make_two_point_five_dof_plane(
            runtime.vehicle.motion,
            gravity_mps2,
            plane_hint);
        runtime.vehicle.motion =
            apply_two_point_five_dof_constraint(runtime.vehicle.motion, dynamics_plane);
        // Project the inherited 3-D orientation into the plane and seed the
        // rigid-body pitch so the body axis matches the pre-switch heading,
        // avoiding an attitude discontinuity at the 3-DOF -> 2.5-DOF boundary.
        if (inherited_orientation_eci) {
            const Vec3 projected =
                project_onto_plane(*inherited_orientation_eci, dynamics_plane);
            if (post2::vehicle::norm(projected) > 1.0e-12) {
                runtime.vehicle.rigid_body.attitude_rad = std::atan2(
                    post2::vehicle::dot(projected, dynamics_plane.axis_v),
                    post2::vehicle::dot(projected, dynamics_plane.axis_u));
            }
        }
    };
    ensure_two_point_five_dof_plane();

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
        ensure_two_point_five_dof_plane();
        // A powered UPFG phase owns its own cutoff (tgo->0, below), so the manual
        // terminal condition is ignored for it -- the algorithm terminates the
        // phase, not the user-set trigger. For every other phase the integrator
        // events localize crossings inside the next step; this discrete check
        // covers phase/action boundaries where the condition is already true.
        const bool powered_upfg =
            !time_terminated && is_powered_upfg_phase(phase, control, actions);
        if (!time_terminated && !powered_upfg &&
            trigger_condition_is_satisfied(
                phase.termination,
                runtime,
                case_config.earth_mu_m3s2,
                case_config.earth_radius_m,
                phase_start_time_s)) {
            break;
        }
        const double burnout_margin_s = std::max(1.0e-3, min_effective_step_s);
        // UPFG steers the orbit to the target exactly at tgo=0, so the algorithm's
        // own cutoff is tgo<=0 (insertion / MECO) -- the same scheme post2_player
        // flies (player main.cpp `cmd.tgo_s <= kBurnoutMarginS`). Recomputed at the
        // committed state each step, bounded by propellant burnout: the phase ends
        // at whichever comes first, the target being reached or the tank running
        // dry. The offline integrator localizes the crossing finely, so a small
        // (one-step) margin suffices here rather than the player's coarse 0.1 s.
        const double upfg_tgo_s = powered_upfg
            ? post2::core::upfg_time_to_go_s(
                  case_config, phase.steering_model, runtime,
                  runtime.vehicle.motion.position_m, runtime.vehicle.motion.velocity_mps)
            : std::numeric_limits<double>::infinity();
        const double burn_time_remaining_s = powered_upfg
            ? active_propulsive_burn_time_s(case_config, runtime, control.controlled_stage_index)
            : std::numeric_limits<double>::infinity();
        const double cutoff_remaining_s = std::min(burn_time_remaining_s, upfg_tgo_s);
        if (powered_upfg && cutoff_remaining_s <= burnout_margin_s) {
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
        if (powered_upfg && cutoff_remaining_s < step_s + burnout_margin_s) {
            step_s = cutoff_remaining_s - burnout_margin_s;
        }
        if (step_s <= 1.0e-12) {
            step_s = std::min(case_config.step_s, phase_end_time_s - time_s);
        }
        if (powered_upfg && step_s > cutoff_remaining_s - burnout_margin_s) {
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
        auto command = is_two_point_five_dof_phase
            ? make_two_point_five_dof_engine_command(
                phase,
                context,
                *throttle_model,
                runtime,
                current_state,
                dynamics_plane,
                time_s,
                control.engine_enabled)
            : make_engine_command(
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
            runtime.vehicle.rigid_body,
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
            integrated.rigid_body = current_extended.rigid_body;
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
            // termination is handled by the outer while-loop guard). A powered
            // UPFG phase is excluded: its manual terminal condition is ignored in
            // favour of the algorithm's own tgo->0 cutoff (handled above), so the
            // trigger is registered as neither a discrete check nor an event.
            const bool register_termination_event = !time_terminated && !powered_upfg;
            const std::size_t termination_event_index =
                register_termination_event ? events.size()
                                           : std::numeric_limits<std::size_t>::max();
            if (register_termination_event) {
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
                    post2::vehicle::VehicleRuntimeState dynamics_runtime = runtime;
                    dynamics_runtime.time_s = dynamics_time_s;
                    dynamics_runtime.vehicle.motion = dynamics_state.motion;
                    dynamics_runtime.vehicle.rigid_body = dynamics_state.rigid_body;
                    auto dynamics_command = is_two_point_five_dof_phase
                        ? make_two_point_five_dof_engine_command(
                            phase,
                            context,
                            *throttle_model,
                            dynamics_runtime,
                            dynamics_state.motion,
                            dynamics_plane,
                            dynamics_time_s,
                            control.engine_enabled)
                        : make_engine_command(
                            phase,
                            context,
                            *throttle_model,
                            *steering_model,
                            dynamics_runtime,
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
                    const double angular_acceleration_radps2 = two_point_five_dof(phase_dof)
                        ? two_point_five_dof_angular_acceleration_radps2(
                            case_config,
                            dynamics_state,
                            eval)
                        : 0.0;
                    auto deriv = phase_extended_dynamics(
                        phase_dof,
                        dynamics_plane,
                        angular_acceleration_radps2,
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
            if (two_point_five_dof(phase_dof) && !control.hold_down_clamp_active) {
                integrated.motion = apply_two_point_five_dof_constraint(
                    integrated.motion,
                    dynamics_plane);
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
                runtime.vehicle.rigid_body,
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
            runtime.vehicle.rigid_body,
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
    DynamicsDof phase_dof = DynamicsDof::ThreeDof;
    dynamics_dof_from_string(phase.dynamics_dof, &phase_dof);
    DynamicsPlane dynamics_plane;
    if (two_point_five_dof(phase_dof)) {
        dynamics_plane = make_two_point_five_dof_plane(
            current_runtime.vehicle.motion,
            plane_gravity_vector(config, current_runtime.vehicle.motion.position_m),
            current_runtime.vehicle.motion.velocity_mps);
        current_runtime.vehicle.motion =
            apply_two_point_five_dof_constraint(current_runtime.vehicle.motion, dynamics_plane);
    }
    const post2::propagation::ForceModelSet force_models =
        post2::propagation::make_force_model_set(case_config, phase);

    return integrator.integrate(
        state_log,
        termination,
        [&config, &case_config, &phase, phase_dof, dynamics_plane, &force_models, &vehicle_propagator, &current_runtime, last_eval, last_cmd](
            double time_s, const post2::integrators::ExtendedState& state) {
            const post2::propagation::EnvironmentState environment =
                make_environment_state(config, phase, time_s, state.motion);
            post2::propagation::EngineCommand cmd{
                current_runtime.engine.enabled,
                vehicle_propagator.throttle_at(time_s),
                current_runtime.engine.direction_body,
            };
            if (two_point_five_dof(phase_dof)) {
                const post2::vehicle::EngineConfig engine =
                    selected_engine_config_for_command(case_config, current_runtime, -1);
                cmd.direction_eci = two_point_five_dof_engine_direction_eci(
                    state.rigid_body,
                    dynamics_plane,
                    engine);
            }
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
            const double angular_acceleration_radps2 = two_point_five_dof(phase_dof)
                ? two_point_five_dof_angular_acceleration_radps2(case_config, state, eval)
                : 0.0;
            auto deriv = phase_extended_dynamics(
                phase_dof,
                dynamics_plane,
                angular_acceleration_radps2,
                force_models,
                force_context,
                state,
                eval.tank_mass_dots_kgps);
            *last_eval = std::move(eval);
            return deriv;
        },
        [&config, phase_dof, dynamics_plane, &vehicle_propagator, &current_runtime, last_eval, last_cmd](
            const post2::vehicle::VehicleRuntimeState& previous_runtime,
            const post2::integrators::ExtendedState& integrated,
            double next_time_s,
            double step_s) {
            (void)step_s;
            post2::integrators::ExtendedState constrained = integrated;
            if (two_point_five_dof(phase_dof)) {
                constrained.motion = apply_two_point_five_dof_constraint(
                    constrained.motion,
                    dynamics_plane);
            }
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

        // Separation branching: when a stage detaches, snapshot the pre-detach
        // runtime + time. A detached-controller (recovery) phase then starts its
        // controlled stage from that separation state instead of inheriting the
        // main stack's continued (orbit) state, so one run can hold both the main
        // mission AND the booster's recovery in the same state log.
        struct SeparationSnapshot {
            int stage_index = -1;
            double time_s = 0.0;
            post2::vehicle::VehicleRuntimeState runtime;
        };
        std::vector<SeparationSnapshot> separation_snapshots;
        std::vector<int> recovered_stages;

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

            // Snapshot the pre-detach runtime for any stage this phase detaches
            // (one snapshot per stage, the first time it separates).
            for (const auto& action : phase.actions) {
                if (action.type != "set_stage_attached" || action.value ||
                    action.stage_index < 0) {
                    continue;
                }
                const bool have = std::any_of(
                    separation_snapshots.begin(), separation_snapshots.end(),
                    [&](const SeparationSnapshot& s) { return s.stage_index == action.stage_index; });
                if (!have) {
                    separation_snapshots.push_back(
                        {action.stage_index, phase_start_time_s, initial_runtime});
                }
            }

            // A recovery (detached-controller) phase begins its controlled stage
            // from that stage's separation snapshot rather than the inherited
            // state, and rewinds the phase clock to the separation time.
            bool branched_from_separation = false;
            if (phase.controller_detached_stage) {
                const int controlled = phase_controlled_stage_index(config, phase);
                const bool already = std::find(recovered_stages.begin(),
                    recovered_stages.end(), controlled) != recovered_stages.end();
                if (controlled >= 0 && !already) {
                    for (const auto& snap : separation_snapshots) {
                        if (snap.stage_index == controlled) {
                            initial_runtime = snap.runtime;
                            phase_start_time_s = snap.time_s;
                            recovered_stages.push_back(controlled);
                            branched_from_separation = true;
                            break;
                        }
                    }
                }
            }

            // At a phase switch, re-anchor any continuity-enabled throttle/
            // steering constants to the previous phase's final state. Skipped for
            // a branch (the recovery starts fresh from separation, not continuous
            // with the main stack's final state).
            if (!branched_from_separation && !state_log.empty()) {
                apply_phase_start_continuity(
                    &phase,
                    config,
                    initial_runtime,
                    state_log.back().throttle,
                    state_log.back().engine_direction_eci);
            }

            // On a 3-DOF -> 2.5-DOF switch that continues from the previous phase,
            // hand propagate_phase the previous phase's final 3-D thrust direction
            // so it can project the orientation into the new vertical plane and seed
            // the rigid-body pitch. Skipped when the previous phase was already
            // 2.5-DOF (its attitude is meaningful and inherited as-is) or when this
            // phase starts fresh (own initial state / separation branch).
            std::optional<Vec3> inherited_orientation_eci;
            if (phase.inherit_initial_state && !branched_from_separation &&
                !state_log.empty()) {
                DynamicsDof this_dof = DynamicsDof::ThreeDof;
                dynamics_dof_from_string(phase.dynamics_dof, &this_dof);
                DynamicsDof previous_dof = DynamicsDof::ThreeDof;
                if (phase_index > 0) {
                    dynamics_dof_from_string(
                        config.phases[phase_index - 1].dynamics_dof, &previous_dof);
                }
                if (this_dof == DynamicsDof::TwoPointFiveDof &&
                    previous_dof != DynamicsDof::TwoPointFiveDof) {
                    inherited_orientation_eci = state_log.back().engine_direction_eci;
                }
            }

            const StateLog phase_log = propagate_phase(
                config, phase, phase_index, phase_start_time_s, initial_runtime,
                &mission_events, inherited_orientation_eci);
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
    const bool has_source_phase =
        last.phase_index >= 0 &&
        static_cast<std::size_t>(last.phase_index) < case_config.phases.size();
    const PhaseConfig* source_phase = has_source_phase
        ? &case_config.phases[static_cast<std::size_t>(last.phase_index)]
        : (case_config.phases.empty() ? nullptr : &case_config.phases.back());
    phase.tolerances = source_phase
        ? source_phase->tolerances
        : post2::integrators::IntegratorTolerances{};
    phase.force_models = source_phase
        ? source_phase->force_models
        : ForceModelSwitches{};
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
    std::vector<post2::integrators::EventFunction> events;
    if (!out.empty() && out.back().altitude_m > 0.0) {
        events.push_back(post2::propagation::altitude_zero_event(true));
    }

    post2::integrators::ExtendedState state{
        coast_runtime.vehicle.motion,
        post2::vehicle::read_tank_masses_flat(coast_runtime),
        coast_runtime.vehicle.rigid_body,
    };
    post2::integrators::Dop853Integrator integrator(phase.tolerances);

    auto derivative = [&](double t_s, const post2::integrators::ExtendedState& eval_state) {
        post2::vehicle::VehicleRuntimeState eval_runtime = coast_runtime;
        eval_runtime.time_s = t_s;
        eval_runtime.vehicle.motion = eval_state.motion;
        eval_runtime.vehicle.rigid_body = eval_state.rigid_body;
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
                integrator.step(state, t_s, h, derivative, events);
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
            if (step.event.has_value()) {
                break;
            }
        }

        coast_runtime.time_s = t_s;
        coast_runtime.vehicle.motion = state.motion;
        const auto env = make_environment_state(
            simulation_config, phase, t_s, coast_runtime.vehicle.motion);
        const auto d = derivative(t_s, state);
        append_entry_with_environment(
            &out,
            coast_runtime,
            env,
            simulation_config.earth_rotation_rad_per_s,
            d.motion_dot.d_velocity_mps2);
        if (env.altitude_m <= 0.0) {
            break;
        }
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
        StateLog predicted = CoastPropagator{}.propagate(case_config, source_log, horizon_s, samples);
        if (!predicted.empty() && predicted.back().altitude_m <= 0.0) {
            return empty;
        }
        return predicted;
    } catch (const std::exception&) {
        return empty;
    }
}

StateLog predict_coast_path(
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

    double horizon_s = 1800.0;
    const double specific_energy = 0.5 * v_norm * v_norm - mu / r_norm;
    if (specific_energy < 0.0) {
        const double a = -mu / (2.0 * specific_energy);
        if (a > 0.0) {
            constexpr double kPi = 3.141592653589793238462643383279502884;
            horizon_s = 2.0 * kPi * std::sqrt((a * a * a) / mu) * 1.02;
        }
    } else {
        horizon_s = 3600.0;
    }

    const int samples = std::max(60, sample_count);
    try {
        return CoastPropagator{}.propagate(case_config, source_log, horizon_s, samples);
    } catch (const std::exception&) {
        return empty;
    }
}

std::vector<PredictedTrajectoryPath> predict_phase_end_trajectory_paths(
    const CaseConfig& case_config,
    const StateLog& source_log,
    int sample_count)
{
    std::vector<PredictedTrajectoryPath> predictions;
    if (source_log.empty()) {
        return predictions;
    }

    const auto& entries = source_log.entries();
    predictions.reserve(case_config.phases.empty() ? 1 : case_config.phases.size());
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const bool phase_ends_here =
            (i + 1 == entries.size()) ||
            entries[i + 1].phase_index != entries[i].phase_index;
        if (!phase_ends_here) {
            continue;
        }

        const LaunchVehicleStateLogEntry& phase_end = entries[i];
        StateLog source(case_config.earth_radius_m, case_config.vehicle);
        source.set_phase_metadata(phase_end.phase_index, phase_end.phase_name);
        source.append(phase_end);
        StateLog predicted = predict_coast_path(case_config, source, sample_count);
        if (predicted.empty()) {
            continue;
        }

        PredictedTrajectoryPath path;
        path.state_log = std::move(predicted);
        path.source_phase_index = phase_end.phase_index;
        path.source_phase_name = phase_end.phase_name;

        const bool has_phase =
            phase_end.phase_index >= 0 &&
            static_cast<std::size_t>(phase_end.phase_index) < case_config.phases.size();
        if (has_phase) {
            const PhaseConfig& phase =
                case_config.phases[static_cast<std::size_t>(phase_end.phase_index)];
            path.source_phase_name = path.source_phase_name.empty()
                ? phase.name
                : path.source_phase_name;
            path.controller_detached_stage = phase.controller_detached_stage;
            path.controller_stage_index = phase.controller_detached_stage
                ? phase_controller_stage_index(case_config, phase)
                : -1;
            path.controller_name =
                prediction_controller_name(case_config, phase, path.controller_stage_index);
        } else {
            path.controller_detached_stage = false;
            path.controller_stage_index = -1;
            path.controller_name = "root stack";
        }
        path.color = prediction_color_for_controller(
            path.controller_detached_stage,
            path.controller_stage_index);
        predictions.push_back(std::move(path));
    }

    return predictions;
}

} // namespace post2::core
