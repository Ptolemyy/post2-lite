#include "post2/core/case_config_io.hpp"

#include "post2/core/coordinates.hpp"
#include "post2/core/frames.hpp"
#include "post2/core/json.hpp"
#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle_config_io.hpp"

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <utility>

namespace post2::core {

namespace {

bool fail(std::string* error, const std::string& message)
{
    if (error) {
        *error = message;
    }
    return false;
}

JsonValue number(double value)
{
    return JsonValue::number(value);
}

JsonValue boolean(bool value)
{
    return JsonValue::boolean(value);
}

JsonValue string(const std::string& value)
{
    return JsonValue::string(value);
}

JsonValue vec3_to_json(const Vec3& value)
{
    return JsonValue::array({number(value.x), number(value.y), number(value.z)});
}

JsonValue state_to_json(const State& state)
{
    return JsonValue::object({
        {"position_m", vec3_to_json(state.position_m)},
        {"velocity_mps", vec3_to_json(state.velocity_mps)},
    });
}

JsonValue poly_to_json(const Poly2Config& poly)
{
    return JsonValue::object({
        {"c0", number(poly.c0)},
        {"c1", number(poly.c1)},
        {"c2", number(poly.c2)},
        {"continuity", boolean(poly.continuity)},
    });
}

JsonValue coefficients_to_json(const std::vector<double>& coefficients)
{
    JsonValue::Array out;
    out.reserve(coefficients.size());
    for (const double value : coefficients) {
        out.push_back(number(value));
    }
    return JsonValue::array(std::move(out));
}

JsonValue segmented_poly_to_json(const SegmentedPolyConfig& poly)
{
    JsonValue::Array segments;
    segments.reserve(poly.segments.size());
    for (const auto& segment : poly.segments) {
        segments.push_back(JsonValue::object({
            {"start_time_s", number(segment.start_time_s)},
            {"coefficients", coefficients_to_json(segment.coefficients)},
        }));
    }
    return JsonValue::object({
        {"order", number(static_cast<double>(poly.order))},
        {"continuity", boolean(poly.continuity)},
        {"segments", JsonValue::array(std::move(segments))},
    });
}

JsonValue segmented_steering_poly_to_json(const SegmentedSteeringPolyConfig& poly)
{
    JsonValue::Array segments;
    segments.reserve(poly.segments.size());
    for (const auto& segment : poly.segments) {
        segments.push_back(JsonValue::object({
            {"start_time_s", number(segment.start_time_s)},
            {"azimuth", coefficients_to_json(segment.azimuth_coefficients)},
            {"elevation", coefficients_to_json(segment.elevation_coefficients)},
        }));
    }
    return JsonValue::object({
        {"order", number(static_cast<double>(poly.order))},
        {"continuity", boolean(poly.continuity)},
        {"segments", JsonValue::array(std::move(segments))},
    });
}

JsonValue quaternion_to_json(const Quaternion& quat)
{
    return JsonValue::array({number(quat.w), number(quat.x), number(quat.y), number(quat.z)});
}

JsonValue tank_ref_to_json(const post2::vehicle::TankRef& ref)
{
    return JsonValue::object({
        {"stage", string(ref.stage_name)},
        {"tank", string(ref.tank_name)},
    });
}

JsonValue feed_tanks_to_json(const std::vector<post2::vehicle::TankRef>& feed_tanks)
{
    JsonValue::Array out;
    out.reserve(feed_tanks.size());
    for (const auto& ref : feed_tanks) {
        out.push_back(tank_ref_to_json(ref));
    }
    return JsonValue::array(std::move(out));
}

JsonValue throttle_curve_to_json(
    const std::vector<post2::vehicle::EngineThrottleCurvePoint>& curve)
{
    JsonValue::Array out;
    out.reserve(curve.size());
    for (const auto& point : curve) {
        out.push_back(JsonValue::object({
            {"throttle", number(point.throttle)},
            {"mdot_ratio", number(point.mdot_ratio)},
        }));
    }
    return JsonValue::array(std::move(out));
}

JsonValue engine_to_json(const post2::vehicle::EngineConfig& engine)
{
    return JsonValue::object({
        {"enabled", boolean(engine.enabled)},
        {"thrust_vac_n", number(engine.thrust_vac_n)},
        {"isp_vac_s", number(engine.isp_vac_s)},
        {"thrust_sl_n", number(engine.thrust_sl_n)},
        {"isp_sl_s", number(engine.isp_sl_s)},
        {"nozzle_exit_area_m2", number(engine.nozzle_exit_area_m2)},
        {"min_throttle", number(engine.min_throttle)},
        {"max_throttle", number(engine.max_throttle)},
        {"throttle_curve", throttle_curve_to_json(engine.throttle_curve)},
        {"ignition_delay_s", number(engine.ignition_delay_s)},
        {"spool_up_rate_per_s", number(engine.spool_up_rate_per_s)},
        {"spool_down_rate_per_s", number(engine.spool_down_rate_per_s)},
        {"thrust_buildup_s", number(engine.thrust_buildup_s)},
        {"shutdown_delay_s", number(engine.shutdown_delay_s)},
        {"engine_count", number(static_cast<double>(engine.engine_count))},
        {"gimbal_max_rad", number(engine.gimbal_max_rad)},
        {"gimbal_rate_rad_s", number(engine.gimbal_rate_rad_s)},
        {"direction_body", vec3_to_json(engine.direction_body)},
        {"feed_tanks", feed_tanks_to_json(engine.feed_tanks)},
    });
}

JsonValue rigid_body_to_json(const post2::vehicle::RigidBodyConfig& rigid_body)
{
    return JsonValue::object({
        {"moment_of_inertia_kgm2", number(rigid_body.moment_of_inertia_kgm2)},
        {"initial_attitude_rad", number(rigid_body.initial_attitude_rad)},
        {"initial_angular_velocity_radps",
         number(rigid_body.initial_angular_velocity_radps)},
        {"engine_moment_arm_m", number(rigid_body.engine_moment_arm_m)},
    });
}

JsonValue gravity_model_to_json(const GravityModelConfig& model)
{
    return JsonValue::object({
        {"type", string(model.type)},
        {"j2", number(model.j2)},
        {"degree", number(static_cast<double>(model.degree))},
        {"order", number(static_cast<double>(model.order))},
    });
}

JsonValue atmosphere_model_to_json(const AtmosphereModelConfig& model)
{
    return JsonValue::object({
        {"type", string(model.type)},
        {"table_path", string(model.table_path)},
    });
}

JsonValue aero_model_to_json(const AeroModelConfig& model)
{
    return JsonValue::object({
        {"type", string(model.type)},
    });
}

JsonValue aero_stage_table_to_json(const post2::vehicle::AeroStageTable& entry)
{
    return JsonValue::object({
        {"activate_at_min_attached_stage",
         number(static_cast<double>(entry.activate_at_min_attached_stage))},
        {"table_path", string(entry.table_path)},
        {"reference_area_m2", number(entry.reference_area_m2)},
        {"ref_diameter_m", number(entry.ref_diameter_m)},
        {"body_length_m", number(entry.body_length_m)},
        {"nose_length_m", number(entry.nose_length_m)},
        {"base_diameter_m", number(entry.base_diameter_m)},
        {"max_attached_stage",
         number(static_cast<double>(entry.max_attached_stage))},
    });
}

JsonValue aero_to_json(const post2::vehicle::AeroConfig& aero)
{
    JsonValue::Array stage_tables;
    stage_tables.reserve(aero.stage_tables.size());
    for (const auto& entry : aero.stage_tables) {
        stage_tables.push_back(aero_stage_table_to_json(entry));
    }
    return JsonValue::object({
        {"enabled", boolean(aero.enabled)},
        {"reference_area_m2", number(aero.reference_area_m2)},
        {"cd", number(aero.cd)},
        {"cl", number(aero.cl)},
        {"aero_table_path", string(aero.aero_table_path)},
        {"use_table", boolean(aero.use_table)},
        {"ref_diameter_m", number(aero.ref_diameter_m)},
        {"body_length_m", number(aero.body_length_m)},
        {"nose_length_m", number(aero.nose_length_m)},
        {"base_diameter_m", number(aero.base_diameter_m)},
        {"stage_tables", JsonValue::array(std::move(stage_tables))},
        {"first_stage_table", aero_stage_table_to_json(aero.first_stage_table)},
    });
}

JsonValue force_models_to_json(const ForceModelSwitches& force_models)
{
    return JsonValue::object({
        {"gravity", boolean(force_models.gravity)},
        {"thrust", boolean(force_models.thrust)},
        {"normal_force", boolean(force_models.normal_force)},
        {"aerodynamic", boolean(force_models.aerodynamic)},
        {"third_body", boolean(force_models.third_body)},
        {"solar_radiation_pressure", boolean(force_models.solar_radiation_pressure)},
        {"gravity_model", gravity_model_to_json(force_models.gravity_model)},
        {"atmosphere_model", atmosphere_model_to_json(force_models.atmosphere_model)},
        {"aero_model", aero_model_to_json(force_models.aero_model)},
    });
}

JsonValue vehicle_to_json(const post2::vehicle::VehicleConfig& vehicle)
{
    post2::vehicle::VehicleConfig normalized_vehicle = vehicle;
    if (normalized_vehicle.stages.empty()) {
        normalized_vehicle.stages = post2::vehicle::effective_stage_configs(normalized_vehicle);
    }
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&normalized_vehicle);

    JsonValue::Array tanks;
    for (const auto& tank : normalized_vehicle.tanks) {
        tanks.push_back(JsonValue::object({
            {"name", string(tank.name)},
            {"propellant", string(tank.propellant)},
            {"capacity_kg", number(tank.capacity_kg)},
            {"initial_kg", number(tank.initial_kg)},
        }));
    }

    JsonValue::Array stages;
    for (const auto& stage : normalized_vehicle.stages) {
        JsonValue::Array stage_tanks;
        for (const auto& tank : stage.tanks) {
            stage_tanks.push_back(JsonValue::object({
                {"name", string(tank.name)},
                {"propellant", string(tank.propellant)},
                {"capacity_kg", number(tank.capacity_kg)},
                {"initial_kg", number(tank.initial_kg)},
            }));
        }
        stages.push_back(JsonValue::object({
            {"name", string(stage.name)},
            {"active", boolean(stage.active)},
            {"attached", boolean(stage.attached)},
            {"dry_mass_kg", number(stage.dry_mass_kg)},
            {"engine", engine_to_json(stage.engine)},
            {"tanks", JsonValue::array(std::move(stage_tanks))},
        }));
    }

    JsonValue::Array t2t;
    t2t.reserve(normalized_vehicle.tank_to_tank_connections.size());
    for (const auto& c : normalized_vehicle.tank_to_tank_connections) {
        JsonValue::Object entry{
            {"source", tank_ref_to_json(c.source)},
            {"dest", tank_ref_to_json(c.dest)},
            {"rate_kgps", number(c.rate_kgps)},
        };
        if (c.start_time_s.has_value()) {
            entry["start_time_s"] = number(*c.start_time_s);
        }
        if (c.end_time_s.has_value()) {
            entry["end_time_s"] = number(*c.end_time_s);
        }
        t2t.push_back(JsonValue::object(std::move(entry)));
    }

    return JsonValue::object({
        {"name", string(normalized_vehicle.name)},
        {"dry_mass_kg", number(normalized_vehicle.dry_mass_kg)},
        {"rigid_body", rigid_body_to_json(normalized_vehicle.rigid_body)},
        {"aero", aero_to_json(normalized_vehicle.aero)},
        {"engine", engine_to_json(normalized_vehicle.engine)},
        {"tanks", JsonValue::array(std::move(tanks))},
        {"stages", JsonValue::array(std::move(stages))},
        {"tank_to_tank_connections", JsonValue::array(std::move(t2t))},
    });
}

JsonValue throttle_to_json(const ThrottleModelConfig& throttle)
{
    JsonValue::Array points;
    for (const auto& point : throttle.points) {
        points.push_back(JsonValue::object({
            {"time_s", number(point.time_s)},
            {"throttle", number(point.throttle)},
        }));
    }

    return JsonValue::object({
        {"type", string(throttle.type)},
        {"c0", number(throttle.c0)},
        {"c1", number(throttle.c1)},
        {"c2", number(throttle.c2)},
        {"target_t2w", number(throttle.target_t2w)},
        {"continuity", boolean(throttle.continuity)},
        {"points", JsonValue::array(std::move(points))},
        {"segmented_poly", segmented_poly_to_json(throttle.segmented_poly)},
    });
}

JsonValue tangent_to_json(const LinearTangentConfig& tangent)
{
    return JsonValue::object({
        {"a", number(tangent.a)},
        {"a_dot", number(tangent.a_dot)},
        {"b", number(tangent.b)},
        {"b_dot", number(tangent.b_dot)},
        {"t_offset_s", number(tangent.t_offset_s)},
        {"continuity", boolean(tangent.continuity)},
    });
}

JsonValue upfg_to_json(const UpfgConfig& upfg)
{
    return JsonValue::object({
        {"periapsis_km", number(upfg.periapsis_km)},
        {"apoapsis_km", number(upfg.apoapsis_km)},
        {"inclination_deg", number(upfg.inclination_deg)},
    });
}

JsonValue steering_to_json(const SteeringModelConfig& steering)
{
    JsonValue::Array points;
    for (const auto& point : steering.points) {
        points.push_back(JsonValue::object({
            {"time_s", number(point.time_s)},
            {"quat", quaternion_to_json(point.quat)},
        }));
    }

    JsonValue::Array segments;
    for (const auto& segment : steering.segments) {
        segments.push_back(JsonValue::object({
            {"start_time_s", number(segment.start_time_s)},
            {"model", steering_to_json(segment.model ? *segment.model : SteeringModelConfig{})},
        }));
    }

    return JsonValue::object({
        {"type", string(steering.type)},
        {"roll", poly_to_json(steering.roll_deg)},
        {"pitch", poly_to_json(steering.pitch_deg)},
        {"yaw", poly_to_json(steering.yaw_deg)},
        {"azimuth", poly_to_json(steering.azimuth_deg)},
        {"elevation", poly_to_json(steering.elevation_deg)},
        {"tangent", tangent_to_json(steering.tangent)},
        {"upfg", upfg_to_json(steering.upfg)},
        {"fixed_direction_eci", vec3_to_json(steering.fixed_direction_eci)},
        {"points", JsonValue::array(std::move(points))},
        {"segments", JsonValue::array(std::move(segments))},
        {"segmented_poly", segmented_steering_poly_to_json(steering.segmented_poly)},
    });
}

JsonValue phase_action_to_json(const PhaseAction& action)
{
    return JsonValue::object({
        {"time_s", number(action.time_s)},
        {"type", string(action.type)},
        {"value", boolean(action.value)},
        {"stage_index", number(action.stage_index)},
        {"stage_name", string(action.stage_name)},
    });
}

JsonValue trigger_to_json(const TriggerCondition& trigger)
{
    return JsonValue::object({
        {"type", string(trigger.type)},
        {"comparison", string(trigger.comparison)},
        {"value", number(trigger.value)},
    });
}

JsonValue event_to_json(const EventConfig& event)
{
    JsonValue::Array actions;
    actions.reserve(event.actions.size());
    for (const auto& action : event.actions) {
        actions.push_back(phase_action_to_json(action));
    }
    return JsonValue::object({
        {"name", string(event.name)},
        {"enabled", boolean(event.enabled)},
        {"trigger", trigger_to_json(event.trigger)},
        {"actions", JsonValue::array(std::move(actions))},
    });
}

JsonValue phase_to_json(const PhaseConfig& phase)
{
    JsonValue::Array actions;
    for (const auto& action : phase.actions) {
        actions.push_back(phase_action_to_json(action));
    }

    JsonValue::Object object{
        {"name", string(phase.name)},
        {"controller_stage_index", number(static_cast<double>(phase.controller_stage_index))},
        {"controller_stage_name", string(phase.controller_stage_name)},
        {"controller_detached_stage", boolean(phase.controller_detached_stage)},
        {"termination", trigger_to_json(phase.termination)},
        {"optimize_enabled", boolean(phase.optimize_enabled)},
        {"inherit_initial_state", boolean(phase.inherit_initial_state)},
        {"hold_down_clamp_initial_active", boolean(phase.hold_down_clamp_initial_active)},
        {"dynamics_dof", string(phase.dynamics_dof)},
        {"integrator", string(phase.integrator)},
        {"tolerances", JsonValue::object({
            {"rtol", number(phase.tolerances.rtol)},
            {"atol_position_m", number(phase.tolerances.atol_position_m)},
            {"atol_velocity_mps", number(phase.tolerances.atol_velocity_mps)},
            {"atol_tank_mass_kg", number(phase.tolerances.atol_tank_mass_kg)},
            {"atol_attitude_rad", number(phase.tolerances.atol_attitude_rad)},
            {"atol_angular_velocity_radps",
             number(phase.tolerances.atol_angular_velocity_radps)},
        })},
        {"force_models", force_models_to_json(phase.force_models)},
        {"throttle_model", throttle_to_json(phase.throttle_model)},
        {"steering_model", steering_to_json(phase.steering_model)},
        {"actions", JsonValue::array(std::move(actions))},
    };
    if (phase.initial_state_eci.has_value()) {
        object["initial_state_eci"] = state_to_json(*phase.initial_state_eci);
    }
    return JsonValue::object(std::move(object));
}

JsonValue objective_to_json(const OptimizationObjectiveConfig& objective)
{
    return JsonValue::object({
        {"enabled", boolean(objective.enabled)},
        {"metric", string(objective.metric)},
        {"direction", string(objective.direction)},
        {"weight", number(objective.weight)},
    });
}

std::vector<OptimizationObjectiveConfig> effective_objective_configs(
    const OptimizationConfig& optimization)
{
    if (!optimization.objectives.empty()) {
        return optimization.objectives;
    }
    return {optimization.objective};
}

JsonValue optimization_to_json(const OptimizationConfig& optimization)
{
    JsonValue::Array variables;
    for (const auto& variable : optimization.variables) {
        variables.push_back(JsonValue::object({
            {"path", string(variable.path)},
            {"enabled", boolean(variable.enabled)},
            {"min_value", number(variable.min_value)},
            {"max_value", number(variable.max_value)},
        }));
    }

    JsonValue::Array targets;
    for (const auto& target : optimization.targets) {
        targets.push_back(JsonValue::object({
            {"metric", string(target.metric)},
            {"mode", string(target.mode)},
            {"scope", string(target.scope)},
            {"phase_index", number(static_cast<double>(target.phase_index))},
            {"value", number(target.value)},
            {"min_value", number(target.min_value)},
            {"max_value", number(target.max_value)},
            {"weight", number(target.weight)},
        }));
    }

    const std::vector<OptimizationObjectiveConfig> effective_objectives =
        effective_objective_configs(optimization);
    JsonValue::Array objectives;
    objectives.reserve(effective_objectives.size());
    for (const auto& objective : effective_objectives) {
        objectives.push_back(objective_to_json(objective));
    }
    const OptimizationObjectiveConfig legacy_objective =
        effective_objectives.empty()
            ? OptimizationObjectiveConfig{}
            : effective_objectives.front();

    return JsonValue::object({
        {"mode", string(optimization.mode)},
        {"optimizer", string(optimization.optimizer)},
        {"qp_solver", string(optimization.qp_solver)},
        {"fd_mode", string(optimization.fd_mode)},
        {"max_iterations", number(optimization.max_iterations)},
        {"tolerance", number(optimization.tolerance)},
        {"stationarity_tolerance", number(optimization.stationarity_tolerance)},
        {"feasibility_tolerance", number(optimization.feasibility_tolerance)},
        {"constraint_tolerance", number(optimization.constraint_tolerance)},
        {"initial_step_fraction", number(optimization.initial_step_fraction)},
        {"parallel_fd", boolean(optimization.parallel_fd)},
        {"max_restoration_iterations", number(optimization.max_restoration_iterations)},
        {"variables", JsonValue::array(std::move(variables))},
        {"targets", JsonValue::array(std::move(targets))},
        {"objectives", JsonValue::array(std::move(objectives))},
        {"objective", objective_to_json(legacy_objective)},
        {"envelope_search", JsonValue::object({
            {"enabled", boolean(optimization.envelope_search.enabled)},
            {"sample_count", number(static_cast<double>(optimization.envelope_search.sample_count))},
            {"seed", number(static_cast<double>(optimization.envelope_search.seed))},
        })},
        {"continuation", JsonValue::object({
            {"enabled", boolean(optimization.continuation.enabled)},
            {"mode", string(optimization.continuation.mode)},
            {"variable_path", string(optimization.continuation.variable_path)},
            {"direction", string(optimization.continuation.direction)},
            {"steps", number(static_cast<double>(optimization.continuation.steps))},
            {"multistart_enabled", boolean(optimization.continuation.multistart_enabled)},
            {"multistart_count", number(static_cast<double>(optimization.continuation.multistart_count))},
        })},
    });
}

JsonValue case_to_json_value(const CaseConfig& config)
{
    JsonValue::Array phases;
    for (const auto& phase : config.phases) {
        phases.push_back(phase_to_json(phase));
    }

    JsonValue::Array events;
    events.reserve(config.events.size());
    for (const auto& event : config.events) {
        events.push_back(event_to_json(event));
    }

    return JsonValue::object({
        {"name", string(config.name)},
        {"earth_radius_m", number(config.earth_radius_m)},
        {"earth_mu_m3s2", number(config.earth_mu_m3s2)},
        {"earth_j2", number(config.earth_j2)},
        {"earth_rotation_rad_per_s", number(config.earth_rotation_rad_per_s)},
        {"step_s", number(config.step_s)},
        {"epoch_utc", JsonValue::object({
            {"year", number(static_cast<double>(config.epoch_utc.year))},
            {"month", number(static_cast<double>(config.epoch_utc.month))},
            {"day", number(static_cast<double>(config.epoch_utc.day))},
            {"hour", number(static_cast<double>(config.epoch_utc.hour))},
            {"minute", number(static_cast<double>(config.epoch_utc.minute))},
            {"second", number(config.epoch_utc.second)},
        })},
        {"launch_site", JsonValue::object({
            {"latitude_deg", number(config.launch_site.latitude_deg)},
            {"longitude_deg", number(config.launch_site.longitude_deg)},
            {"altitude_m", number(config.launch_site.altitude_m)},
        })},
        {"vehicle", vehicle_to_json(config.vehicle)},
        {"phases", JsonValue::array(std::move(phases))},
        {"events", JsonValue::array(std::move(events))},
        {"optimization", optimization_to_json(config.optimization)},
    });
}

bool read_string(const JsonValue& object, const char* key, std::string* target, std::string* error)
{
    const JsonValue* value = find_member(object, key);
    if (!value) {
        return true;
    }
    if (!value->is_string()) {
        return fail(error, std::string(key) + " must be a string");
    }
    *target = value->string_value;
    return true;
}

bool read_number(const JsonValue& object, const char* key, double* target, std::string* error)
{
    const JsonValue* value = find_member(object, key);
    if (!value) {
        return true;
    }
    if (!value->is_number()) {
        return fail(error, std::string(key) + " must be a number");
    }
    *target = value->number_value;
    return true;
}

bool read_bool(const JsonValue& object, const char* key, bool* target, std::string* error)
{
    const JsonValue* value = find_member(object, key);
    if (!value) {
        return true;
    }
    if (value->is_bool()) {
        *target = value->bool_value;
        return true;
    }
    if (value->is_number()) {
        *target = value->number_value != 0.0;
        return true;
    }
    return fail(error, std::string(key) + " must be a bool");
}

bool parse_vec3(const JsonValue& value, Vec3* target, std::string* error)
{
    if (value.is_array()) {
        if (value.array_value.size() != 3) {
            return fail(error, "Vec3 array must have three numbers");
        }
        for (const auto& member : value.array_value) {
            if (!member.is_number()) {
                return fail(error, "Vec3 array members must be numbers");
            }
        }
        *target = {value.array_value[0].number_value, value.array_value[1].number_value, value.array_value[2].number_value};
        return true;
    }

    if (value.is_object()) {
        Vec3 parsed = *target;
        if (!read_number(value, "x", &parsed.x, error) ||
            !read_number(value, "y", &parsed.y, error) ||
            !read_number(value, "z", &parsed.z, error)) {
            return false;
        }
        *target = parsed;
        return true;
    }

    return fail(error, "Vec3 must be an array or object");
}

bool read_vec3(const JsonValue& object, const char* key, Vec3* target, std::string* error)
{
    const JsonValue* value = find_member(object, key);
    return !value || parse_vec3(*value, target, error);
}

bool parse_state(const JsonValue& value, State* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "initial_state_eci must be an object");
    }
    State parsed = *target;
    if (!read_vec3(value, "position_m", &parsed.position_m, error) ||
        !read_vec3(value, "velocity_mps", &parsed.velocity_mps, error)) {
        return false;
    }
    *target = parsed;
    return true;
}

bool parse_poly(const JsonValue& value, Poly2Config* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "poly config must be an object");
    }
    Poly2Config parsed = *target;
    if (!read_number(value, "c0", &parsed.c0, error) ||
        !read_number(value, "c1", &parsed.c1, error) ||
        !read_number(value, "c2", &parsed.c2, error) ||
        !read_bool(value, "continuity", &parsed.continuity, error)) {
        return false;
    }
    *target = parsed;
    return true;
}

bool read_poly(const JsonValue& object, const char* key, Poly2Config* target, std::string* error)
{
    const JsonValue* value = find_member(object, key);
    return !value || parse_poly(*value, target, error);
}

bool parse_coefficients(const JsonValue& value, std::vector<double>* target, std::string* error)
{
    if (!value.is_array()) {
        return fail(error, "coefficients must be an array");
    }
    std::vector<double> parsed;
    parsed.reserve(value.array_value.size());
    for (const auto& member : value.array_value) {
        if (!member.is_number()) {
            return fail(error, "coefficient entries must be numbers");
        }
        parsed.push_back(member.number_value);
    }
    *target = std::move(parsed);
    return true;
}

bool parse_segmented_poly(const JsonValue& value, SegmentedPolyConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "segmented_poly must be an object");
    }
    SegmentedPolyConfig parsed = *target;
    double order = static_cast<double>(parsed.order);
    if (!read_number(value, "order", &order, error) ||
        !read_bool(value, "continuity", &parsed.continuity, error)) {
        return false;
    }
    parsed.order = std::max(0, static_cast<int>(order));
    if (const JsonValue* segments = find_member(value, "segments")) {
        if (!segments->is_array()) {
            return fail(error, "segmented_poly.segments must be an array");
        }
        parsed.segments.clear();
        parsed.segments.reserve(segments->array_value.size());
        for (const auto& segment_value : segments->array_value) {
            if (!segment_value.is_object()) {
                return fail(error, "segmented_poly segment must be an object");
            }
            SegmentedPolySegmentConfig segment;
            if (!read_number(segment_value, "start_time_s", &segment.start_time_s, error)) {
                return false;
            }
            if (const JsonValue* coefficients = find_member(segment_value, "coefficients")) {
                if (!parse_coefficients(*coefficients, &segment.coefficients, error)) {
                    return false;
                }
            }
            segment.coefficients.resize(static_cast<std::size_t>(parsed.order + 1), 0.0);
            parsed.segments.push_back(std::move(segment));
        }
    }
    *target = std::move(parsed);
    return true;
}

bool read_segmented_poly(const JsonValue& object, const char* key, SegmentedPolyConfig* target, std::string* error)
{
    const JsonValue* value = find_member(object, key);
    return !value || parse_segmented_poly(*value, target, error);
}

bool parse_segmented_steering_poly(
    const JsonValue& value,
    SegmentedSteeringPolyConfig* target,
    std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "steering_model.segmented_poly must be an object");
    }
    SegmentedSteeringPolyConfig parsed = *target;
    double order = static_cast<double>(parsed.order);
    if (!read_number(value, "order", &order, error) ||
        !read_bool(value, "continuity", &parsed.continuity, error)) {
        return false;
    }
    parsed.order = std::max(0, static_cast<int>(order));
    if (const JsonValue* segments = find_member(value, "segments")) {
        if (!segments->is_array()) {
            return fail(error, "steering_model.segmented_poly.segments must be an array");
        }
        parsed.segments.clear();
        parsed.segments.reserve(segments->array_value.size());
        for (const auto& segment_value : segments->array_value) {
            if (!segment_value.is_object()) {
                return fail(error, "steering segmented_poly segment must be an object");
            }
            SegmentedSteeringPolySegmentConfig segment;
            if (!read_number(segment_value, "start_time_s", &segment.start_time_s, error)) {
                return false;
            }
            if (const JsonValue* azimuth = find_member(segment_value, "azimuth")) {
                if (!parse_coefficients(*azimuth, &segment.azimuth_coefficients, error)) {
                    return false;
                }
            }
            if (const JsonValue* elevation = find_member(segment_value, "elevation")) {
                if (!parse_coefficients(*elevation, &segment.elevation_coefficients, error)) {
                    return false;
                }
            }
            segment.azimuth_coefficients.resize(static_cast<std::size_t>(parsed.order + 1), 0.0);
            segment.elevation_coefficients.resize(static_cast<std::size_t>(parsed.order + 1), 0.0);
            parsed.segments.push_back(std::move(segment));
        }
    }
    *target = std::move(parsed);
    return true;
}

bool read_segmented_steering_poly(
    const JsonValue& object,
    const char* key,
    SegmentedSteeringPolyConfig* target,
    std::string* error)
{
    const JsonValue* value = find_member(object, key);
    return !value || parse_segmented_steering_poly(*value, target, error);
}

bool parse_tangent(const JsonValue& value, LinearTangentConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "steering_model.tangent must be an object");
    }
    LinearTangentConfig parsed = *target;
    if (!read_number(value, "a", &parsed.a, error) ||
        !read_number(value, "a_dot", &parsed.a_dot, error) ||
        !read_number(value, "b", &parsed.b, error) ||
        !read_number(value, "b_dot", &parsed.b_dot, error) ||
        !read_number(value, "t_offset_s", &parsed.t_offset_s, error) ||
        !read_bool(value, "continuity", &parsed.continuity, error)) {
        return false;
    }
    *target = parsed;
    return true;
}

bool read_tangent(const JsonValue& object, const char* key, LinearTangentConfig* target, std::string* error)
{
    const JsonValue* value = find_member(object, key);
    return !value || parse_tangent(*value, target, error);
}

bool parse_upfg(const JsonValue& value, UpfgConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "steering_model.upfg must be an object");
    }
    UpfgConfig parsed = *target;
    if (!read_number(value, "periapsis_km", &parsed.periapsis_km, error) ||
        !read_number(value, "apoapsis_km", &parsed.apoapsis_km, error) ||
        !read_number(value, "inclination_deg", &parsed.inclination_deg, error)) {
        return false;
    }
    *target = parsed;
    return true;
}

bool read_upfg(const JsonValue& object, const char* key, UpfgConfig* target, std::string* error)
{
    const JsonValue* value = find_member(object, key);
    return !value || parse_upfg(*value, target, error);
}

bool parse_gravity_model(const JsonValue& value, GravityModelConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "gravity_model must be an object");
    }

    GravityModelConfig parsed = *target;
    double degree = static_cast<double>(parsed.degree);
    double order = static_cast<double>(parsed.order);
    if (!read_string(value, "type", &parsed.type, error) ||
        !read_number(value, "j2", &parsed.j2, error) ||
        !read_number(value, "degree", &degree, error) ||
        !read_number(value, "order", &order, error)) {
        return false;
    }
    parsed.degree = static_cast<int>(degree);
    parsed.order = static_cast<int>(order);
    *target = parsed;
    return true;
}

bool read_gravity_model(const JsonValue& object, const char* key, GravityModelConfig* target, std::string* error)
{
    const JsonValue* value = find_member(object, key);
    return !value || parse_gravity_model(*value, target, error);
}

bool parse_atmosphere_model(const JsonValue& value, AtmosphereModelConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "atmosphere_model must be an object");
    }

    AtmosphereModelConfig parsed = *target;
    if (!read_string(value, "type", &parsed.type, error) ||
        !read_string(value, "table_path", &parsed.table_path, error)) {
        return false;
    }
    *target = parsed;
    return true;
}

bool read_atmosphere_model(const JsonValue& object, const char* key, AtmosphereModelConfig* target, std::string* error)
{
    const JsonValue* value = find_member(object, key);
    return !value || parse_atmosphere_model(*value, target, error);
}

bool parse_aero_model(const JsonValue& value, AeroModelConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "aero_model must be an object");
    }

    AeroModelConfig parsed = *target;
    if (!read_string(value, "type", &parsed.type, error)) {
        return false;
    }
    *target = parsed;
    return true;
}

bool read_aero_model(const JsonValue& object, const char* key, AeroModelConfig* target, std::string* error)
{
    const JsonValue* value = find_member(object, key);
    return !value || parse_aero_model(*value, target, error);
}

bool parse_aero_stage_table(const JsonValue& item, post2::vehicle::AeroStageTable* entry,
                            std::string* error)
{
    if (!item.is_object()) {
        return fail(error, "vehicle.aero stage table entries must be objects");
    }
    double activate = 0.0;
    double max_attached = static_cast<double>(entry->max_attached_stage);
    if (!read_number(item, "activate_at_min_attached_stage", &activate, error) ||
        !read_string(item, "table_path", &entry->table_path, error) ||
        !read_number(item, "reference_area_m2", &entry->reference_area_m2, error) ||
        !read_number(item, "ref_diameter_m", &entry->ref_diameter_m, error) ||
        !read_number(item, "body_length_m", &entry->body_length_m, error) ||
        !read_number(item, "nose_length_m", &entry->nose_length_m, error) ||
        !read_number(item, "base_diameter_m", &entry->base_diameter_m, error) ||
        // Optional; defaults to -1 (open-top) when absent for back-compat.
        !read_number(item, "max_attached_stage", &max_attached, error)) {
        return false;
    }
    entry->activate_at_min_attached_stage = static_cast<int>(activate);
    entry->max_attached_stage = static_cast<int>(max_attached);
    return true;
}

bool parse_vehicle_aero(const JsonValue& value, post2::vehicle::AeroConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "vehicle.aero must be an object");
    }

    post2::vehicle::AeroConfig parsed = *target;
    if (!read_bool(value, "enabled", &parsed.enabled, error) ||
        !read_number(value, "reference_area_m2", &parsed.reference_area_m2, error) ||
        !read_number(value, "cd", &parsed.cd, error) ||
        !read_number(value, "cl", &parsed.cl, error) ||
        !read_string(value, "aero_table_path", &parsed.aero_table_path, error) ||
        !read_bool(value, "use_table", &parsed.use_table, error) ||
        !read_number(value, "ref_diameter_m", &parsed.ref_diameter_m, error) ||
        !read_number(value, "body_length_m", &parsed.body_length_m, error) ||
        !read_number(value, "nose_length_m", &parsed.nose_length_m, error) ||
        !read_number(value, "base_diameter_m", &parsed.base_diameter_m, error)) {
        return false;
    }
    if (const JsonValue* tables = find_member(value, "stage_tables")) {
        if (!tables->is_array()) {
            return fail(error, "vehicle.aero.stage_tables must be an array");
        }
        parsed.stage_tables.clear();
        parsed.stage_tables.reserve(tables->array_value.size());
        for (const JsonValue& item : tables->array_value) {
            post2::vehicle::AeroStageTable entry;
            if (!parse_aero_stage_table(item, &entry, error)) {
                return false;
            }
            parsed.stage_tables.push_back(std::move(entry));
        }
    }
    if (const JsonValue* first = find_member(value, "first_stage_table")) {
        if (!parse_aero_stage_table(*first, &parsed.first_stage_table, error)) {
            return false;
        }
    }
    *target = std::move(parsed);
    return true;
}

bool parse_rigid_body(
    const JsonValue& value,
    post2::vehicle::RigidBodyConfig* target,
    std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "vehicle.rigid_body must be an object");
    }

    post2::vehicle::RigidBodyConfig parsed = *target;
    if (!read_number(value, "moment_of_inertia_kgm2", &parsed.moment_of_inertia_kgm2, error) ||
        !read_number(value, "initial_attitude_rad", &parsed.initial_attitude_rad, error) ||
        !read_number(
            value,
            "initial_angular_velocity_radps",
            &parsed.initial_angular_velocity_radps,
            error) ||
        !read_number(value, "engine_moment_arm_m", &parsed.engine_moment_arm_m, error)) {
        return false;
    }
    *target = parsed;
    return true;
}

bool parse_quaternion(const JsonValue& value, Quaternion* target, std::string* error)
{
    if (!value.is_array() || value.array_value.size() != 4) {
        return fail(error, "quat must be [w,x,y,z]");
    }
    for (const auto& member : value.array_value) {
        if (!member.is_number()) {
            return fail(error, "quat members must be numbers");
        }
    }
    *target = {
        value.array_value[0].number_value,
        value.array_value[1].number_value,
        value.array_value[2].number_value,
        value.array_value[3].number_value,
    };
    return true;
}

bool parse_vehicle(const JsonValue& value, post2::vehicle::VehicleConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "vehicle must be an object");
    }

    post2::vehicle::VehicleConfig parsed = *target;
    if (!read_string(value, "name", &parsed.name, error) ||
        !read_number(value, "dry_mass_kg", &parsed.dry_mass_kg, error)) {
        return false;
    }
    if (const JsonValue* aero = find_member(value, "aero")) {
        if (!parse_vehicle_aero(*aero, &parsed.aero, error)) {
            return false;
        }
    }
    if (const JsonValue* rigid_body = find_member(value, "rigid_body")) {
        if (!parse_rigid_body(*rigid_body, &parsed.rigid_body, error)) {
            return false;
        }
    }

    auto parse_tank_ref = [&](const JsonValue& v, post2::vehicle::TankRef* ref) -> bool {
        if (!v.is_object()) {
            return fail(error, "tank ref must be an object {stage, tank}");
        }
        return read_string(v, "stage", &ref->stage_name, error) &&
            read_string(v, "tank", &ref->tank_name, error);
    };

    auto parse_engine = [&](const JsonValue& engine, post2::vehicle::EngineConfig* engine_config) -> bool {
        if (!engine.is_object()) {
            return fail(error, "vehicle.engine must be an object");
        }
        if (!read_bool(engine, "enabled", &engine_config->enabled, error)) {
            return false;
        }
        // Thrust: prefer new name, fall back to legacy max_thrust_n.
        if (find_member(engine, "thrust_vac_n")) {
            if (!read_number(engine, "thrust_vac_n", &engine_config->thrust_vac_n, error)) {
                return false;
            }
        } else if (find_member(engine, "max_thrust_n")) {
            if (!read_number(engine, "max_thrust_n", &engine_config->thrust_vac_n, error)) {
                return false;
            }
        }
        // Isp: prefer new name, fall back to legacy isp_s.
        if (find_member(engine, "isp_vac_s")) {
            if (!read_number(engine, "isp_vac_s", &engine_config->isp_vac_s, error)) {
                return false;
            }
        } else if (find_member(engine, "isp_s")) {
            if (!read_number(engine, "isp_s", &engine_config->isp_vac_s, error)) {
                return false;
            }
        }
        if (!read_number(engine, "thrust_sl_n", &engine_config->thrust_sl_n, error) ||
            !read_number(engine, "isp_sl_s", &engine_config->isp_sl_s, error) ||
            !read_number(engine, "nozzle_exit_area_m2", &engine_config->nozzle_exit_area_m2, error) ||
            !read_number(engine, "min_throttle", &engine_config->min_throttle, error) ||
            !read_number(engine, "max_throttle", &engine_config->max_throttle, error) ||
            !read_number(engine, "ignition_delay_s", &engine_config->ignition_delay_s, error) ||
            !read_number(engine, "spool_up_rate_per_s", &engine_config->spool_up_rate_per_s, error) ||
            !read_number(engine, "spool_down_rate_per_s", &engine_config->spool_down_rate_per_s, error) ||
            !read_number(engine, "thrust_buildup_s", &engine_config->thrust_buildup_s, error) ||
            !read_number(engine, "shutdown_delay_s", &engine_config->shutdown_delay_s, error) ||
            !read_number(engine, "gimbal_max_rad", &engine_config->gimbal_max_rad, error) ||
            !read_number(engine, "gimbal_rate_rad_s", &engine_config->gimbal_rate_rad_s, error) ||
            !read_vec3(engine, "direction_body", &engine_config->direction_body, error)) {
            return false;
        }
        if (const JsonValue* count = find_member(engine, "engine_count")) {
            if (!count->is_number()) {
                return fail(error, "engine.engine_count must be a number");
            }
            engine_config->engine_count =
                std::max(1, static_cast<int>(count->number_value));
        }
        if (const JsonValue* curve = find_member(engine, "throttle_curve")) {
            if (!curve->is_array()) {
                return fail(error, "engine.throttle_curve must be an array");
            }
            engine_config->throttle_curve.clear();
            engine_config->throttle_curve.reserve(curve->array_value.size());
            for (const auto& entry : curve->array_value) {
                if (!entry.is_object()) {
                    return fail(error, "throttle_curve entry must be an object");
                }
                post2::vehicle::EngineThrottleCurvePoint point;
                if (!read_number(entry, "throttle", &point.throttle, error) ||
                    !read_number(entry, "mdot_ratio", &point.mdot_ratio, error)) {
                    return false;
                }
                engine_config->throttle_curve.push_back(point);
            }
            std::sort(
                engine_config->throttle_curve.begin(),
                engine_config->throttle_curve.end(),
                [](const post2::vehicle::EngineThrottleCurvePoint& a,
                   const post2::vehicle::EngineThrottleCurvePoint& b) {
                    return a.throttle < b.throttle;
                });
        }
        if (const JsonValue* feed = find_member(engine, "feed_tanks")) {
            if (!feed->is_array()) {
                return fail(error, "engine.feed_tanks must be an array");
            }
            engine_config->feed_tanks.clear();
            engine_config->feed_tanks.reserve(feed->array_value.size());
            for (const auto& entry : feed->array_value) {
                post2::vehicle::TankRef ref;
                if (!parse_tank_ref(entry, &ref)) {
                    return false;
                }
                engine_config->feed_tanks.push_back(std::move(ref));
            }
        }
        return true;
    };

    auto parse_tanks = [&](const JsonValue& tanks_value, std::vector<post2::vehicle::TankConfig>* tanks) -> bool {
        if (!tanks_value.is_array()) {
            return fail(error, "tanks must be an array");
        }
        tanks->clear();
        for (const auto& tank_value : tanks_value.array_value) {
            if (!tank_value.is_object()) {
                return fail(error, "tank entry must be an object");
            }
            post2::vehicle::TankConfig tank;
            if (!read_string(tank_value, "name", &tank.name, error) ||
                !read_string(tank_value, "propellant", &tank.propellant, error) ||
                !read_number(tank_value, "capacity_kg", &tank.capacity_kg, error) ||
                !read_number(tank_value, "initial_kg", &tank.initial_kg, error)) {
                return false;
            }
            tanks->push_back(std::move(tank));
        }
        return true;
    };

    if (const JsonValue* engine = find_member(value, "engine")) {
        if (!parse_engine(*engine, &parsed.engine)) {
            return false;
        }
    }

    if (const JsonValue* tanks = find_member(value, "tanks")) {
        if (!parse_tanks(*tanks, &parsed.tanks)) {
            return false;
        }
    }

    if (const JsonValue* stages = find_member(value, "stages")) {
        if (!stages->is_array()) {
            return fail(error, "vehicle.stages must be an array");
        }
        parsed.stages.clear();
        for (const auto& stage_value : stages->array_value) {
            if (!stage_value.is_object()) {
                return fail(error, "stage entry must be an object");
            }
            post2::vehicle::StageConfig stage;
            if (!read_string(stage_value, "name", &stage.name, error) ||
                !read_bool(stage_value, "active", &stage.active, error) ||
                !read_bool(stage_value, "attached", &stage.attached, error) ||
                !read_number(stage_value, "dry_mass_kg", &stage.dry_mass_kg, error)) {
                return false;
            }
            if (const JsonValue* engine = find_member(stage_value, "engine")) {
                if (!parse_engine(*engine, &stage.engine)) {
                    return false;
                }
            }
            if (const JsonValue* tanks_value = find_member(stage_value, "tanks")) {
                if (!parse_tanks(*tanks_value, &stage.tanks)) {
                    return false;
                }
            }
            parsed.stages.push_back(std::move(stage));
        }
    }
    if (parsed.stages.empty()) {
        parsed.stages = post2::vehicle::effective_stage_configs(parsed);
    }

    if (const JsonValue* t2t = find_member(value, "tank_to_tank_connections")) {
        if (!t2t->is_array()) {
            return fail(error, "tank_to_tank_connections must be an array");
        }
        parsed.tank_to_tank_connections.clear();
        parsed.tank_to_tank_connections.reserve(t2t->array_value.size());
        for (const auto& entry : t2t->array_value) {
            if (!entry.is_object()) {
                return fail(error, "tank_to_tank_connections entry must be an object");
            }
            post2::vehicle::T2TConnection c;
            if (const JsonValue* src = find_member(entry, "source")) {
                if (!parse_tank_ref(*src, &c.source)) {
                    return false;
                }
            }
            if (const JsonValue* dst = find_member(entry, "dest")) {
                if (!parse_tank_ref(*dst, &c.dest)) {
                    return false;
                }
            }
            if (!read_number(entry, "rate_kgps", &c.rate_kgps, error)) {
                return false;
            }
            if (const JsonValue* start = find_member(entry, "start_time_s")) {
                if (!start->is_number()) {
                    return fail(error, "tank_to_tank_connections.start_time_s must be a number");
                }
                c.start_time_s = start->number_value;
            }
            if (const JsonValue* end = find_member(entry, "end_time_s")) {
                if (!end->is_number()) {
                    return fail(error, "tank_to_tank_connections.end_time_s must be a number");
                }
                c.end_time_s = end->number_value;
            }
            parsed.tank_to_tank_connections.push_back(std::move(c));
        }
    }

    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&parsed);

    if (!post2::vehicle::validate_vehicle_config(parsed, error)) {
        return false;
    }
    *target = std::move(parsed);
    return true;
}

bool parse_throttle(const JsonValue& value, ThrottleModelConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "throttle_model must be an object");
    }
    ThrottleModelConfig parsed = *target;
    if (!read_string(value, "type", &parsed.type, error) ||
        !read_number(value, "c0", &parsed.c0, error) ||
        !read_number(value, "c1", &parsed.c1, error) ||
        !read_number(value, "c2", &parsed.c2, error) ||
        !read_number(value, "target_t2w", &parsed.target_t2w, error) ||
        !read_bool(value, "continuity", &parsed.continuity, error) ||
        !read_segmented_poly(value, "segmented_poly", &parsed.segmented_poly, error)) {
        return false;
    }

    if (const JsonValue* points = find_member(value, "points")) {
        if (!points->is_array()) {
            return fail(error, "throttle_model.points must be an array");
        }
        parsed.points.clear();
        for (const auto& point_value : points->array_value) {
            if (!point_value.is_object()) {
                return fail(error, "throttle point must be an object");
            }
            ThrottlePoint point;
            if (!read_number(point_value, "time_s", &point.time_s, error) ||
                !read_number(point_value, "throttle", &point.throttle, error)) {
                return false;
            }
            parsed.points.push_back(point);
        }
    }

    *target = std::move(parsed);
    return true;
}

bool parse_steering(const JsonValue& value, SteeringModelConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "steering_model must be an object");
    }
    SteeringModelConfig parsed = *target;
    if (!read_string(value, "type", &parsed.type, error) ||
        !read_poly(value, "roll", &parsed.roll_deg, error) ||
        !read_poly(value, "pitch", &parsed.pitch_deg, error) ||
        !read_poly(value, "yaw", &parsed.yaw_deg, error) ||
        !read_poly(value, "azimuth", &parsed.azimuth_deg, error) ||
        !read_poly(value, "elevation", &parsed.elevation_deg, error) ||
        !read_tangent(value, "tangent", &parsed.tangent, error) ||
        !read_upfg(value, "upfg", &parsed.upfg, error) ||
        !read_vec3(value, "fixed_direction_eci", &parsed.fixed_direction_eci, error) ||
        !read_segmented_steering_poly(value, "segmented_poly", &parsed.segmented_poly, error)) {
        return false;
    }

    if (const JsonValue* points = find_member(value, "points")) {
        if (!points->is_array()) {
            return fail(error, "steering_model.points must be an array");
        }
        parsed.points.clear();
        for (const auto& point_value : points->array_value) {
            if (!point_value.is_object()) {
                return fail(error, "quat point must be an object");
            }
            QuaternionPoint point;
            if (!read_number(point_value, "time_s", &point.time_s, error)) {
                return false;
            }
            if (const JsonValue* quat = find_member(point_value, "quat")) {
                if (!parse_quaternion(*quat, &point.quat, error)) {
                    return false;
                }
            }
            parsed.points.push_back(point);
        }
    }

    if (const JsonValue* segments = find_member(value, "segments")) {
        if (!segments->is_array()) {
            return fail(error, "steering_model.segments must be an array");
        }
        parsed.segments.clear();
        for (const auto& segment_value : segments->array_value) {
            if (!segment_value.is_object()) {
                return fail(error, "selectable segment must be an object");
            }
            SelectableSteeringSegment segment;
            if (!read_number(segment_value, "start_time_s", &segment.start_time_s, error)) {
                return false;
            }
            SteeringModelConfig child;
            if (const JsonValue* model = find_member(segment_value, "model")) {
                if (!parse_steering(*model, &child, error)) {
                    return false;
                }
            }
            segment.model = std::make_shared<SteeringModelConfig>(std::move(child));
            parsed.segments.push_back(std::move(segment));
        }
    }

    *target = std::move(parsed);
    return true;
}

bool parse_optimization(const JsonValue& value, OptimizationConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "optimization must be an object");
    }

    OptimizationConfig parsed = *target;
    if (!read_string(value, "mode", &parsed.mode, error) ||
        !read_string(value, "optimizer", &parsed.optimizer, error) ||
        !read_string(value, "qp_solver", &parsed.qp_solver, error) ||
        !read_string(value, "fd_mode", &parsed.fd_mode, error)) {
        return false;
    }
    double max_iterations = static_cast<double>(parsed.max_iterations);
    double max_restoration_iterations = static_cast<double>(parsed.max_restoration_iterations);
    if (!read_number(value, "max_iterations", &max_iterations, error) ||
        !read_number(value, "tolerance", &parsed.tolerance, error) ||
        !read_number(value, "stationarity_tolerance", &parsed.stationarity_tolerance, error) ||
        !read_number(value, "feasibility_tolerance", &parsed.feasibility_tolerance, error) ||
        !read_number(value, "constraint_tolerance", &parsed.constraint_tolerance, error) ||
        !read_number(value, "initial_step_fraction", &parsed.initial_step_fraction, error)) {
        return false;
    }
    if (!read_bool(value, "parallel_fd", &parsed.parallel_fd, error) ||
        !read_number(value, "max_restoration_iterations", &max_restoration_iterations, error)) {
        return false;
    }
    parsed.max_iterations = static_cast<int>(max_iterations);
    parsed.max_restoration_iterations = static_cast<int>(max_restoration_iterations);

    if (const JsonValue* variables = find_member(value, "variables")) {
        if (!variables->is_array()) {
            return fail(error, "optimization.variables must be an array");
        }
        parsed.variables.clear();
        for (const auto& variable_value : variables->array_value) {
            if (!variable_value.is_object()) {
                return fail(error, "optimization variable must be an object");
            }
            OptimizationVariableConfig variable;
            if (!read_string(variable_value, "path", &variable.path, error) ||
                !read_bool(variable_value, "enabled", &variable.enabled, error) ||
                !read_number(variable_value, "min_value", &variable.min_value, error) ||
                !read_number(variable_value, "max_value", &variable.max_value, error)) {
                return false;
            }
            parsed.variables.push_back(std::move(variable));
        }
    }

    if (const JsonValue* targets = find_member(value, "targets")) {
        if (!targets->is_array()) {
            return fail(error, "optimization.targets must be an array");
        }
        parsed.targets.clear();
        for (const auto& target_value : targets->array_value) {
            if (!target_value.is_object()) {
                return fail(error, "optimization target must be an object");
            }
            OptimizationTargetConfig target_config;
            double phase_index = static_cast<double>(target_config.phase_index);
            if (!read_string(target_value, "metric", &target_config.metric, error) ||
                !read_string(target_value, "mode", &target_config.mode, error) ||
                !read_string(target_value, "scope", &target_config.scope, error) ||
                !read_number(target_value, "phase_index", &phase_index, error) ||
                !read_number(target_value, "value", &target_config.value, error) ||
                !read_number(target_value, "min_value", &target_config.min_value, error) ||
                !read_number(target_value, "max_value", &target_config.max_value, error) ||
                !read_number(target_value, "weight", &target_config.weight, error)) {
                return false;
            }
            target_config.phase_index = static_cast<int>(phase_index);
            parsed.targets.push_back(std::move(target_config));
        }
    }

    auto parse_objective_value =
        [&](const JsonValue& objective_value,
            OptimizationObjectiveConfig* objective_config,
            const char* context) -> bool {
            if (!objective_value.is_object()) {
                return fail(error, std::string(context) + " must be an object");
            }
            return read_bool(objective_value, "enabled", &objective_config->enabled, error) &&
                read_string(objective_value, "metric", &objective_config->metric, error) &&
                read_string(objective_value, "direction", &objective_config->direction, error) &&
                read_number(objective_value, "weight", &objective_config->weight, error);
        };

    bool parsed_legacy_objective = false;
    if (const JsonValue* objective = find_member(value, "objective")) {
        if (!parse_objective_value(*objective, &parsed.objective, "optimization.objective")) {
            return false;
        }
        parsed_legacy_objective = true;
    }

    if (const JsonValue* objectives = find_member(value, "objectives")) {
        if (!objectives->is_array()) {
            return fail(error, "optimization.objectives must be an array");
        }
        parsed.objectives.clear();
        for (const auto& objective_value : objectives->array_value) {
            OptimizationObjectiveConfig objective_config;
            if (!parse_objective_value(
                    objective_value,
                    &objective_config,
                    "optimization objective")) {
                return false;
            }
            parsed.objectives.push_back(std::move(objective_config));
        }
        if (!parsed.objectives.empty()) {
            parsed.objective = parsed.objectives.front();
        }
    } else if (parsed_legacy_objective) {
        parsed.objectives.clear();
        parsed.objectives.push_back(parsed.objective);
    }

    if (const JsonValue* envelope = find_member(value, "envelope_search")) {
        if (!envelope->is_object()) {
            return fail(error, "optimization.envelope_search must be an object");
        }
        double sample_count = static_cast<double>(parsed.envelope_search.sample_count);
        double seed = static_cast<double>(parsed.envelope_search.seed);
        if (!read_bool(*envelope, "enabled", &parsed.envelope_search.enabled, error) ||
            !read_number(*envelope, "sample_count", &sample_count, error) ||
            !read_number(*envelope, "seed", &seed, error)) {
            return false;
        }
        parsed.envelope_search.sample_count = static_cast<int>(sample_count);
        parsed.envelope_search.seed = static_cast<int>(seed);
    }

    if (const JsonValue* continuation = find_member(value, "continuation")) {
        if (!continuation->is_object()) {
            return fail(error, "optimization.continuation must be an object");
        }
        double steps = static_cast<double>(parsed.continuation.steps);
        double multistart_count = static_cast<double>(parsed.continuation.multistart_count);
        if (!read_bool(*continuation, "enabled", &parsed.continuation.enabled, error) ||
            !read_string(*continuation, "variable_path", &parsed.continuation.variable_path, error) ||
            !read_string(*continuation, "direction", &parsed.continuation.direction, error) ||
            !read_number(*continuation, "steps", &steps, error) ||
            !read_bool(*continuation, "multistart_enabled", &parsed.continuation.multistart_enabled, error) ||
            !read_number(*continuation, "multistart_count", &multistart_count, error)) {
            return false;
        }
        parsed.continuation.steps = static_cast<int>(steps);
        parsed.continuation.multistart_count = static_cast<int>(multistart_count);
        // "mode" is optional so pre-mode case files still load (default "variable").
        if (find_member(*continuation, "mode") &&
            !read_string(*continuation, "mode", &parsed.continuation.mode, error)) {
            return false;
        }
    }

    *target = std::move(parsed);
    return true;
}

bool parse_phase_action(const JsonValue& value, PhaseAction* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "action must be an object");
    }
    PhaseAction action;
    if (!read_number(value, "time_s", &action.time_s, error) ||
        !read_string(value, "type", &action.type, error) ||
        !read_bool(value, "value", &action.value, error)) {
        return false;
    }
    double stage_index = static_cast<double>(action.stage_index);
    if (!read_number(value, "stage_index", &stage_index, error) ||
        !read_string(value, "stage_name", &action.stage_name, error)) {
        return false;
    }
    action.stage_index = static_cast<int>(stage_index);
    *target = std::move(action);
    return true;
}

bool parse_trigger_condition(const JsonValue& value, TriggerCondition* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "trigger must be an object");
    }
    TriggerCondition parsed = *target;
    if (!read_string(value, "type", &parsed.type, error) ||
        !read_string(value, "comparison", &parsed.comparison, error) ||
        !read_number(value, "value", &parsed.value, error)) {
        return false;
    }
    if (parsed.type != "time" &&
        parsed.type != "altitude_m" &&
        parsed.type != "velocity_mps" &&
        parsed.type != "total_mass_kg" &&
        parsed.type != "propellant_mass_kg" &&
        parsed.type != "apoapsis_altitude_m" &&
        parsed.type != "periapsis_altitude_m" &&
        parsed.type != "orbital_energy" &&
        parsed.type != "sma_m" &&
        parsed.type != "thrust_fraction") {
        return fail(error, "trigger.type unrecognised: " + parsed.type);
    }
    if (parsed.comparison != ">=" && parsed.comparison != "<=") {
        return fail(error, "trigger.comparison must be \">=\" or \"<=\"");
    }
    *target = std::move(parsed);
    return true;
}

bool parse_event(const JsonValue& value, EventConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "event must be an object");
    }
    EventConfig parsed = *target;
    if (!read_string(value, "name", &parsed.name, error) ||
        !read_bool(value, "enabled", &parsed.enabled, error)) {
        return false;
    }
    if (const JsonValue* trigger = find_member(value, "trigger")) {
        if (!parse_trigger_condition(*trigger, &parsed.trigger, error)) {
            return false;
        }
    }
    parsed.actions.clear();
    if (const JsonValue* actions = find_member(value, "actions")) {
        if (!actions->is_array()) {
            return fail(error, "event.actions must be an array");
        }
        for (const auto& action_value : actions->array_value) {
            PhaseAction action;
            if (!parse_phase_action(action_value, &action, error)) {
                return false;
            }
            parsed.actions.push_back(std::move(action));
        }
    }
    *target = std::move(parsed);
    return true;
}

bool parse_phase(const JsonValue& value, PhaseConfig* target, std::string* error)
{
    if (!value.is_object()) {
        return fail(error, "phase must be an object");
    }
    PhaseConfig parsed = *target;
    if (!read_string(value, "name", &parsed.name, error) ||
        !read_bool(value, "optimize_enabled", &parsed.optimize_enabled, error) ||
        !read_bool(value, "inherit_initial_state", &parsed.inherit_initial_state, error) ||
        !read_bool(value, "hold_down_clamp_initial_active", &parsed.hold_down_clamp_initial_active, error) ||
        !read_string(value, "dynamics_dof", &parsed.dynamics_dof, error) ||
        !read_string(value, "integrator", &parsed.integrator, error)) {
        return false;
    }
    double controller_stage_index = static_cast<double>(parsed.controller_stage_index);
    if (!read_number(value, "controller_stage_index", &controller_stage_index, error) ||
        !read_string(value, "controller_stage_name", &parsed.controller_stage_name, error) ||
        !read_bool(value, "controller_detached_stage", &parsed.controller_detached_stage, error)) {
        return false;
    }
    parsed.controller_stage_index = static_cast<int>(controller_stage_index);

    // Termination: prefer an explicit block. Old JSON files that only carry
    // duration_s are migrated by synthesising a {time, >=, duration_s} block.
    if (const JsonValue* termination = find_member(value, "termination")) {
        if (!parse_trigger_condition(*termination, &parsed.termination, error)) {
            return false;
        }
    } else if (const JsonValue* legacy_duration = find_member(value, "duration_s")) {
        if (!legacy_duration->is_number()) {
            return fail(error, "duration_s must be a number");
        }
        parsed.termination = TriggerCondition{"time", ">=", legacy_duration->number_value};
    }

    if (const JsonValue* initial_state = find_member(value, "initial_state_eci")) {
        State state;
        if (!parse_state(*initial_state, &state, error)) {
            return false;
        }
        parsed.initial_state_eci = state;
    }

    if (const JsonValue* tol = find_member(value, "tolerances")) {
        if (!tol->is_object()) {
            return fail(error, "phase.tolerances must be an object");
        }
        if (!read_number(*tol, "rtol", &parsed.tolerances.rtol, error) ||
            !read_number(*tol, "atol_position_m", &parsed.tolerances.atol_position_m, error) ||
            !read_number(*tol, "atol_velocity_mps", &parsed.tolerances.atol_velocity_mps, error) ||
            !read_number(*tol, "atol_tank_mass_kg", &parsed.tolerances.atol_tank_mass_kg, error) ||
            !read_number(*tol, "atol_attitude_rad", &parsed.tolerances.atol_attitude_rad, error) ||
            !read_number(
                *tol,
                "atol_angular_velocity_radps",
                &parsed.tolerances.atol_angular_velocity_radps,
                error)) {
            return false;
        }
    }

    if (const JsonValue* force = find_member(value, "force_models")) {
        if (!force->is_object()) {
            return fail(error, "force_models must be an object");
        }
        if (!read_bool(*force, "gravity", &parsed.force_models.gravity, error) ||
            !read_bool(*force, "thrust", &parsed.force_models.thrust, error) ||
            !read_bool(*force, "normal_force", &parsed.force_models.normal_force, error) ||
            !read_bool(*force, "aerodynamic", &parsed.force_models.aerodynamic, error) ||
            !read_bool(*force, "third_body", &parsed.force_models.third_body, error) ||
            !read_bool(
                *force,
                "solar_radiation_pressure",
                &parsed.force_models.solar_radiation_pressure,
                error) ||
            !read_gravity_model(*force, "gravity_model", &parsed.force_models.gravity_model, error) ||
            !read_atmosphere_model(
                *force,
                "atmosphere_model",
                &parsed.force_models.atmosphere_model,
                error) ||
            !read_aero_model(*force, "aero_model", &parsed.force_models.aero_model, error)) {
            return false;
        }
    }

    if (const JsonValue* throttle = find_member(value, "throttle_model")) {
        if (!parse_throttle(*throttle, &parsed.throttle_model, error)) {
            return false;
        }
    }

    if (const JsonValue* steering = find_member(value, "steering_model")) {
        if (!parse_steering(*steering, &parsed.steering_model, error)) {
            return false;
        }
    }

    if (const JsonValue* actions = find_member(value, "actions")) {
        if (!actions->is_array()) {
            return fail(error, "actions must be an array");
        }
        parsed.actions.clear();
        for (const auto& action_value : actions->array_value) {
            PhaseAction action;
            if (!parse_phase_action(action_value, &action, error)) {
                return false;
            }
            parsed.actions.push_back(std::move(action));
        }
    }

    *target = std::move(parsed);
    return true;
}

} // namespace

CaseConfig case_from_simulation_config(const SimulationConfig& config)
{
    CaseConfig case_config;
    case_config.name = "legacy";
    case_config.vehicle = config.vehicle;
    if (case_config.vehicle.stages.empty()) {
        case_config.vehicle.stages = post2::vehicle::effective_stage_configs(case_config.vehicle);
    }
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&case_config.vehicle);
    case_config.launch_site = config.launch_site;
    case_config.earth_radius_m = config.earth_radius_m;
    case_config.earth_mu_m3s2 = config.earth_mu_m3s2;
    case_config.earth_j2 = config.gravity_model.j2 != kEarthJ2
        ? config.gravity_model.j2
        : config.earth_j2;
    case_config.earth_rotation_rad_per_s = config.earth_rotation_rad_per_s;
    case_config.epoch_utc = config.epoch_utc;
    case_config.earth_rotation_at_epoch_rad = config.earth_rotation_at_epoch_rad;
    case_config.step_s = config.step_s;

    PhaseConfig phase;
    phase.name = "default";
    phase.termination = {"time", ">=", config.duration_s};
    phase.inherit_initial_state = false;
    phase.hold_down_clamp_initial_active =
        config.hold_down_clamp.enabled && config.hold_down_clamp.release_time_s > 0.0;
    phase.force_models.normal_force = config.normal_force.enabled;
    phase.force_models.aerodynamic = config.vehicle.aero.enabled;
    phase.force_models.gravity_model = config.gravity_model;
    phase.force_models.gravity_model.j2 = case_config.earth_j2;
    phase.throttle_model.type = "poly";
    phase.throttle_model.c0 = 1.0;
    phase.steering_model.type = "fixed_eci";
    phase.steering_model.fixed_direction_eci = config.vehicle.engine.direction_body;

    if (config.hold_down_clamp.enabled && config.hold_down_clamp.release_time_s <= 0.0) {
        phase.initial_state_eci = launch_site_inertial_state(config, 0.0);
    }

    if (config.hold_down_clamp.enabled) {
        phase.actions.push_back({config.hold_down_clamp.release_time_s, "set_hold_down_clamp_active", false});
    }

    case_config.phases = {std::move(phase)};
    return case_config;
}

SimulationConfig simulation_config_from_case(const CaseConfig& config)
{
    SimulationConfig simulation;
    simulation.earth_radius_m = config.earth_radius_m;
    simulation.earth_mu_m3s2 = config.earth_mu_m3s2;
    simulation.earth_j2 = config.earth_j2;
    simulation.earth_rotation_rad_per_s = config.earth_rotation_rad_per_s;
    simulation.epoch_utc = config.epoch_utc;
    simulation.earth_rotation_at_epoch_rad = config.earth_rotation_at_epoch_rad;
    simulation.step_s = config.step_s;
    simulation.launch_site = config.launch_site;
    simulation.vehicle = config.vehicle;

    if (!config.phases.empty()) {
        const PhaseConfig& phase = config.phases.front();
        // Non-time terminations don't have a fixed duration; fall back to a
        // generous upper bound here (24 h). The simulation_driver actually
        // honours the trigger event regardless of this value.
        simulation.duration_s = (phase.termination.type == "time")
            ? phase.termination.value
            : 86400.0;
        simulation.hold_down_clamp.enabled = phase.hold_down_clamp_initial_active;
        simulation.normal_force.enabled = phase.force_models.normal_force;
        simulation.gravity_model = phase.force_models.gravity_model;
        if (simulation.gravity_model.j2 == kEarthJ2 &&
            simulation.earth_j2 != kEarthJ2) {
            simulation.gravity_model.j2 = simulation.earth_j2;
        }
        for (const auto& action : phase.actions) {
            if (action.type == "set_hold_down_clamp_active" && !action.value) {
                simulation.hold_down_clamp.enabled = true;
                simulation.hold_down_clamp.release_time_s = action.time_s;
                break;
            }
        }
    }

    return simulation;
}

std::string case_config_to_json(const CaseConfig& config)
{
    return json_to_string(case_to_json_value(config), true);
}

bool case_config_from_json(const std::string& text, CaseConfig* config, std::string* error)
{
    JsonValue root;
    if (!parse_json(text, &root, error)) {
        return false;
    }
    if (!root.is_object()) {
        return fail(error, "case JSON root must be an object");
    }

    CaseConfig parsed;
    if (!read_string(root, "name", &parsed.name, error) ||
        !read_number(root, "earth_radius_m", &parsed.earth_radius_m, error) ||
        !read_number(root, "earth_mu_m3s2", &parsed.earth_mu_m3s2, error) ||
        !read_number(root, "earth_j2", &parsed.earth_j2, error) ||
        !read_number(root, "earth_rotation_rad_per_s", &parsed.earth_rotation_rad_per_s, error) ||
        !read_number(root, "step_s", &parsed.step_s, error)) {
        return false;
    }

    if (const JsonValue* launch_site = find_member(root, "launch_site")) {
        if (!launch_site->is_object()) {
            return fail(error, "launch_site must be an object");
        }
        if (!read_number(*launch_site, "latitude_deg", &parsed.launch_site.latitude_deg, error) ||
            !read_number(*launch_site, "longitude_deg", &parsed.launch_site.longitude_deg, error) ||
            !read_number(*launch_site, "altitude_m", &parsed.launch_site.altitude_m, error)) {
            return false;
        }
    }

    if (const JsonValue* epoch = find_member(root, "epoch_utc")) {
        if (!epoch->is_object()) {
            return fail(error, "epoch_utc must be an object");
        }
        double year_d = parsed.epoch_utc.year;
        double month_d = parsed.epoch_utc.month;
        double day_d = parsed.epoch_utc.day;
        double hour_d = parsed.epoch_utc.hour;
        double minute_d = parsed.epoch_utc.minute;
        if (!read_number(*epoch, "year", &year_d, error) ||
            !read_number(*epoch, "month", &month_d, error) ||
            !read_number(*epoch, "day", &day_d, error) ||
            !read_number(*epoch, "hour", &hour_d, error) ||
            !read_number(*epoch, "minute", &minute_d, error) ||
            !read_number(*epoch, "second", &parsed.epoch_utc.second, error)) {
            return false;
        }
        parsed.epoch_utc.year = static_cast<int>(year_d);
        parsed.epoch_utc.month = static_cast<int>(month_d);
        parsed.epoch_utc.day = static_cast<int>(day_d);
        parsed.epoch_utc.hour = static_cast<int>(hour_d);
        parsed.epoch_utc.minute = static_cast<int>(minute_d);
    }

    if (const JsonValue* vehicle = find_member(root, "vehicle")) {
        if (!parse_vehicle(*vehicle, &parsed.vehicle, error)) {
            return false;
        }
    }

    parsed.phases.clear();
    if (const JsonValue* phases = find_member(root, "phases")) {
        if (!phases->is_array()) {
            return fail(error, "phases must be an array");
        }
        for (const auto& phase_value : phases->array_value) {
            PhaseConfig phase;
            phase.force_models.gravity_model.j2 = parsed.earth_j2;
            if (!parse_phase(phase_value, &phase, error)) {
                return false;
            }
            parsed.phases.push_back(std::move(phase));
        }
    }
    if (parsed.phases.empty()) {
        PhaseConfig phase;
        phase.force_models.gravity_model.j2 = parsed.earth_j2;
        parsed.phases.push_back(std::move(phase));
    }
    if (!find_member(root, "earth_j2") && !parsed.phases.empty()) {
        parsed.earth_j2 = parsed.phases.front().force_models.gravity_model.j2;
    }

    parsed.events.clear();
    if (const JsonValue* events = find_member(root, "events")) {
        if (!events->is_array()) {
            return fail(error, "events must be an array");
        }
        for (const auto& event_value : events->array_value) {
            EventConfig event;
            if (!parse_event(event_value, &event, error)) {
                return false;
            }
            parsed.events.push_back(std::move(event));
        }
    }

    if (const JsonValue* optimization = find_member(root, "optimization")) {
        if (!parse_optimization(*optimization, &parsed.optimization, error)) {
            return false;
        }
    }

    parsed.earth_rotation_at_epoch_rad =
        frames::gmst_rad(frames::julian_date_utc(parsed.epoch_utc));

    *config = std::move(parsed);
    return true;
}

bool load_case_config_file(const std::string& path, CaseConfig* config, std::string* error)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return fail(error, "failed to open case config: " + path);
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return case_config_from_json(buffer.str(), config, error);
}

bool save_case_config_file(const std::string& path, const CaseConfig& config, std::string* error)
{
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        return fail(error, "failed to open case config for writing: " + path);
    }

    output << case_config_to_json(config);
    return true;
}

} // namespace post2::core
