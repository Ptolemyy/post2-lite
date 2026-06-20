#include "post2/propagation/force_model.hpp"

#include "post2/aero/aero_table.hpp"
#include "post2/propagation/force_models.hpp"

#include <cmath>
#include <string>

namespace post2::propagation {

namespace {

post2::core::Vec3 cross_product(
    const post2::core::Vec3& lhs,
    const post2::core::Vec3& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

post2::core::SimulationConfig simulation_config_from_context(
    const ForceModelContext& context,
    const char* gravity_model_type)
{
    post2::core::SimulationConfig config;
    if (context.case_config) {
        config.earth_radius_m = context.case_config->earth_radius_m;
        config.earth_mu_m3s2 = context.case_config->earth_mu_m3s2;
        config.earth_j2 = context.case_config->earth_j2;
        config.earth_rotation_rad_per_s = context.case_config->earth_rotation_rad_per_s;
        config.launch_site = context.case_config->launch_site;
    }

    if (context.phase) {
        config.normal_force.enabled = context.phase->force_models.normal_force;
        config.gravity_model = context.phase->force_models.gravity_model;
    }
    if (config.gravity_model.j2 == post2::core::kEarthJ2 &&
        config.earth_j2 != post2::core::kEarthJ2) {
        config.gravity_model.j2 = config.earth_j2;
    }
    config.gravity_model.type = gravity_model_type;
    return config;
}

ForceModelOutput zero_output()
{
    return {};
}

post2::core::Vec3 scale_vec(const post2::core::Vec3& v, double s)
{
    return {v.x * s, v.y * s, v.z * s};
}

double dot_vec(const post2::core::Vec3& a, const post2::core::Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Loads (once, cached by path) an offline-generated aero table. Several tables
// (one per staging configuration) may be live at once, so the cache keeps each
// loaded path. Returns nullptr when the path is empty or fails to load.
const post2::aero::AeroTable* load_cached_aero_table(const std::string& path)
{
    struct CacheEntry {
        std::string path;
        post2::aero::AeroTable table;
        bool ok = false;
    };
    static std::vector<CacheEntry> cache;
    if (path.empty()) {
        return nullptr;
    }
    for (const auto& entry : cache) {
        if (entry.path == path) {
            return entry.ok ? &entry.table : nullptr;
        }
    }
    CacheEntry entry;
    entry.path = path;
    std::string error;
    entry.ok = post2::aero::read_aero_table_csv(path, &entry.table, &error);
    cache.push_back(std::move(entry));
    const CacheEntry& stored = cache.back();
    return stored.ok ? &stored.table : nullptr;
}

// Lowest/highest still-attached stage indices (the currently-attached
// configuration). Returns false when no stage is attached.
bool attached_stage_range(const post2::vehicle::VehicleRuntimeState& runtime,
                          int* lo, int* hi)
{
    int first = -1;
    int last = -1;
    for (std::size_t i = 0; i < runtime.stages.size(); ++i) {
        if (runtime.stages[i].attached) {
            if (first < 0) {
                first = static_cast<int>(i);
            }
            last = static_cast<int>(i);
        }
    }
    if (first < 0) {
        return false;
    }
    *lo = first;
    *hi = last;
    return true;
}

struct ActiveAeroTable {
    std::string path;
    double reference_area_m2 = 0.0;
};

// Picks the aero table matching the vehicle's currently-attached components.
// The attached stages form a contiguous range [lo, hi]:
//   * If the topmost stage is still attached (the normal ascending vehicle),
//     use the open-top upper-stack table for the lowest attached stage (the
//     full stack when lo == 0).
//   * Otherwise the vehicle is a separated stage / sub-stack flying alone -- use
//     the bounded table whose [lo, hi] matches exactly (e.g. a recovered
//     booster).
// Falls back to the single legacy table, then the nearest lower activation
// level, so older configs and partial table sets still resolve to something.
ActiveAeroTable select_active_aero_table(const post2::vehicle::AeroConfig& aero,
                                         const post2::vehicle::VehicleRuntimeState& runtime)
{
    ActiveAeroTable result;
    if (aero.stage_tables.empty()) {
        result.path = aero.aero_table_path;
        result.reference_area_m2 = aero.reference_area_m2;
        return result;
    }

    const int stage_count = static_cast<int>(runtime.stages.size());
    int lo = 0;
    int hi = stage_count > 0 ? stage_count - 1 : 0;
    const bool any_attached = attached_stage_range(runtime, &lo, &hi);
    if (!any_attached) {
        lo = 0;
        hi = stage_count > 0 ? stage_count - 1 : 0;
    }
    const bool top_attached = stage_count == 0 || hi == stage_count - 1;

    const post2::vehicle::AeroStageTable* best = nullptr;

    // 1) Exact bounded match: a separated stage / sub-stack flying on its own.
    for (const auto& entry : aero.stage_tables) {
        if (entry.max_attached_stage >= 0 &&
            entry.activate_at_min_attached_stage == lo &&
            entry.max_attached_stage == hi) {
            best = &entry;
            break;
        }
    }

    // 2) Open-top upper-stack table for the ascending vehicle: the largest
    //    activation level not exceeding the lowest attached stage.
    if (best == nullptr && top_attached) {
        for (const auto& entry : aero.stage_tables) {
            if (entry.max_attached_stage < 0 &&
                entry.activate_at_min_attached_stage <= lo &&
                (best == nullptr ||
                 entry.activate_at_min_attached_stage > best->activate_at_min_attached_stage)) {
                best = &entry;
            }
        }
    }

    // 3) Generic fallback: nearest table at or below the lowest attached stage.
    if (best == nullptr) {
        for (const auto& entry : aero.stage_tables) {
            if (entry.activate_at_min_attached_stage <= lo &&
                (best == nullptr ||
                 entry.activate_at_min_attached_stage > best->activate_at_min_attached_stage)) {
                best = &entry;
            }
        }
    }
    if (best == nullptr) {
        best = &aero.stage_tables.front();  // current config below all entries -> first
    }
    result.path = best->table_path;
    result.reference_area_m2 =
        best->reference_area_m2 > 0.0 ? best->reference_area_m2 : aero.reference_area_m2;
    return result;
}

struct AeroFlow {
    post2::core::Vec3 v_rel_mps{0.0, 0.0, 0.0};
    double speed_mps = 0.0;
    double mach = 0.0;
    double alpha_deg = 0.0;
    post2::core::Vec3 body_axis{0.0, 0.0, 0.0};  // unit thrust/body direction (0 if coasting)
    bool has_body_axis = false;
};

// Atmosphere-relative velocity, Mach, and angle of attack (body axis taken from
// the thrust direction; alpha = 0 while coasting).
AeroFlow compute_aero_flow(const ForceModelContext& context,
                           const post2::integrators::ExtendedState& state)
{
    AeroFlow flow;
    const double omega_radps = context.case_config->earth_rotation_rad_per_s;
    const post2::core::Vec3 atmosphere_velocity_mps =
        cross_product({0.0, 0.0, omega_radps}, state.motion.position_m);
    flow.v_rel_mps =
        state.motion.velocity_mps - atmosphere_velocity_mps - context.environment->wind_eci_mps;
    flow.speed_mps = post2::vehicle::norm(flow.v_rel_mps);
    if (flow.speed_mps <= 1.0e-9) {
        return flow;
    }
    const double a = context.environment->speed_of_sound_mps;
    flow.mach = a > 0.0 ? flow.speed_mps / a : 0.0;

    const double thrust_mag = post2::vehicle::norm(context.thrust_acceleration_eci_mps2);
    if (thrust_mag > 1.0e-9) {
        flow.body_axis = scale_vec(context.thrust_acceleration_eci_mps2, 1.0 / thrust_mag);
        flow.has_body_axis = true;
        const post2::core::Vec3 v_hat = scale_vec(flow.v_rel_mps, 1.0 / flow.speed_mps);
        const double c = std::max(-1.0, std::min(1.0, dot_vec(flow.body_axis, v_hat)));
        flow.alpha_deg = std::acos(c) * (180.0 / 3.14159265358979323846);
    }
    return flow;
}

} // namespace

ForceModelOutput PointMassGravityModel::evaluate(
    const ForceModelContext& context,
    const post2::integrators::ExtendedState& state) const
{
    const post2::core::SimulationConfig config =
        simulation_config_from_context(context, "point_mass");
    return {gravity_acceleration_mps2(config, state.motion.position_m)};
}

ForceModelOutput J2GravityModel::evaluate(
    const ForceModelContext& context,
    const post2::integrators::ExtendedState& state) const
{
    const post2::core::SimulationConfig config =
        simulation_config_from_context(context, "j2");
    return {gravity_acceleration_mps2(config, state.motion.position_m)};
}

ForceModelOutput SphericalHarmonicGravityModel::evaluate(
    const ForceModelContext& context,
    const post2::integrators::ExtendedState& state) const
{
    (void)context;
    (void)state;
    return zero_output();
}

ForceModelOutput AtmosphericDragModel::evaluate(
    const ForceModelContext& context,
    const post2::integrators::ExtendedState& state) const
{
    if (!context.case_config || !context.runtime || !context.environment) {
        return zero_output();
    }

    const post2::vehicle::AeroConfig& aero = context.case_config->vehicle.aero;
    if (!aero.enabled ||
        aero.reference_area_m2 <= 0.0 ||
        context.runtime->vehicle.total_mass_kg <= 0.0 ||
        context.environment->density_kgpm3 <= 0.0) {
        return zero_output();
    }

    const AeroFlow flow = compute_aero_flow(context, state);
    if (flow.speed_mps <= 1.0e-12) {
        return zero_output();
    }

    double cd = aero.cd;
    double reference_area_m2 = aero.reference_area_m2;
    if (aero.use_table) {
        const ActiveAeroTable active = select_active_aero_table(aero, *context.runtime);
        const post2::aero::AeroTable* table = load_cached_aero_table(active.path);
        if (table != nullptr) {
            double cd_table = 0.0;
            double cl_table = 0.0;
            table->lookup(flow.mach, flow.alpha_deg, &cd_table, &cl_table);
            cd = cd_table;
            reference_area_m2 = table->reference_area_m2 > 0.0 ? table->reference_area_m2
                                                              : active.reference_area_m2;
        }
    }
    if (cd <= 0.0) {
        return zero_output();
    }

    const double factor =
        -0.5 * context.environment->density_kgpm3 * cd * reference_area_m2 /
        context.runtime->vehicle.total_mass_kg * flow.speed_mps;
    return {flow.v_rel_mps * factor};
}

ForceModelOutput LiftAeroModel::evaluate(
    const ForceModelContext& context,
    const post2::integrators::ExtendedState& state) const
{
    if (!context.case_config || !context.runtime || !context.environment) {
        return zero_output();
    }
    const post2::vehicle::AeroConfig& aero = context.case_config->vehicle.aero;
    if (!aero.enabled || !aero.use_table ||
        aero.reference_area_m2 <= 0.0 ||
        context.runtime->vehicle.total_mass_kg <= 0.0 ||
        context.environment->density_kgpm3 <= 0.0) {
        return zero_output();
    }

    const ActiveAeroTable active = select_active_aero_table(aero, *context.runtime);
    const post2::aero::AeroTable* table = load_cached_aero_table(active.path);
    if (table == nullptr) {
        return zero_output();
    }

    const AeroFlow flow = compute_aero_flow(context, state);
    if (flow.speed_mps <= 1.0e-12 || !flow.has_body_axis) {
        return zero_output();  // no attitude reference -> assume zero lift
    }

    double cd_table = 0.0;
    double cl_table = 0.0;
    table->lookup(flow.mach, flow.alpha_deg, &cd_table, &cl_table);
    if (std::fabs(cl_table) <= 1.0e-9) {
        return zero_output();
    }

    // Lift acts perpendicular to v_rel, in the (v_rel, body axis) plane, toward
    // the body axis (the side the nose is offset to).
    const post2::core::Vec3 v_hat = scale_vec(flow.v_rel_mps, 1.0 / flow.speed_mps);
    const double axial = dot_vec(flow.body_axis, v_hat);
    const post2::core::Vec3 lift_dir_raw = flow.body_axis - scale_vec(v_hat, axial);
    const double lift_dir_norm = post2::vehicle::norm(lift_dir_raw);
    if (lift_dir_norm <= 1.0e-9) {
        return zero_output();
    }
    const post2::core::Vec3 lift_dir = scale_vec(lift_dir_raw, 1.0 / lift_dir_norm);

    double reference_area_m2 = aero.reference_area_m2;
    if (table->reference_area_m2 > 0.0) {
        reference_area_m2 = table->reference_area_m2;
    }
    const double q = 0.5 * context.environment->density_kgpm3 * flow.speed_mps * flow.speed_mps;
    const double accel = q * cl_table * reference_area_m2 / context.runtime->vehicle.total_mass_kg;
    return {scale_vec(lift_dir, accel)};
}

ForceModelOutput ThrustForceModel::evaluate(
    const ForceModelContext& context,
    const post2::integrators::ExtendedState& state) const
{
    (void)state;
    return {context.thrust_acceleration_eci_mps2};
}

ForceModelOutput SurfaceContactModel::evaluate(
    const ForceModelContext& context,
    const post2::integrators::ExtendedState& state) const
{
    const post2::core::SimulationConfig config =
        simulation_config_from_context(context, context.phase
            ? context.phase->force_models.gravity_model.type.c_str()
            : "j2");
    return {surface_normal_acceleration_mps2(config, state.motion)};
}

ForceModelOutput SolarRadiationPressureModel::evaluate(
    const ForceModelContext& context,
    const post2::integrators::ExtendedState& state) const
{
    (void)context;
    (void)state;
    return zero_output();
}

ForceModelOutput ThirdBodyGravityModel::evaluate(
    const ForceModelContext& context,
    const post2::integrators::ExtendedState& state) const
{
    (void)context;
    (void)state;
    return zero_output();
}

} // namespace post2::propagation
