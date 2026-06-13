#include "post2/propagation/force_model.hpp"

#include "post2/propagation/force_models.hpp"

namespace post2::propagation {

namespace {

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
    (void)context;
    (void)state;
    return zero_output();
}

ForceModelOutput LiftAeroModel::evaluate(
    const ForceModelContext& context,
    const post2::integrators::ExtendedState& state) const
{
    (void)context;
    (void)state;
    return zero_output();
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
