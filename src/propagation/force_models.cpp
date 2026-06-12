#include "post2/propagation/force_models.hpp"

#include <stdexcept>

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

post2::core::Vec3 normalized_or(
    const post2::core::Vec3& value,
    const post2::core::Vec3& fallback)
{
    const double length = post2::vehicle::norm(value);
    if (length <= 1.0e-12) {
        return fallback;
    }
    return value / length;
}

double surface_radius_m(const post2::core::SimulationConfig& config)
{
    return config.earth_radius_m + config.launch_site.altitude_m;
}

} // namespace

post2::core::Vec3 gravity_acceleration_mps2(
    const post2::core::SimulationConfig& config,
    const post2::core::Vec3& position_m)
{
    const double radius_m = post2::vehicle::norm(position_m);
    if (radius_m <= 0.0) {
        throw std::runtime_error("vehicle reached the gravity singularity");
    }

    const double factor = -config.earth_mu_m3s2 / (radius_m * radius_m * radius_m);
    return position_m * factor;
}

post2::core::Vec3 surface_normal_acceleration_mps2(
    const post2::core::SimulationConfig& config,
    const post2::core::State& state)
{
    if (!config.normal_force.enabled) {
        return {};
    }

    const double radius_m = post2::vehicle::norm(state.position_m);
    if (radius_m <= 0.0 || radius_m > surface_radius_m(config)) {
        return {};
    }

    const post2::core::Vec3 gravity_mps2 = gravity_acceleration_mps2(config, state.position_m);
    const post2::core::Vec3 omega_radps = {0.0, 0.0, config.earth_rotation_rad_per_s};
    const post2::core::Vec3 centripetal_mps2 =
        cross_product(omega_radps, cross_product(omega_radps, state.position_m));
    return centripetal_mps2 - gravity_mps2;
}

post2::core::State apply_surface_contact_constraint(
    const post2::core::SimulationConfig& config,
    const post2::core::State& state)
{
    if (!config.normal_force.enabled) {
        return state;
    }

    const double radius_m = post2::vehicle::norm(state.position_m);
    const double contact_radius_m = surface_radius_m(config);
    if (radius_m <= 0.0 || radius_m >= contact_radius_m) {
        return state;
    }

    const post2::core::Vec3 normal = normalized_or(state.position_m, {1.0, 0.0, 0.0});
    post2::core::State constrained = state;
    constrained.position_m = normal * contact_radius_m;

    const double radial_velocity_mps = post2::vehicle::dot(constrained.velocity_mps, normal);
    if (radial_velocity_mps < 0.0) {
        constrained.velocity_mps = constrained.velocity_mps - normal * radial_velocity_mps;
    }

    return constrained;
}

} // namespace post2::propagation
