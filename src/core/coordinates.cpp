#include "post2/core/coordinates.hpp"

#include <cmath>

namespace post2::core {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

Vec3 cross_product(const Vec3& lhs, const Vec3& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

} // namespace

double degrees_to_radians(double degrees)
{
    return degrees * kPi / 180.0;
}

Vec3 rotate_z(const Vec3& value, double angle_rad)
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return {
        value.x * c - value.y * s,
        value.x * s + value.y * c,
        value.z,
    };
}

Vec3 launch_site_planet_fixed_position_m(const SimulationConfig& config)
{
    const double latitude_rad = degrees_to_radians(config.launch_site.latitude_deg);
    const double longitude_rad = degrees_to_radians(config.launch_site.longitude_deg);
    const double radius_m = config.earth_radius_m + config.launch_site.altitude_m;
    const double cos_latitude = std::cos(latitude_rad);

    return {
        radius_m * cos_latitude * std::cos(longitude_rad),
        radius_m * cos_latitude * std::sin(longitude_rad),
        radius_m * std::sin(latitude_rad),
    };
}

Vec3 planet_fixed_to_inertial(
    const Vec3& planet_fixed_m,
    double time_s,
    double rotation_rad_per_s)
{
    return rotate_z(planet_fixed_m, rotation_rad_per_s * time_s);
}

Vec3 inertial_to_planet_fixed(
    const Vec3& inertial_m,
    double time_s,
    double rotation_rad_per_s)
{
    return rotate_z(inertial_m, -rotation_rad_per_s * time_s);
}

Vec3 planet_fixed_ground_velocity_inertial_mps(
    const Vec3& planet_fixed_m,
    double time_s,
    double rotation_rad_per_s)
{
    const Vec3 inertial_position_m = planet_fixed_to_inertial(planet_fixed_m, time_s, rotation_rad_per_s);
    return cross_product({0.0, 0.0, rotation_rad_per_s}, inertial_position_m);
}

State launch_site_inertial_state(const SimulationConfig& config, double time_s)
{
    const Vec3 planet_fixed_m = launch_site_planet_fixed_position_m(config);
    return {
        planet_fixed_to_inertial(planet_fixed_m, time_s, config.earth_rotation_rad_per_s),
        planet_fixed_ground_velocity_inertial_mps(planet_fixed_m, time_s, config.earth_rotation_rad_per_s),
    };
}

} // namespace post2::core
