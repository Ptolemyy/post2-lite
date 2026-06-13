#include "post2/core/coordinates.hpp"

#include "post2/core/frames.hpp"

#include <cmath>

namespace post2::core {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

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
    // Launch site latitude is interpreted as WGS84 geodetic latitude.
    // altitude_m is the ellipsoid altitude.
    return frames::geodetic_to_ecef({
        degrees_to_radians(config.launch_site.latitude_deg),
        degrees_to_radians(config.launch_site.longitude_deg),
        config.launch_site.altitude_m,
    });
}

Vec3 planet_fixed_to_inertial(
    const Vec3& planet_fixed_m,
    double time_s,
    double rotation_rad_per_s)
{
    return frames::ecef_to_eci_position(planet_fixed_m, rotation_rad_per_s * time_s);
}

Vec3 inertial_to_planet_fixed(
    const Vec3& inertial_m,
    double time_s,
    double rotation_rad_per_s)
{
    return frames::eci_to_ecef_position(inertial_m, rotation_rad_per_s * time_s);
}

Vec3 planet_fixed_ground_velocity_inertial_mps(
    const Vec3& planet_fixed_m,
    double time_s,
    double rotation_rad_per_s)
{
    const Vec3 inertial_position_m =
        frames::ecef_to_eci_position(planet_fixed_m, rotation_rad_per_s * time_s);
    return {-rotation_rad_per_s * inertial_position_m.y,
             rotation_rad_per_s * inertial_position_m.x,
             0.0};
}

State launch_site_inertial_state(const SimulationConfig& config, double time_s)
{
    const Vec3 planet_fixed_m = launch_site_planet_fixed_position_m(config);
    const double theta_rad = frames::earth_rotation_angle_rad(
        config.earth_rotation_at_epoch_rad,
        config.earth_rotation_rad_per_s,
        time_s);
    const frames::EciState eci = frames::ecef_to_eci_state(
        planet_fixed_m,
        {0.0, 0.0, 0.0},
        theta_rad,
        config.earth_rotation_rad_per_s);
    return {eci.position_m, eci.velocity_mps};
}

} // namespace post2::core
