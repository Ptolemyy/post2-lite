#pragma once

#include "post2/core/types.hpp"

namespace post2::core {

double degrees_to_radians(double degrees);

Vec3 rotate_z(const Vec3& value, double angle_rad);

Vec3 launch_site_planet_fixed_position_m(const SimulationConfig& config);

Vec3 planet_fixed_to_inertial(
    const Vec3& planet_fixed_m,
    double time_s,
    double rotation_rad_per_s);

Vec3 inertial_to_planet_fixed(
    const Vec3& inertial_m,
    double time_s,
    double rotation_rad_per_s);

Vec3 planet_fixed_ground_velocity_inertial_mps(
    const Vec3& planet_fixed_m,
    double time_s,
    double rotation_rad_per_s);

State launch_site_inertial_state(const SimulationConfig& config, double time_s);

} // namespace post2::core
