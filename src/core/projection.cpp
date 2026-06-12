#include "post2/core/projection.hpp"

#include <algorithm>
#include <cmath>

namespace post2::core {

namespace {

Vec3 cross(const Vec3& lhs, const Vec3& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

Vec3 normalized_or(const Vec3& value, const Vec3& fallback)
{
    const double length = norm(value);
    if (length <= 1.0e-12) {
        return fallback;
    }
    return value / length;
}

} // namespace

OrbitPlaneProjector::OrbitPlaneProjector(Vec3 x_axis, Vec3 y_axis)
    : x_axis_(x_axis)
    , y_axis_(y_axis)
{
}

PlanePoint OrbitPlaneProjector::project(const Vec3& position_m) const
{
    return {
        dot(position_m, x_axis_),
        dot(position_m, y_axis_),
    };
}

OrbitPlaneProjector make_orbit_plane_projector(const StateLog& state_log)
{
    if (state_log.empty()) {
        return {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    }

    const auto& entries = state_log.entries();
    const State& initial = state_log.front().state;
    const Vec3 x_axis = normalized_or(initial.position_m, {1.0, 0.0, 0.0});
    Vec3 angular_momentum = cross(initial.position_m, initial.velocity_mps);

    if (norm(angular_momentum) <= 1.0e-12 && entries.size() > 1) {
        angular_momentum = cross(initial.position_m, entries[1].state.position_m);
    }

    const Vec3 h_axis = normalized_or(angular_momentum, {0.0, 0.0, 1.0});
    const Vec3 y_axis = normalized_or(cross(h_axis, x_axis), {0.0, 1.0, 0.0});
    return {x_axis, y_axis};
}

double max_abs_projected_xy(const StateLog& state_log, const OrbitPlaneProjector& projector)
{
    double max_value = kEarthRadiusM;
    for (const auto& point : state_log.entries()) {
        const PlanePoint projected = projector.project(point.state.position_m);
        max_value = std::max(max_value, std::abs(projected.x_m));
        max_value = std::max(max_value, std::abs(projected.y_m));
    }
    return max_value * 1.08;
}

} // namespace post2::core
