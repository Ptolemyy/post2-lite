#pragma once

#include "post2/core/state_log.hpp"
#include "post2/core/types.hpp"

namespace post2::core {

struct PlanePoint {
    double x_m = 0.0;
    double y_m = 0.0;
};

class OrbitPlaneProjector {
public:
    OrbitPlaneProjector(Vec3 x_axis, Vec3 y_axis);

    PlanePoint project(const Vec3& position_m) const;

private:
    Vec3 x_axis_;
    Vec3 y_axis_;
};

OrbitPlaneProjector make_orbit_plane_projector(const StateLog& state_log);
double max_abs_projected_xy(const StateLog& state_log, const OrbitPlaneProjector& projector);

} // namespace post2::core
