#include "post2/propagation/builtin_events.hpp"

#include "post2/core/frames.hpp"

namespace post2::propagation {

namespace {

double radial_velocity(const post2::integrators::ExtendedState& s)
{
    return post2::vehicle::dot(s.motion.position_m, s.motion.velocity_mps);
}

} // namespace

post2::integrators::EventFunction altitude_zero_event(bool terminating)
{
    post2::integrators::EventFunction ev;
    ev.name = "altitude_zero";
    ev.terminating = terminating;
    ev.direction = -1;
    ev.g = [](double, const post2::integrators::ExtendedState& s) {
        return post2::core::frames::ecef_to_geodetic(s.motion.position_m).altitude_m;
    };
    return ev;
}

post2::integrators::EventFunction altitude_threshold_event(double altitude_m, bool terminating)
{
    post2::integrators::EventFunction ev;
    ev.name = "altitude_threshold";
    ev.terminating = terminating;
    ev.g = [altitude_m](double, const post2::integrators::ExtendedState& s) {
        return post2::core::frames::ecef_to_geodetic(s.motion.position_m).altitude_m - altitude_m;
    };
    return ev;
}

post2::integrators::EventFunction periapsis_event(bool terminating)
{
    post2::integrators::EventFunction ev;
    ev.name = "periapsis";
    ev.terminating = terminating;
    ev.direction = +1;
    ev.g = [](double, const post2::integrators::ExtendedState& s) {
        return radial_velocity(s);
    };
    return ev;
}

post2::integrators::EventFunction apoapsis_event(bool terminating)
{
    post2::integrators::EventFunction ev;
    ev.name = "apoapsis";
    ev.terminating = terminating;
    ev.direction = -1;
    ev.g = [](double, const post2::integrators::ExtendedState& s) {
        return radial_velocity(s);
    };
    return ev;
}

} // namespace post2::propagation
