#include "post2/core/state_log.hpp"

#include "post2/core/frames.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace post2::core {

namespace {

post2::vehicle::Vec3 lerp(const post2::vehicle::Vec3& lhs, const post2::vehicle::Vec3& rhs, double alpha)
{
    return lhs + (rhs - lhs) * alpha;
}

post2::vehicle::CartesianState6D interpolate_state(
    const post2::vehicle::CartesianState6D& lhs,
    const post2::vehicle::CartesianState6D& rhs,
    double alpha)
{
    return {
        lerp(lhs.position_m, rhs.position_m, alpha),
        lerp(lhs.velocity_mps, rhs.velocity_mps, alpha),
    };
}

} // namespace

StateLog::StateLog() = default;

StateLog::StateLog(double reference_radius_m)
    : reference_radius_m_(reference_radius_m)
{
}

StateLog::StateLog(double reference_radius_m, post2::vehicle::VehicleConfig vehicle_config)
    : reference_radius_m_(reference_radius_m)
    , vehicle_config_(std::move(vehicle_config))
{
}

double StateLog::reference_radius_m() const
{
    return reference_radius_m_;
}

const post2::vehicle::VehicleConfig& StateLog::vehicle_config() const
{
    return vehicle_config_;
}

bool StateLog::empty() const
{
    return entries_.empty();
}

std::size_t StateLog::size() const
{
    return entries_.size();
}

const LaunchVehicleStateLogEntry& StateLog::front() const
{
    return entries_.front();
}

const LaunchVehicleStateLogEntry& StateLog::back() const
{
    return entries_.back();
}

const std::vector<LaunchVehicleStateLogEntry>& StateLog::entries() const
{
    return entries_;
}

void StateLog::set_phase_metadata(int phase_index, std::string phase_name)
{
    phase_index_ = phase_index;
    phase_name_ = std::move(phase_name);
}

void StateLog::clear()
{
    entries_.clear();
}

void StateLog::append(double time_s, const post2::vehicle::CartesianState6D& state)
{
    entries_.push_back(make_entry(time_s, state));
}

void StateLog::append(const post2::vehicle::VehicleRuntimeState& runtime)
{
    entries_.push_back(make_entry(runtime));
}

void StateLog::append(const LaunchVehicleStateLogEntry& entry)
{
    entries_.push_back(entry);
}

void StateLog::truncate_after(double time_s)
{
    const auto first_after = std::upper_bound(
        entries_.begin(),
        entries_.end(),
        time_s,
        [](double value, const LaunchVehicleStateLogEntry& entry) {
            return value < entry.time_s;
        });

    if (first_after == entries_.end()) {
        return;
    }

    if (first_after == entries_.begin()) {
        entries_.clear();
        return;
    }

    const auto previous = first_after - 1;
    if (previous->time_s < time_s) {
        const double span_s = first_after->time_s - previous->time_s;
        if (span_s <= 0.0) {
            throw std::runtime_error("StateLog contains non-increasing time entries");
        }

        const double alpha = (time_s - previous->time_s) / span_s;
        const auto state = interpolate_state(previous->state, first_after->state, alpha);
        entries_.erase(first_after, entries_.end());
        append(time_s, state);
        return;
    }

    entries_.erase(first_after, entries_.end());
}

LaunchVehicleStateLogEntry StateLog::make_entry(double time_s, const post2::vehicle::CartesianState6D& state) const
{
    return make_entry(post2::vehicle::make_initial_runtime_state(vehicle_config_, state, time_s));
}

LaunchVehicleStateLogEntry StateLog::make_entry(const post2::vehicle::VehicleRuntimeState& runtime) const
{
    const double radius_m = post2::vehicle::norm(runtime.vehicle.motion.position_m);
    // Geodetic altitude (height above WGS84 ellipsoid) is rotation-invariant
    // about the z-axis, so ecef_to_geodetic on the ECI position yields the
    // correct altitude regardless of Earth rotation phase.
    const double geodetic_altitude_m =
        frames::ecef_to_geodetic(runtime.vehicle.motion.position_m).altitude_m;
    LaunchVehicleStateLogEntry entry;
    entry.time_s = runtime.time_s;
    entry.runtime = runtime;
    entry.state = runtime.vehicle.motion;
    entry.radius_m = radius_m;
    entry.altitude_m = geodetic_altitude_m;
    entry.speed_mps = post2::vehicle::norm(runtime.vehicle.motion.velocity_mps);
    entry.total_mass_kg = runtime.vehicle.total_mass_kg;
    entry.propellant_mass_kg = runtime.vehicle.propellant_mass_kg;
    entry.rigid_body_attitude_rad = runtime.vehicle.rigid_body.attitude_rad;
    entry.rigid_body_angular_velocity_radps =
        runtime.vehicle.rigid_body.angular_velocity_radps;
    entry.rigid_body_moment_of_inertia_kgm2 =
        runtime.vehicle.rigid_body.moment_of_inertia_kgm2;
    entry.engine_thrust_n = runtime.engine.actual_thrust_n;
    entry.engine_mass_flow_kgps = runtime.engine.mass_flow_kgps;
    entry.throttle = runtime.engine.throttle;
    entry.engine_direction_eci = runtime.engine.direction_body;
    entry.hold_down_clamp_active = runtime.hold_down_clamp.active;
    entry.phase_index = phase_index_;
    entry.phase_name = phase_name_;
    return entry;
}

LaunchVehicleStateLogEntry StateLog::build_entry(const post2::vehicle::VehicleRuntimeState& runtime) const
{
    return make_entry(runtime);
}

} // namespace post2::core
