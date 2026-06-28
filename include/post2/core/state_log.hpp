#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle.hpp"

namespace post2::core {

struct LaunchVehicleStateLogEntry {
    double time_s = 0.0;
    post2::vehicle::VehicleRuntimeState runtime;
    post2::vehicle::CartesianState6D state;
    double radius_m = 0.0;
    double altitude_m = 0.0;
    double speed_mps = 0.0;
    double total_mass_kg = 0.0;
    double propellant_mass_kg = 0.0;
    double rigid_body_attitude_rad = 0.0;
    double rigid_body_angular_velocity_radps = 0.0;
    double rigid_body_moment_of_inertia_kgm2 = 0.0;
    double engine_thrust_n = 0.0;
    double engine_mass_flow_kgps = 0.0;
    post2::vehicle::Vec3 acceleration_eci_mps2 = {0.0, 0.0, 0.0};
    double acceleration_mps2 = 0.0;
    // Engine command snapshot at this step. throttle and direction come from
    // the steering / throttle models that fed into the integrator step; env
    // fields are populated by simulation_driver from the EnvironmentState
    // sampled at start-of-step.
    double throttle = 0.0;
    post2::vehicle::Vec3 engine_direction_eci = {1.0, 0.0, 0.0};
    double ambient_pressure_pa = 0.0;
    double atmosphere_density_kgpm3 = 0.0;
    double dynamic_pressure_pa = 0.0;
    double mach_number = 0.0;
    // Sutton-Graves stagnation-point convective heat flux [W/m^2].
    double heat_flux_wpm2 = 0.0;
    bool hold_down_clamp_active = false;
    int phase_index = 0;
    std::string phase_name;
};

class StateLog {
public:
    StateLog();
    explicit StateLog(double reference_radius_m);
    StateLog(double reference_radius_m, post2::vehicle::VehicleConfig vehicle_config);

    double reference_radius_m() const;
    const post2::vehicle::VehicleConfig& vehicle_config() const;
    bool empty() const;
    std::size_t size() const;

    const LaunchVehicleStateLogEntry& front() const;
    const LaunchVehicleStateLogEntry& back() const;
    const std::vector<LaunchVehicleStateLogEntry>& entries() const;

    void set_phase_metadata(int phase_index, std::string phase_name);
    void clear();
    void append(double time_s, const post2::vehicle::CartesianState6D& state);
    void append(const post2::vehicle::VehicleRuntimeState& runtime);
    void append(const LaunchVehicleStateLogEntry& entry);
    void truncate_after(double time_s);

    // Exposed so simulation_driver can synthesize an entry from a runtime and
    // then patch in environment-derived fields (q, Mach, pressure) before
    // appending the entry directly.
    LaunchVehicleStateLogEntry build_entry(const post2::vehicle::VehicleRuntimeState& runtime) const;

private:
    LaunchVehicleStateLogEntry make_entry(double time_s, const post2::vehicle::CartesianState6D& state) const;
    LaunchVehicleStateLogEntry make_entry(const post2::vehicle::VehicleRuntimeState& runtime) const;

    double reference_radius_m_ = 0.0;
    post2::vehicle::VehicleConfig vehicle_config_;
    int phase_index_ = 0;
    std::string phase_name_;
    std::vector<LaunchVehicleStateLogEntry> entries_;
};

} // namespace post2::core
