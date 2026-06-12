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
    double engine_thrust_n = 0.0;
    double engine_mass_flow_kgps = 0.0;
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
