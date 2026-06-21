#pragma once

#include <vector>

#include "post2/integrators/ode_integrator.hpp"
#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle.hpp"

namespace post2::propagation {

struct EngineTimeAction {
    double start_time_s = 0.0;
    double end_time_s = 0.0;
    double throttle = 1.0;
};

struct EngineCommand {
    bool enabled = false;
    double throttle = 0.0;
    post2::vehicle::Vec3 direction_eci = {1.0, 0.0, 0.0};
    // Ambient atmospheric pressure at the vehicle position. Drives the
    // pressure-correction term F(p) = F_vac - p * Ae inside the engine
    // performance model. Zero means vacuum (no correction).
    double ambient_pressure_pa = 0.0;
    // Single stage controlled by a detached-stage controller. Negative means
    // no controller filter, which is used by the main-stack controller and
    // preserves legacy all-attached-stage behavior.
    int controlled_stage_index = -1;
};

class EngineActionSchedule {
public:
    void add_action(const EngineTimeAction& action);
    double throttle_at(double time_s, bool default_engine_enabled) const;

private:
    std::vector<EngineTimeAction> actions_;
};

// Output of compute_derivatives: instantaneous accelerations and tank mass
// rates. The engine bookkeeping (per_stage_engine) records the instantaneous
// thrust / flow / firing state at the time of evaluation; commit() copies
// these into the runtime engine state.
struct DerivativeResult {
    post2::vehicle::Vec3 thrust_acceleration_mps2;
    std::vector<double> tank_mass_dots_kgps;     // size == total_tank_count(runtime)
    std::vector<post2::vehicle::EngineState> per_stage_engine;
    double total_actual_thrust_n = 0.0;
    double total_commanded_thrust_n = 0.0;
    double total_mass_flow_kgps = 0.0;
    double weighted_isp_s = 0.0;
    double throttle = 0.0;
    bool engine_enabled = false;
    post2::vehicle::Vec3 engine_direction_eci = {1.0, 0.0, 0.0};
};

class VehicleConsumptionPropagator {
public:
    explicit VehicleConsumptionPropagator(
        post2::vehicle::VehicleConfig config,
        EngineActionSchedule schedule = {});

    post2::vehicle::VehicleRuntimeState make_initial_state(
        const post2::vehicle::CartesianState6D& motion,
        double time_s) const;

    // Continuous derivative evaluation. `runtime_topology` provides
    // stage/engine topology (active, attached, feed_tanks, configs) - its
    // tank masses are NOT read; the live integrator state in tank_masses_kg
    // is the truth. `motion` is also taken from the integrator state.
    DerivativeResult compute_derivatives(
        double time_s,
        const post2::vehicle::VehicleRuntimeState& runtime_topology,
        const std::vector<double>& tank_masses_kg,
        const post2::vehicle::CartesianState6D& motion,
        const EngineCommand& command) const;

    // Write the integrator's integrated state back into a runtime: copies
    // motion + tank masses, runs `refresh_vehicle_masses`, and updates per-
    // stage engine bookkeeping from `last_eval` (an instantaneous snapshot,
    // typically taken at the END of the step for a representative value).
    post2::vehicle::VehicleRuntimeState commit(
        const post2::vehicle::VehicleRuntimeState& previous,
        const post2::integrators::ExtendedState& integrated,
        double next_time_s,
        const DerivativeResult& last_eval,
        const EngineCommand& command) const;

    double throttle_at(double time_s) const;

    const post2::vehicle::VehicleConfig& config() const { return config_; }

private:
    post2::vehicle::VehicleConfig config_;
    EngineActionSchedule schedule_;
};

} // namespace post2::propagation
