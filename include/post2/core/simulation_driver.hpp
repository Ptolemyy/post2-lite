#pragma once

#include "post2/core/types.hpp"

namespace post2::core {

class GravityPropagator {
public:
    StateLog propagate(const SimulationConfig& config, const StateLog& initial_state_log) const;
};

class LaunchVehicleEvent {
public:
    bool has_actions_before_propagation() const;
    bool has_actions_after_propagation() const;
    void cleanupEvent(StateLog& state_log) const;
    void run_actions_after_propagation(StateLog& state_log) const;
    StateLog executeEvent(const SimulationConfig& config, const StateLog& state_log) const;
};

class SimulationDriver {
public:
    SimulationResult run(const SimulationConfig& config) const;
    SimulationResult run(const CaseConfig& config) const;

private:
    StateLog initialize_or_truncate_state_log(const SimulationConfig& config) const;
    StateLog integrateOneEvent(
        const LaunchVehicleEvent& event,
        const SimulationConfig& config,
        const StateLog& state_log) const;
};

} // namespace post2::core
