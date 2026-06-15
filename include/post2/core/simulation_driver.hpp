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

// Propagates the final state of `source_log` forward by ~one orbital period under
// the case's gravity model with thrust off, returning the numerically integrated
// predicted orbit (NOT an analytic Kepler ellipse — it carries J2 etc.). Returns
// an empty log when the final state is sub-orbital / hyperbolic. Intended for
// visualization: the apoapsis/periapsis are the path's max/min-altitude points.
StateLog predict_orbit_path(
    const CaseConfig& case_config,
    const StateLog& source_log,
    int sample_count = 480);

} // namespace post2::core
