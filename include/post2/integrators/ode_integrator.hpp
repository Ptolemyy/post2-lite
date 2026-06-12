#pragma once

#include <functional>
#include <vector>

#include "post2/core/state_log.hpp"
#include "post2/vehicle/vehicle.hpp"

namespace post2::integrators {

// Extended ODE state. The motion sub-struct keeps the original 6-DOF
// representation so callers can read `.motion.position_m` etc. without
// changes. `tank_masses_kg` carries propellant mass for every tank in
// stable (stage, tank) order - that ordering matches
// post2::vehicle::flat_tank_index. The vector's size stays constant within
// a phase (detached stages keep their slot with dot = 0).
struct ExtendedState {
    post2::vehicle::CartesianState6D motion;
    std::vector<double> tank_masses_kg;
};

struct ExtendedDerivative {
    post2::vehicle::CartesianStateDerivative6D motion_dot;
    std::vector<double> tank_mass_dots_kgps;
};

using DynamicsFunction = std::function<
    ExtendedDerivative(double time_s, const ExtendedState& state)>;
using RuntimeUpdateFunction = std::function<
    post2::vehicle::VehicleRuntimeState(
        const post2::vehicle::VehicleRuntimeState& previous_runtime,
        const ExtendedState& integrated,
        double next_time_s,
        double step_s)>;

struct OdeIntegratorOptions {
    double step_s = 10.0;
};

struct MaxTimeTerminationCondition {
    double max_time_s = 0.0;
};

class Rk4OdeIntegrator {
public:
    explicit Rk4OdeIntegrator(OdeIntegratorOptions options);

    ExtendedState step(
        const ExtendedState& state,
        double time_s,
        double step_s,
        const DynamicsFunction& dynamics) const;

    post2::core::StateLog integrate(
        const post2::core::StateLog& initial_state_log,
        const MaxTimeTerminationCondition& termination,
        const DynamicsFunction& dynamics,
        const RuntimeUpdateFunction& runtime_update) const;

private:
    OdeIntegratorOptions options_;
};

} // namespace post2::integrators
