#pragma once

#include "post2/integrators/integrator.hpp"

namespace post2::integrators {

// Adaptive Dormand-Prince RK5(4) with FSAL and Hermite cubic dense output
// for event root finding. Implements the IIntegrator interface.
class Dopri5Integrator final : public IIntegrator {
public:
    explicit Dopri5Integrator(IntegratorTolerances tolerances);

    StepResult step(
        const ExtendedState& state,
        double t_s,
        double h_suggested,
        const DynamicsFunction& dynamics,
        const std::vector<EventFunction>& events) override;

private:
    IntegratorTolerances tolerances_;
};

// Lightweight adapter that gives the existing fixed-step RK4 the
// IIntegrator interface (events detected by end-of-step linear interpolation).
class Rk4IntegratorAdapter final : public IIntegrator {
public:
    Rk4IntegratorAdapter() = default;

    StepResult step(
        const ExtendedState& state,
        double t_s,
        double h_suggested,
        const DynamicsFunction& dynamics,
        const std::vector<EventFunction>& events) override;
};

} // namespace post2::integrators
