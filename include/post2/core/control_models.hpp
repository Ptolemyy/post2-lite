#pragma once

#include <memory>

#include "post2/core/types.hpp"
#include "post2/vehicle/runtime_state.hpp"

namespace post2::core {

struct PhaseContext {
    const CaseConfig* case_config = nullptr;
    const PhaseConfig* phase_config = nullptr;
    std::size_t phase_index = 0;
    double phase_start_time_s = 0.0;
};

class IThrottleModel {
public:
    virtual ~IThrottleModel() = default;
    virtual double throttle(
        double time_s,
        const post2::vehicle::VehicleRuntimeState& runtime,
        const PhaseContext& context) const = 0;
};

class ISteeringModel {
public:
    virtual ~ISteeringModel() = default;
    virtual Vec3 thrust_direction_eci(
        double time_s,
        const State& state,
        const post2::vehicle::VehicleRuntimeState& runtime,
        const PhaseContext& context) const = 0;
};

double clamp_throttle(double value);
std::unique_ptr<IThrottleModel> make_throttle_model(const ThrottleModelConfig& config);
std::unique_ptr<ISteeringModel> make_steering_model(const SteeringModelConfig& config);

// Re-anchors a phase's throttle/steering models at a phase boundary so that any
// model or angle whose `continuity` flag is set continues smoothly from the
// previous phase's final state. `start_runtime` is the new phase's initial
// runtime (used for the local frame and, for T/W throttle, mass + max thrust);
// `boundary_throttle` and `boundary_direction_eci` are the previous phase's
// final commanded throttle and ECI thrust direction. No-op for models with no
// continuity flags enabled, so it is safe to call on every transition.
void apply_phase_start_continuity(
    PhaseConfig* phase,
    const CaseConfig& case_config,
    const post2::vehicle::VehicleRuntimeState& start_runtime,
    double boundary_throttle,
    const Vec3& boundary_direction_eci);

} // namespace post2::core
