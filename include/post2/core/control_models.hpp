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

    // Number of engines to light on the active cluster this step, or -1 for
    // "all of them" (the default that every model except the re-entry burn
    // returns). The driver copies it into EngineCommand::ignited_engine_count.
    virtual int ignited_engine_count(
        double /*time_s*/,
        const post2::vehicle::VehicleRuntimeState& /*runtime*/,
        const PhaseContext& /*context*/) const
    {
        return -1;
    }
};

// Maps a commanded total thrust to a discrete (engine count, per-engine
// throttle) pair: picks the count from `options` whose achievable thrust band
// [count*min_throttle*per_engine_thrust, count*per_engine_thrust] best realises
// `target_thrust_n` (fewest engines on a tie), and the throttle within it. When
// `options` is empty no discrete restriction applies and {0, 0} is returned
// (caller falls back to the full cluster). Per-engine throttle is clamped to
// [min_throttle, 1].
void select_ignited_engines(
    double target_thrust_n,
    double per_engine_thrust_n,
    const std::vector<int>& options,
    double min_throttle,
    int* count_out,
    double* throttle_out);

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

// UPFG time-to-go [s] to the steering config's target orbit at the given
// inertial state, using the same stage stack / target setup as the "upfg"
// steering model (and as post2_player). A non-positive result means the target
// orbit has been reached (insertion / MECO). Returns +infinity when UPFG cannot
// be set up or does not converge, so a tgo-based phase cutoff never fires
// spuriously. This is the algorithm's own termination signal, matching the
// player's `tgo <= margin` cutoff.
double upfg_time_to_go_s(
    const CaseConfig& case_config,
    const SteeringModelConfig& steering,
    const post2::vehicle::VehicleRuntimeState& runtime,
    const Vec3& position_m,
    const Vec3& velocity_mps);

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
