#include "post2/propagation/engine_performance.hpp"

#include <algorithm>

namespace post2::propagation {

namespace {

constexpr double kStandardGravityMps2 = 9.80665;

double clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

// Interpolate mdot_ratio at the given throttle on a curve table that is
// already sorted ascending by .throttle. Clamps to endpoints outside the
// table. Caller guarantees the table is non-empty when this is invoked.
double interpolate_mdot_ratio(
    const std::vector<post2::vehicle::EngineThrottleCurvePoint>& curve,
    double throttle)
{
    if (throttle <= curve.front().throttle) {
        return curve.front().mdot_ratio;
    }
    if (throttle >= curve.back().throttle) {
        return curve.back().mdot_ratio;
    }
    for (std::size_t i = 1; i < curve.size(); ++i) {
        const auto& a = curve[i - 1];
        const auto& b = curve[i];
        if (throttle <= b.throttle) {
            const double span = b.throttle - a.throttle;
            if (span <= 0.0) {
                return a.mdot_ratio;
            }
            const double alpha = (throttle - a.throttle) / span;
            return a.mdot_ratio + alpha * (b.mdot_ratio - a.mdot_ratio);
        }
    }
    return curve.back().mdot_ratio;
}

} // namespace

EnginePerformanceOutputs evaluate_engine(
    const post2::vehicle::EngineConfig& engine,
    const EnginePerformanceInputs& inputs)
{
    EnginePerformanceOutputs out;

    const int count = std::max(0, engine.engine_count);
    if (!engine.enabled ||
        count <= 0 ||
        engine.thrust_vac_n <= 0.0 ||
        engine.isp_vac_s <= 0.0) {
        return out;
    }

    // 1) Clip commanded throttle to bounds. A command below min_throttle
    //    cuts the engine to zero (sub-min combustion is unstable). At-or-above
    //    min the value is rounded into [min, max].
    const double command = clamp(inputs.throttle_command, 0.0, 1.0);
    const double min_throttle = clamp(engine.min_throttle, 0.0, 1.0);
    const double max_throttle = clamp(engine.max_throttle, min_throttle, 1.0);
    if (command < min_throttle) {
        return out;
    }
    const double throttle_effective = clamp(command, min_throttle, max_throttle);

    // 2) Apply throttle curve. Empty curve -> linear (mdot_ratio = throttle).
    const double mdot_ratio = engine.throttle_curve.empty()
        ? throttle_effective
        : clamp(interpolate_mdot_ratio(engine.throttle_curve, throttle_effective), 0.0, 1.0);
    if (mdot_ratio <= 0.0) {
        out.effective_throttle = throttle_effective;
        return out;
    }

    // 3) Mass flow per engine is the (vacuum-defined) nozzle-throat-limited
    //    quantity scaled by the throttle curve. Thrust at altitude follows
    //    from F_vac and the pressure-area correction.
    const double mdot_vac_single = engine.thrust_vac_n / (engine.isp_vac_s * kStandardGravityMps2);
    const double mdot_single = mdot_vac_single * mdot_ratio;
    const double thrust_vac_single = engine.thrust_vac_n * mdot_ratio;
    const double pressure_correction =
        std::max(0.0, inputs.ambient_pressure_pa) * std::max(0.0, engine.nozzle_exit_area_m2);
    const double thrust_single = std::max(0.0, thrust_vac_single - pressure_correction);

    out.thrust_n = thrust_single * count;
    out.mdot_kgps = mdot_single * count;
    out.isp_s = out.mdot_kgps > 0.0
        ? out.thrust_n / (out.mdot_kgps * kStandardGravityMps2)
        : 0.0;
    out.effective_throttle = throttle_effective;
    return out;
}

} // namespace post2::propagation
