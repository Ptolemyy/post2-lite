#pragma once

#include "post2/vehicle/vehicle.hpp"

namespace post2::propagation {

struct EnginePerformanceInputs {
    double throttle_command = 0.0;     // commanded throttle in [0, 1]
    double ambient_pressure_pa = 0.0;  // local atmospheric pressure
};

struct EnginePerformanceOutputs {
    // Cluster aggregate values (multiplied by engine_count).
    double thrust_n = 0.0;
    double mdot_kgps = 0.0;
    double isp_s = 0.0;
    // The throttle actually applied after bounds + curve lookup. Useful for
    // bookkeeping when min_throttle clips a sub-min command to zero.
    double effective_throttle = 0.0;
};

// Pure function: derives instantaneous thrust / mdot / Isp from EngineConfig
// and current inputs, without consulting tank state or simulation time.
// Tank availability gating happens in vehicle_consumption.
//
// Physics:
//   - mdot is determined by the nozzle (chamber pressure, throat area), modeled
//     here as proportional to throttle via throttle_curve. At commanded
//     throttle = 1 and curve empty, mdot equals the vacuum design point.
//   - thrust at altitude is F(p) = F_vac - p_amb * Ae for each engine
//     (Ae = nozzle_exit_area_m2). When Ae = 0 we skip the correction and the
//     engine behaves as a constant-thrust model.
//   - Cluster outputs scale linearly with engine_count.
EnginePerformanceOutputs evaluate_engine(
    const post2::vehicle::EngineConfig& engine,
    const EnginePerformanceInputs& inputs);

} // namespace post2::propagation
