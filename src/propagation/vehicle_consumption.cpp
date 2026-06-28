#include "post2/propagation/vehicle_consumption.hpp"

#include "post2/propagation/engine_performance.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

namespace post2::propagation {

namespace {

constexpr double kStandardGravityMps2 = 9.80665;
// Tank-empty / tank-full hard threshold. Below this mass the integrator's
// event detection is expected to have already truncated the step and the
// simulation_driver to have clamped the tank to 0. We still apply a hard
// cutoff in the derivative as a safety net in case events are disabled or
// the integrator missed the crossing.
constexpr double kEpsKg = 1.0e-3;

double clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

post2::vehicle::Vec3 normalized_or(const post2::vehicle::Vec3& value, const post2::vehicle::Vec3& fallback)
{
    const double length = post2::vehicle::norm(value);
    if (length <= 1.0e-12) {
        return fallback;
    }
    return value / length;
}

double tank_capacity_kg(
    const post2::vehicle::VehicleConfig& config,
    const std::string& stage_name,
    const std::string& tank_name)
{
    const std::vector<post2::vehicle::StageConfig> stages =
        post2::vehicle::effective_stage_configs(config);
    for (const auto& stage : stages) {
        if (stage.name != stage_name) {
            continue;
        }
        for (const auto& tank : stage.tanks) {
            if (tank.name == tank_name) {
                return std::max(0.0, tank.capacity_kg);
            }
        }
    }
    return 0.0;
}

double total_attached_mass_kg(
    const post2::vehicle::VehicleRuntimeState& topology,
    const std::vector<double>& tank_masses_kg)
{
    double total = 0.0;
    for (std::size_t i = 0; i < topology.stages.size(); ++i) {
        if (!topology.stages[i].attached) {
            continue;
        }
        total += std::max(0.0, topology.stages[i].dry_mass_kg);
        for (std::size_t j = 0; j < topology.stages[i].tanks.size(); ++j) {
            const std::size_t flat = post2::vehicle::flat_tank_index(topology, i, j);
            if (flat < tank_masses_kg.size()) {
                total += std::max(0.0, tank_masses_kg[flat]);
            }
        }
    }
    if (total <= 0.0) {
        return topology.vehicle.total_mass_kg;
    }
    return total;
}

// Actual (spooled) throttle of an engine at eval_time_s, given the actual
// throttle spool_at_ref sampled at ref_time_s and relaxing toward
// command_throttle. Implements the first-order spool lag with an ignition
// dead-time documented on EngineConfig. Closed-form (exact for a command held
// constant over [ref_time_s, eval_time_s]), so it is correct for an arbitrary
// step size and stable. Returns 0 before ignition / during the dead-time, and
// the raw command when no spool rate is configured (legacy instantaneous).
double spooled_throttle(
    const post2::vehicle::EngineConfig& engine,
    double ignition_time_s,
    double spool_at_ref,
    double ref_time_s,
    double eval_time_s,
    double command_throttle)
{
    if (ignition_time_s < 0.0) {
        return 0.0;
    }
    const double up = engine.spool_up_rate_per_s;
    const double down = engine.spool_down_rate_per_s;
    if (up <= 0.0 && down <= 0.0) {
        return std::max(0.0, command_throttle);  // no transient configured
    }
    const double active_start_s = ignition_time_s + std::max(0.0, engine.ignition_delay_s);
    double theta = spool_at_ref;
    double t0 = ref_time_s;
    if (t0 < active_start_s) {
        // Pinned at zero through the ignition dead-time.
        theta = 0.0;
        t0 = active_start_s;
    }
    if (eval_time_s <= t0) {
        return std::max(0.0, theta);
    }
    const double rate = (command_throttle >= theta) ? up : down;
    if (rate <= 0.0) {
        return std::max(0.0, command_throttle);  // direction not configured
    }
    const double dt = eval_time_s - t0;
    return std::max(0.0, command_throttle + (theta - command_throttle) * std::exp(-rate * dt));
}

} // namespace

void EngineActionSchedule::add_action(const EngineTimeAction& action)
{
    actions_.push_back(action);
}

double EngineActionSchedule::throttle_at(double time_s, bool default_engine_enabled) const
{
    if (actions_.empty()) {
        return default_engine_enabled ? 1.0 : 0.0;
    }

    double throttle = 0.0;
    for (const auto& action : actions_) {
        if (time_s >= action.start_time_s && time_s <= action.end_time_s) {
            throttle = action.throttle;
        }
    }
    return clamp(throttle, 0.0, 1.0);
}

VehicleConsumptionPropagator::VehicleConsumptionPropagator(
    post2::vehicle::VehicleConfig config,
    EngineActionSchedule schedule)
    : config_(std::move(config))
    , schedule_(std::move(schedule))
{
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&config_);
}

post2::vehicle::VehicleRuntimeState VehicleConsumptionPropagator::make_initial_state(
    const post2::vehicle::CartesianState6D& motion,
    double time_s) const
{
    return post2::vehicle::make_initial_runtime_state(config_, motion, time_s);
}

double VehicleConsumptionPropagator::throttle_at(double time_s) const
{
    return schedule_.throttle_at(time_s, config_.engine.enabled);
}

DerivativeResult VehicleConsumptionPropagator::compute_derivatives(
    double time_s,
    const post2::vehicle::VehicleRuntimeState& runtime_topology,
    const std::vector<double>& tank_masses_kg,
    const post2::vehicle::CartesianState6D& motion,
    const EngineCommand& command) const
{
    (void)motion;
    DerivativeResult result;
    const std::size_t n_tanks = post2::vehicle::total_tank_count(runtime_topology);
    result.tank_mass_dots_kgps.assign(n_tanks, 0.0);
    result.per_stage_engine.assign(runtime_topology.stages.size(), post2::vehicle::EngineState{});
    const double throttle = clamp(command.throttle, 0.0, 1.0);
    result.throttle = throttle;
    result.engine_enabled = command.enabled;
    result.engine_direction_eci = normalized_or(command.direction_eci, {1.0, 0.0, 0.0});

    // (a) T2T flows
    for (const auto& c : config_.tank_to_tank_connections) {
        if (c.start_time_s.has_value() && time_s < *c.start_time_s) {
            continue;
        }
        if (c.end_time_s.has_value() && time_s > *c.end_time_s) {
            continue;
        }
        const auto src = post2::vehicle::resolve_runtime_tank(runtime_topology, c.source);
        const auto dst = post2::vehicle::resolve_runtime_tank(runtime_topology, c.dest);
        if (!src.has_value() || !dst.has_value()) {
            continue;
        }
        const std::size_t i_src = post2::vehicle::flat_tank_index(runtime_topology, src->first, src->second);
        const std::size_t i_dst = post2::vehicle::flat_tank_index(runtime_topology, dst->first, dst->second);
        if (i_src >= tank_masses_kg.size() || i_dst >= tank_masses_kg.size()) {
            continue;
        }
        const double src_kg = tank_masses_kg[i_src];
        const double dst_kg = tank_masses_kg[i_dst];
        const double dst_cap = tank_capacity_kg(config_, c.dest.stage_name, c.dest.tank_name);
        const bool src_has_mass = src_kg > kEpsKg;
        const bool dst_has_room = dst_cap > 0.0 ? (dst_kg < dst_cap - kEpsKg) : true;
        const double rate = (src_has_mass && dst_has_room) ? std::max(0.0, c.rate_kgps) : 0.0;
        result.tank_mass_dots_kgps[i_src] -= rate;
        result.tank_mass_dots_kgps[i_dst] += rate;
    }

    // (b) Effective total mass for thrust-to-acceleration
    const double total_mass_kg = std::max(1.0, total_attached_mass_kg(runtime_topology, tank_masses_kg));

    // (c) Per-stage engine demand -> distribute to feed_tanks in priority order
    const std::vector<post2::vehicle::StageConfig> stage_configs =
        post2::vehicle::effective_stage_configs(config_);
    if (command.enabled && !runtime_topology.stages.empty()) {
        for (std::size_t i = 0; i < runtime_topology.stages.size() && i < stage_configs.size(); ++i) {
            if (command.controlled_stage_index >= 0 &&
                i != static_cast<std::size_t>(command.controlled_stage_index)) {
                continue;
            }
            const auto& stage_rt = runtime_topology.stages[i];
            const auto& stage_cfg = stage_configs[i];
            post2::vehicle::EngineState& engine_out = result.per_stage_engine[i];
            engine_out.enabled = stage_cfg.engine.enabled;
            engine_out.isp_s = stage_cfg.engine.isp_vac_s;
            engine_out.direction_body = result.engine_direction_eci;

            if (!stage_rt.attached || !stage_rt.active ||
                !stage_cfg.engine.enabled ||
                stage_cfg.engine.thrust_vac_n <= 0.0 ||
                stage_cfg.engine.isp_vac_s <= 0.0 ||
                throttle <= 0.0 ||
                stage_cfg.engine.feed_tanks.empty()) {
                continue;
            }

            // Steady (target) performance at the commanded throttle. This is the
            // thrust the engine is spooling toward; it defines the denominator of
            // the "thrust established" fraction and is reported as commanded_thrust.
            EnginePerformanceInputs steady_in;
            steady_in.throttle_command = throttle;
            steady_in.ambient_pressure_pa = command.ambient_pressure_pa;
            steady_in.ignited_engine_count = command.ignited_engine_count;
            const EnginePerformanceOutputs steady = evaluate_engine(stage_cfg.engine, steady_in);

            // Transient spool: the actual throttle lags the command. The
            // reference point is the start of the current integration step
            // (runtime_topology carries its spool state and time); the closed-form
            // relaxation gives the actual throttle at this evaluation time.
            double ignition_time_s = stage_rt.engine.ignition_time_s;
            if (ignition_time_s < 0.0) {
                ignition_time_s = runtime_topology.time_s;  // ignites at step start
            }
            const double spool_eff = spooled_throttle(
                stage_cfg.engine,
                ignition_time_s,
                stage_rt.engine.spool_throttle,
                runtime_topology.time_s,
                time_s,
                throttle);

            engine_out.ignition_time_s = ignition_time_s;
            engine_out.spool_throttle = spool_eff;
            engine_out.commanded_thrust_n = steady.thrust_n;
            result.total_commanded_thrust_n += steady.thrust_n;

            // EnginePerformanceModel produces the cluster-aggregate thrust/mdot at
            // the (spooled) actual throttle and ambient pressure. The tank gating
            // below downscales these by the propellant available this step.
            EnginePerformanceInputs perf_in;
            perf_in.throttle_command = spool_eff;
            perf_in.ambient_pressure_pa = command.ambient_pressure_pa;
            perf_in.ignited_engine_count = command.ignited_engine_count;
            const EnginePerformanceOutputs perf = evaluate_engine(stage_cfg.engine, perf_in);
            if (perf.thrust_n <= 0.0 || perf.mdot_kgps <= 0.0) {
                // Still in ignition dead-time / below min throttle: no thrust yet.
                engine_out.throttle = perf.effective_throttle;
                continue;
            }

            const double commanded_thrust_n = perf.thrust_n;
            const double requested_flow_kgps = perf.mdot_kgps;

            double remaining = requested_flow_kgps;
            for (const auto& feed : stage_cfg.engine.feed_tanks) {
                if (remaining <= 0.0) {
                    break;
                }
                const auto resolved = post2::vehicle::resolve_runtime_tank(runtime_topology, feed);
                if (!resolved.has_value()) {
                    continue;
                }
                const std::size_t flat = post2::vehicle::flat_tank_index(
                    runtime_topology, resolved->first, resolved->second);
                if (flat >= tank_masses_kg.size()) {
                    continue;
                }
                const double available_kg = tank_masses_kg[flat];
                // Hard cutoff: a tank below kEpsKg supplies nothing. The
                // integrator's tank-empty event normally pre-empts the step
                // before we reach this case; this guard catches the rest.
                if (available_kg <= kEpsKg) {
                    continue;
                }
                const double take = remaining;
                result.tank_mass_dots_kgps[flat] -= take;
                remaining -= take;
            }

            const double actual_flow_kgps = requested_flow_kgps - std::max(0.0, remaining);
            const double flow_ratio = requested_flow_kgps > 0.0
                ? actual_flow_kgps / requested_flow_kgps
                : 0.0;
            const double actual_thrust_n = commanded_thrust_n * flow_ratio;

            // commanded_thrust_n here is the spooled (current) thrust; the steady
            // target was already recorded as engine_out.commanded_thrust_n above.
            engine_out.throttle = perf.effective_throttle;
            engine_out.actual_thrust_n = actual_thrust_n;
            engine_out.isp_s = perf.isp_s;
            engine_out.mass_flow_kgps = actual_flow_kgps;
            engine_out.firing = actual_thrust_n > 0.0;

            result.total_actual_thrust_n += actual_thrust_n;
            result.total_mass_flow_kgps += actual_flow_kgps;
            result.weighted_isp_s += perf.isp_s * actual_thrust_n;
        }
    }

    result.thrust_acceleration_mps2 =
        result.engine_direction_eci * (result.total_actual_thrust_n / total_mass_kg);
    return result;
}

post2::vehicle::VehicleRuntimeState VehicleConsumptionPropagator::commit(
    const post2::vehicle::VehicleRuntimeState& previous,
    const post2::integrators::ExtendedState& integrated,
    double next_time_s,
    const DerivativeResult& last_eval,
    const EngineCommand& command) const
{
    post2::vehicle::VehicleRuntimeState next = previous;
    next.time_s = next_time_s;
    next.vehicle.motion = integrated.motion;
    next.vehicle.rigid_body = integrated.rigid_body;
    next.engine.enabled = command.enabled;
    next.engine.direction_body = normalized_or(command.direction_eci, {1.0, 0.0, 0.0});

    // Reset per-stage engine bookkeeping to zero before applying last_eval.
    for (auto& stage : next.stages) {
        stage.engine.throttle = 0.0;
        stage.engine.commanded_thrust_n = 0.0;
        stage.engine.actual_thrust_n = 0.0;
        stage.engine.mass_flow_kgps = 0.0;
        stage.engine.firing = false;
    }
    for (std::size_t i = 0; i < next.stages.size() && i < last_eval.per_stage_engine.size(); ++i) {
        const post2::vehicle::EngineState& src = last_eval.per_stage_engine[i];
        post2::vehicle::EngineState& dst = next.stages[i].engine;
        dst.enabled = src.enabled;
        dst.firing = src.firing;
        dst.throttle = src.throttle;
        dst.commanded_thrust_n = src.commanded_thrust_n;
        dst.actual_thrust_n = src.actual_thrust_n;
        dst.isp_s = src.isp_s;
        dst.mass_flow_kgps = src.mass_flow_kgps;
        dst.direction_body = src.direction_body;
    }

    // Advance the per-stage transient spool state to the end of the step. This
    // is recomputed authoritatively from the start-of-step state (previous)
    // rather than from last_eval, whose spool sample is taken at an arbitrary
    // sub-step time. An off->on edge stamps the ignition time; losing
    // eligibility resets the spool to rest.
    const std::vector<post2::vehicle::StageConfig> stage_configs =
        post2::vehicle::effective_stage_configs(config_);
    const double cmd_throttle = clamp(command.throttle, 0.0, 1.0);
    double aggregate_spool = 0.0;
    double aggregate_ignition_time_s = -1.0;
    for (std::size_t i = 0; i < next.stages.size() && i < stage_configs.size(); ++i) {
        const post2::vehicle::EngineState& prev_engine = previous.stages[i].engine;
        const post2::vehicle::EngineConfig& ecfg = stage_configs[i].engine;
        const post2::vehicle::StageRuntimeState& srt = next.stages[i];
        const bool controller_allows_stage =
            command.controlled_stage_index < 0 ||
            i == static_cast<std::size_t>(command.controlled_stage_index);
        const bool eligible = command.enabled && srt.active && srt.attached &&
            controller_allows_stage &&
            ecfg.enabled && ecfg.thrust_vac_n > 0.0 && ecfg.isp_vac_s > 0.0;

        double ignition_time_s = prev_engine.ignition_time_s;
        double spool_start = prev_engine.spool_throttle;
        if (!eligible) {
            ignition_time_s = -1.0;
            spool_start = 0.0;
        } else if (ignition_time_s < 0.0) {
            ignition_time_s = previous.time_s;  // off->on edge: ignite at step start
            spool_start = 0.0;
        }
        const double spool_end = eligible
            ? spooled_throttle(ecfg, ignition_time_s, spool_start, previous.time_s,
                               next_time_s, cmd_throttle)
            : 0.0;
        next.stages[i].engine.ignition_time_s = ignition_time_s;
        next.stages[i].engine.spool_throttle = spool_end;
        if (eligible) {
            aggregate_spool = std::max(aggregate_spool, spool_end);
            if (ignition_time_s >= 0.0 &&
                (aggregate_ignition_time_s < 0.0 || ignition_time_s < aggregate_ignition_time_s)) {
                aggregate_ignition_time_s = ignition_time_s;
            }
        }
    }
    next.engine.spool_throttle = aggregate_spool;
    next.engine.ignition_time_s = aggregate_ignition_time_s;

    // Vehicle-level aggregated engine snapshot.
    next.engine.throttle = last_eval.throttle;
    next.engine.commanded_thrust_n = last_eval.total_commanded_thrust_n;
    next.engine.actual_thrust_n = last_eval.total_actual_thrust_n;
    next.engine.mass_flow_kgps = last_eval.total_mass_flow_kgps;
    next.engine.isp_s = last_eval.total_actual_thrust_n > 0.0
        ? last_eval.weighted_isp_s / last_eval.total_actual_thrust_n
        : 0.0;
    next.engine.firing = last_eval.total_actual_thrust_n > 0.0;

    post2::vehicle::write_tank_masses_flat(&next, integrated.tank_masses_kg);
    post2::vehicle::refresh_vehicle_masses(&next);
    return next;
}

} // namespace post2::propagation
