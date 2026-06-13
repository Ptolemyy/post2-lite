#include "post2/vehicle/runtime_state.hpp"

#include <algorithm>
#include <cstddef>
#include <cctype>
#include <utility>

namespace post2::vehicle {

double total_propellant_kg(const std::vector<TankState>& tanks)
{
    double total = 0.0;
    for (const auto& tank : tanks) {
        total += std::max(0.0, tank.remaining_kg);
    }
    return total;
}

double total_stage_dry_mass_kg(const std::vector<StageConfig>& stages)
{
    double total = 0.0;
    for (const auto& stage : stages) {
        total += std::max(0.0, stage.dry_mass_kg);
    }
    return total;
}

double total_attached_stage_dry_mass_kg(const std::vector<StageRuntimeState>& stages)
{
    double total = 0.0;
    for (const auto& stage : stages) {
        if (stage.attached) {
            total += std::max(0.0, stage.dry_mass_kg);
        }
    }
    return total;
}

std::vector<StageConfig> effective_stage_configs(const VehicleConfig& config)
{
    if (!config.stages.empty()) {
        return config.stages;
    }

    StageConfig stage;
    stage.name = "stage 1";
    stage.active = true;
    stage.engine = config.engine;
    stage.tanks = config.tanks.empty() ? std::vector<TankConfig>{TankConfig{}} : config.tanks;
    return {std::move(stage)};
}

double effective_dry_mass_kg(const VehicleConfig& config)
{
    const std::vector<StageConfig> stages = effective_stage_configs(config);
    const double stage_dry_mass_kg = total_stage_dry_mass_kg(stages);
    return stage_dry_mass_kg > 0.0 ? stage_dry_mass_kg : config.dry_mass_kg;
}

double payload_stage_dry_mass_kg(const VehicleConfig& config)
{
    const std::vector<StageConfig> stages = effective_stage_configs(config);
    for (const auto& stage : stages) {
        std::string name = stage.name;
        std::transform(name.begin(), name.end(), name.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        if (name.find("payload") != std::string::npos) {
            return std::max(0.0, stage.dry_mass_kg);
        }
    }
    if (!stages.empty()) {
        return std::max(0.0, stages.back().dry_mass_kg);
    }
    return 0.0;
}

void sync_legacy_vehicle_fields_from_first_stage(VehicleConfig* config)
{
    if (!config || config->stages.empty()) {
        return;
    }
    config->engine = config->stages.front().engine;   // copies feed_tanks too
    config->tanks = config->stages.front().tanks;
}

double stage_propellant_kg(const StageRuntimeState& stage)
{
    return total_propellant_kg(stage.tanks);
}

double active_stage_propellant_kg(const VehicleRuntimeState& runtime)
{
    if (runtime.stages.empty()) {
        return total_propellant_kg(runtime.tanks);
    }

    double total = 0.0;
    for (const auto& stage : runtime.stages) {
        if (stage.attached && stage.active) {
            total += stage_propellant_kg(stage);
        }
    }
    return total;
}

namespace {

void sync_flat_tanks_from_stages(VehicleRuntimeState* runtime)
{
    if (!runtime || runtime->stages.empty()) {
        return;
    }

    runtime->tanks.clear();
    for (const auto& stage : runtime->stages) {
        if (!stage.attached) {
            continue;
        }
        for (const auto& tank : stage.tanks) {
            TankState flattened = tank;
            flattened.name = stage.name + "/" + tank.name;
            runtime->tanks.push_back(std::move(flattened));
        }
    }
}

} // namespace

void refresh_vehicle_masses(VehicleRuntimeState* runtime)
{
    sync_flat_tanks_from_stages(runtime);
    runtime->vehicle.propellant_mass_kg = total_propellant_kg(runtime->tanks);
    if (!runtime->stages.empty()) {
        const double stage_dry_mass_kg = total_attached_stage_dry_mass_kg(runtime->stages);
        if (stage_dry_mass_kg > 0.0) {
            runtime->vehicle.dry_mass_kg = stage_dry_mass_kg;
        }
    }
    runtime->vehicle.total_mass_kg = runtime->vehicle.dry_mass_kg + runtime->vehicle.propellant_mass_kg;
}

VehicleRuntimeState make_initial_runtime_state(
    const VehicleConfig& config,
    const CartesianState6D& motion,
    double time_s)
{
    VehicleRuntimeState runtime;
    runtime.time_s = time_s;
    runtime.vehicle.motion = motion;
    runtime.vehicle.dry_mass_kg = effective_dry_mass_kg(config);
    runtime.engine.enabled = config.engine.enabled;
    runtime.engine.firing = false;
    runtime.engine.throttle = 0.0;
    runtime.engine.commanded_thrust_n = 0.0;
    runtime.engine.actual_thrust_n = 0.0;
    runtime.engine.isp_s = config.engine.isp_vac_s;
    runtime.engine.mass_flow_kgps = 0.0;
    runtime.engine.direction_body = config.engine.direction_body;

    const std::vector<StageConfig> stages = effective_stage_configs(config);
    runtime.stages.reserve(stages.size());
    for (const auto& stage_config : stages) {
        StageRuntimeState stage;
        stage.name = stage_config.name;
        stage.active = stage_config.active;
        stage.attached = stage_config.attached;
        stage.dry_mass_kg = stage_config.dry_mass_kg;
        stage.engine.enabled = stage_config.engine.enabled;
        stage.engine.firing = false;
        stage.engine.throttle = 0.0;
        stage.engine.commanded_thrust_n = 0.0;
        stage.engine.actual_thrust_n = 0.0;
        stage.engine.isp_s = stage_config.engine.isp_vac_s;
        stage.engine.mass_flow_kgps = 0.0;
        stage.engine.direction_body = stage_config.engine.direction_body;
        stage.tanks.reserve(stage_config.tanks.size());
        for (const auto& tank_config : stage_config.tanks) {
            stage.tanks.push_back({
                tank_config.name,
                tank_config.propellant,
                tank_config.capacity_kg,
                tank_config.initial_kg,
            });
        }
        runtime.stages.push_back(std::move(stage));
    }

    refresh_vehicle_masses(&runtime);
    if (!runtime.stages.empty()) {
        runtime.engine = runtime.stages.front().engine;
    }
    return runtime;
}

double consume_propellant_kg(VehicleRuntimeState* runtime, double requested_kg)
{
    if (!runtime->stages.empty()) {
        double remaining_request = std::max(0.0, requested_kg);
        double consumed = 0.0;
        for (std::size_t i = 0; i < runtime->stages.size() && remaining_request > 0.0; ++i) {
            if (!runtime->stages[i].attached || !runtime->stages[i].active) {
                continue;
            }
            const double draw = consume_stage_propellant_kg(runtime, i, remaining_request);
            remaining_request -= draw;
            consumed += draw;
        }
        refresh_vehicle_masses(runtime);
        return consumed;
    }

    double remaining_request = std::max(0.0, requested_kg);
    double consumed = 0.0;

    for (auto& tank : runtime->tanks) {
        if (remaining_request <= 0.0) {
            break;
        }

        const double available = std::max(0.0, tank.remaining_kg);
        const double draw = std::min(available, remaining_request);
        tank.remaining_kg -= draw;
        remaining_request -= draw;
        consumed += draw;
    }

    refresh_vehicle_masses(runtime);
    return consumed;
}

double consume_stage_propellant_kg(
    VehicleRuntimeState* runtime,
    std::size_t stage_index,
    double requested_kg)
{
    if (!runtime || stage_index >= runtime->stages.size()) {
        return 0.0;
    }

    double remaining_request = std::max(0.0, requested_kg);
    double consumed = 0.0;
    auto& stage = runtime->stages[stage_index];
    for (auto& tank : stage.tanks) {
        if (remaining_request <= 0.0) {
            break;
        }

        const double available = std::max(0.0, tank.remaining_kg);
        const double draw = std::min(available, remaining_request);
        tank.remaining_kg -= draw;
        remaining_request -= draw;
        consumed += draw;
    }
    sync_flat_tanks_from_stages(runtime);
    return consumed;
}

double active_max_thrust_n(const VehicleConfig& config, const VehicleRuntimeState& runtime)
{
    const std::vector<StageConfig> stage_configs = effective_stage_configs(config);
    auto cluster_thrust_n = [](const EngineConfig& e) {
        return std::max(0.0, e.thrust_vac_n) * std::max(0, e.engine_count);
    };
    if (runtime.stages.empty()) {
        return config.engine.enabled ? cluster_thrust_n(config.engine) : 0.0;
    }

    double total = 0.0;
    for (std::size_t i = 0; i < stage_configs.size() && i < runtime.stages.size(); ++i) {
        if (runtime.stages[i].attached &&
            runtime.stages[i].active &&
            runtime.stages[i].engine.enabled &&
            stage_configs[i].engine.enabled) {
            total += cluster_thrust_n(stage_configs[i].engine);
        }
    }
    return total;
}

bool set_stage_active(VehicleRuntimeState* runtime, std::size_t stage_index, bool active)
{
    if (!runtime || stage_index >= runtime->stages.size()) {
        return false;
    }
    runtime->stages[stage_index].active = active;
    if (!active) {
        runtime->stages[stage_index].engine.firing = false;
        runtime->stages[stage_index].engine.throttle = 0.0;
        runtime->stages[stage_index].engine.commanded_thrust_n = 0.0;
        runtime->stages[stage_index].engine.actual_thrust_n = 0.0;
        runtime->stages[stage_index].engine.mass_flow_kgps = 0.0;
    }
    refresh_vehicle_masses(runtime);
    return true;
}

bool set_stage_attached(VehicleRuntimeState* runtime, std::size_t stage_index, bool attached)
{
    if (!runtime || stage_index >= runtime->stages.size()) {
        return false;
    }
    runtime->stages[stage_index].attached = attached;
    if (!attached) {
        runtime->stages[stage_index].active = false;
        runtime->stages[stage_index].engine.firing = false;
        runtime->stages[stage_index].engine.throttle = 0.0;
        runtime->stages[stage_index].engine.commanded_thrust_n = 0.0;
        runtime->stages[stage_index].engine.actual_thrust_n = 0.0;
        runtime->stages[stage_index].engine.mass_flow_kgps = 0.0;
    }
    refresh_vehicle_masses(runtime);
    return true;
}

std::optional<std::pair<std::size_t, std::size_t>>
    resolve_tank_ref(const VehicleConfig& config, const TankRef& ref)
{
    const std::vector<StageConfig> stages = effective_stage_configs(config);
    for (std::size_t i = 0; i < stages.size(); ++i) {
        if (stages[i].name != ref.stage_name) {
            continue;
        }
        for (std::size_t j = 0; j < stages[i].tanks.size(); ++j) {
            if (stages[i].tanks[j].name == ref.tank_name) {
                return std::make_pair(i, j);
            }
        }
    }
    return std::nullopt;
}

std::optional<std::pair<std::size_t, std::size_t>>
    resolve_runtime_tank(const VehicleRuntimeState& runtime, const TankRef& ref)
{
    for (std::size_t i = 0; i < runtime.stages.size(); ++i) {
        if (runtime.stages[i].name != ref.stage_name) {
            continue;
        }
        for (std::size_t j = 0; j < runtime.stages[i].tanks.size(); ++j) {
            if (runtime.stages[i].tanks[j].name == ref.tank_name) {
                return std::make_pair(i, j);
            }
        }
    }
    return std::nullopt;
}

std::size_t total_tank_count(const VehicleRuntimeState& runtime)
{
    std::size_t total = 0;
    for (const auto& stage : runtime.stages) {
        total += stage.tanks.size();
    }
    return total;
}

std::size_t flat_tank_index(
    const VehicleRuntimeState& runtime,
    std::size_t stage_index,
    std::size_t tank_index)
{
    std::size_t base = 0;
    for (std::size_t i = 0; i < stage_index && i < runtime.stages.size(); ++i) {
        base += runtime.stages[i].tanks.size();
    }
    return base + tank_index;
}

std::vector<double> read_tank_masses_flat(const VehicleRuntimeState& runtime)
{
    std::vector<double> masses;
    masses.reserve(total_tank_count(runtime));
    for (const auto& stage : runtime.stages) {
        for (const auto& tank : stage.tanks) {
            masses.push_back(tank.remaining_kg);
        }
    }
    return masses;
}

void write_tank_masses_flat(VehicleRuntimeState* runtime, const std::vector<double>& masses_kg)
{
    if (!runtime) {
        return;
    }
    std::size_t cursor = 0;
    for (auto& stage : runtime->stages) {
        for (auto& tank : stage.tanks) {
            if (cursor >= masses_kg.size()) {
                return;
            }
            tank.remaining_kg = std::max(0.0, masses_kg[cursor]);
            ++cursor;
        }
    }
}

} // namespace post2::vehicle
