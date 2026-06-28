#pragma once

#include <cstddef>
#include <optional>
#include <utility>

#include "post2/vehicle/vehicle.hpp"

namespace post2::vehicle {

double total_propellant_kg(const std::vector<TankState>& tanks);
std::vector<StageConfig> effective_stage_configs(const VehicleConfig& config);
void sync_legacy_vehicle_fields_from_first_stage(VehicleConfig* config);
double effective_dry_mass_kg(const VehicleConfig& config);
double payload_stage_dry_mass_kg(const VehicleConfig& config);
void refresh_vehicle_masses(VehicleRuntimeState* runtime);

VehicleRuntimeState make_initial_runtime_state(
    const VehicleConfig& config,
    const CartesianState6D& motion,
    double time_s);

double consume_propellant_kg(VehicleRuntimeState* runtime, double requested_kg);
double consume_stage_propellant_kg(
    VehicleRuntimeState* runtime,
    std::size_t stage_index,
    double requested_kg);
double stage_propellant_kg(const StageRuntimeState& stage);
double active_stage_propellant_kg(const VehicleRuntimeState& runtime);
double active_max_thrust_n(const VehicleConfig& config, const VehicleRuntimeState& runtime);
bool set_stage_active(VehicleRuntimeState* runtime, std::size_t stage_index, bool active);
bool set_stage_attached(VehicleRuntimeState* runtime, std::size_t stage_index, bool attached);

// Selects the per-staging-configuration AeroStageTable matching the vehicle's
// currently-attached stages (the contiguous attached range [lo, hi]):
//   * exact bounded match for a separated stage / sub-stack flying alone;
//   * else the open-top upper-stack table for the ascending vehicle;
//   * else the nearest table at or below the lowest attached stage.
// Returns nullptr only when aero.stage_tables is empty. Used by both the force
// model (CD/CL table) and the heat-flux diagnostic so they stay consistent.
const AeroStageTable* select_active_aero_stage_table(
    const AeroConfig& aero,
    const VehicleRuntimeState& runtime);

// Tank addressing across stages. (stage_index, tank_index) is the canonical
// ordering - flat_tank_index walks runtime.stages[*].tanks[*] in declaration
// order. Detached stages keep their slots so the ODE state vector size is
// constant within a phase.
std::optional<std::pair<std::size_t, std::size_t>>
    resolve_tank_ref(const VehicleConfig& config, const TankRef& ref);
std::optional<std::pair<std::size_t, std::size_t>>
    resolve_runtime_tank(const VehicleRuntimeState& runtime, const TankRef& ref);

std::size_t total_tank_count(const VehicleRuntimeState& runtime);
std::size_t flat_tank_index(
    const VehicleRuntimeState& runtime,
    std::size_t stage_index,
    std::size_t tank_index);

std::vector<double> read_tank_masses_flat(const VehicleRuntimeState& runtime);
void write_tank_masses_flat(VehicleRuntimeState* runtime, const std::vector<double>& masses_kg);

} // namespace post2::vehicle
