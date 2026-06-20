#pragma once

#include <string>

#include "post2/core/ksp_vehicle_site_import.hpp"
#include "post2/core/types.hpp"

namespace post2::core {

// Generate the per-staging-configuration aerodynamic tables for `config` from a
// KSP import and wire the case to use them. Writes the table CSVs into
// `output_dir` (which must already end with a path separator, or be empty for
// the current directory), then:
//   * sets config->vehicle.aero to use the generated tables (enabled, use_table,
//     stage_tables, plus the legacy aero_table_path / reference area / geometry
//     pointers kept in sync with the full-stack table),
//   * flips every phase to real aerodynamics and the US 1976 atmosphere.
//
// A level-0 (full stack) table is always produced; a level-1 (post-booster
// upper-stack) table is added when the vehicle has at least two propulsive
// stages, using the upper-stack length measured from the import when available
// and a slenderness estimate otherwise. Returns false (and sets *error) if a
// table CSV cannot be written. This is the single source of truth shared by the
// GUI "Import KSP" action and the post2_regen_aero CLI.
bool generate_case_aero_tables(CaseConfig* config,
                               const KspVehicleSiteImport& imported,
                               const std::string& output_dir,
                               std::string* error);

} // namespace post2::core
