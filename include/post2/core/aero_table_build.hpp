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
// A full-stack table [0, top] is always produced. For a multi-stage vehicle the
// whole staging ladder is generated into stage_tables:
//   * open-top upper-stack tables [lo, top] for lo = 1..top -- the
//     configurations the ascending vehicle flies through as lower stages drop
//     (the terminal [top, top] is the last stage on its own); and
//   * single-stage tables [i, i] for the lower stages i = 0..top-1 -- one
//     separated stage flying alone (booster recovery / flyback), built with that
//     stage's own geometry and no fairing.
// The force model auto-selects among these by the vehicle's currently-attached
// stage range. The first-stage [0, 0] table is also mirrored into
// aero.first_stage_table for tooling / back-compat. Per-stage lengths use the
// measured full and upper-stack lengths from the import when available and a
// wet-mass distribution otherwise. Returns false (and sets *error) if a table
// CSV cannot be written. This is the single source of truth shared by the GUI
// "Import KSP" action and the post2_regen_aero CLI.
bool generate_case_aero_tables(CaseConfig* config,
                               const KspVehicleSiteImport& imported,
                               const std::string& output_dir,
                               std::string* error);

} // namespace post2::core
