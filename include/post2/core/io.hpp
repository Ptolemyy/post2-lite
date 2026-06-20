#pragma once

#include <string>

#include "post2/core/state_log.hpp"
#include "post2/core/types.hpp"

namespace post2::core {

std::string trajectory_to_csv(const StateLog& state_log);
// Compact, per-phase guidance "script" CSV consumed by the C++ kRPC player
// (post2_player). Unlike the dense trajectory CSV this emits only the guidance
// *parameters* -- a sectioned, record-typed file (META / STAGE / PHASE / POLY /
// SEG / TAN / UPFG / ACTION / EVENT). A UPFG phase is exported as a single UPFG
// marker row (periapsis/apoapsis/inclination), not sampled. The player re-flies
// the ascent by evaluating the polynomials and running the real UPFG.
std::string guidance_script_to_csv(const CaseConfig& case_config);
SimulationResult trajectory_from_csv(const std::string& csv);

bool write_csv_file(const std::string& path, const StateLog& state_log, std::string* error);
bool write_guidance_script_file(
    const std::string& path,
    const CaseConfig& case_config,
    std::string* error);
// Draws the ascent trajectory and, when `predicted_orbit` is non-null and
// non-empty, the integrated predicted orbit (distinct colour) with its
// apoapsis/periapsis (max/min altitude points) marked. Pass nullptr to skip.
bool write_svg_file(
    const std::string& path,
    const StateLog& state_log,
    const StateLog* predicted_orbit,
    std::string* error);

} // namespace post2::core
