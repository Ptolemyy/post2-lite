#pragma once

#include <string>

#include "post2/core/state_log.hpp"
#include "post2/core/types.hpp"

namespace post2::core {

std::string trajectory_to_csv(const StateLog& state_log);
std::string kos_trajectory_to_csv(const StateLog& state_log, const CaseConfig& case_config);
SimulationResult trajectory_from_csv(const std::string& csv);

bool write_csv_file(const std::string& path, const StateLog& state_log, std::string* error);
bool write_kos_csv_file(
    const std::string& path,
    const StateLog& state_log,
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
