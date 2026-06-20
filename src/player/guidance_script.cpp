#include "post2/player/guidance_script.hpp"

#include <algorithm>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

namespace post2::player {

namespace {

std::vector<std::string> split_csv_line(const std::string& line)
{
    std::vector<std::string> parts;
    std::istringstream input(line);
    std::string item;
    while (std::getline(input, item, ',')) {
        parts.push_back(item);
    }
    return parts;
}

std::string lowercase(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

double field_double(const std::vector<std::string>& row, std::size_t index, double fallback = 0.0)
{
    if (index >= row.size()) {
        return fallback;
    }
    try {
        return std::stod(row[index]);
    } catch (...) {
        return fallback;
    }
}

int field_int(const std::vector<std::string>& row, std::size_t index, int fallback = 0)
{
    return static_cast<int>(field_double(row, index, static_cast<double>(fallback)));
}

std::string field_string(const std::vector<std::string>& row, std::size_t index)
{
    return index < row.size() ? row[index] : std::string();
}

ScriptPhase& phase_at(GuidanceScript* script, int index)
{
    for (auto& phase : script->phases) {
        if (phase.index == index) {
            return phase;
        }
    }
    ScriptPhase phase;
    phase.index = index;
    script->phases.push_back(phase);
    return script->phases.back();
}

} // namespace

bool parse_guidance_script(const std::string& text, GuidanceScript* out, std::string* error)
{
    if (!out) {
        if (error) {
            *error = "null output";
        }
        return false;
    }
    *out = GuidanceScript{};

    std::istringstream input(text);
    std::string line;
    bool saw_meta = false;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) {
            continue;
        }
        const auto row = split_csv_line(line);
        if (row.empty()) {
            continue;
        }
        const std::string record = row[0];

        if (record == "META") {
            out->meta.name = field_string(row, 1);
            out->meta.mu_m3s2 = field_double(row, 2);
            out->meta.radius_m = field_double(row, 3);
            out->meta.rotation_rad_s = field_double(row, 4);
            out->meta.rotation_at_epoch_rad = field_double(row, 5);
            out->meta.launch_lat_deg = field_double(row, 6);
            out->meta.launch_lon_deg = field_double(row, 7);
            out->meta.launch_alt_m = field_double(row, 8);
            saw_meta = true;
        } else if (record == "STAGE") {
            ScriptStage stage;
            stage.index = field_int(row, 1);
            stage.name = field_string(row, 2);
            stage.thrust_n = field_double(row, 3);
            stage.exhaust_velocity_mps = field_double(row, 4);
            stage.mdot_kgps = field_double(row, 5);
            stage.propellant_kg = field_double(row, 6);
            stage.dry_mass_kg = field_double(row, 7);
            out->stages.push_back(stage);
        } else if (record == "PHASE") {
            ScriptPhase& phase = phase_at(out, field_int(row, 1));
            phase.name = field_string(row, 2);
            phase.steering_type = lowercase(field_string(row, 3));
            phase.termination_type = field_string(row, 4);
            phase.termination_comparison = field_string(row, 5);
            phase.termination_value = field_double(row, 6);
            phase.throttle_type = lowercase(field_string(row, 7));
            phase.throttle_c0 = field_double(row, 8);
            phase.throttle_c1 = field_double(row, 9);
            phase.throttle_c2 = field_double(row, 10);
            // Trailing field (added with the thrust-transient work); old scripts
            // omit it and default to "not clamped".
            phase.hold_down_clamp_initial_active = field_double(row, 11, 0.0) != 0.0;
        } else if (record == "POLY") {
            ScriptPhase& phase = phase_at(out, field_int(row, 1));
            phase.azimuth[0] = field_double(row, 2);
            phase.azimuth[1] = field_double(row, 3);
            phase.azimuth[2] = field_double(row, 4);
            phase.elevation[0] = field_double(row, 5);
            phase.elevation[1] = field_double(row, 6);
            phase.elevation[2] = field_double(row, 7);
            phase.roll[0] = field_double(row, 8);
            phase.roll[1] = field_double(row, 9);
            phase.roll[2] = field_double(row, 10);
        } else if (record == "SEG") {
            ScriptPhase& phase = phase_at(out, field_int(row, 1));
            ScriptSegment segment;
            segment.start_time_s = field_double(row, 2);
            segment.order = field_int(row, 3);
            const int count = std::max(0, segment.order) + 1;
            std::size_t cursor = 4;
            for (int k = 0; k < count; ++k) {
                segment.azimuth_coefficients.push_back(field_double(row, cursor++));
            }
            for (int k = 0; k < count; ++k) {
                segment.elevation_coefficients.push_back(field_double(row, cursor++));
            }
            phase.segments.push_back(segment);
        } else if (record == "TAN") {
            ScriptPhase& phase = phase_at(out, field_int(row, 1));
            phase.has_tangent = true;
            phase.tangent_a = field_double(row, 2);
            phase.tangent_a_dot = field_double(row, 3);
            phase.tangent_b = field_double(row, 4);
            phase.tangent_b_dot = field_double(row, 5);
            phase.tangent_t_offset_s = field_double(row, 6);
            phase.tangent_bilinear = field_int(row, 7) != 0;
        } else if (record == "UPFG") {
            ScriptPhase& phase = phase_at(out, field_int(row, 1));
            phase.is_upfg = true;
            phase.upfg_periapsis_km = field_double(row, 2);
            phase.upfg_apoapsis_km = field_double(row, 3);
            phase.upfg_inclination_deg = field_double(row, 4);
        } else if (record == "ACTION") {
            ScriptAction action;
            action.phase_index = field_int(row, 1);
            action.time_s = field_double(row, 2);
            action.type = field_string(row, 3);
            action.value = field_int(row, 4) != 0;
            action.stage_index = field_int(row, 5, -1);
            action.stage_name = field_string(row, 6);
            phase_at(out, action.phase_index).actions.push_back(action);
        }
        // EVENT records are accepted but not yet driven by the player.
    }

    std::sort(out->phases.begin(), out->phases.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.index < rhs.index;
    });

    if (!saw_meta) {
        if (error) {
            *error = "guidance script missing META record";
        }
        return false;
    }
    if (out->phases.empty()) {
        if (error) {
            *error = "guidance script has no PHASE records";
        }
        return false;
    }
    return true;
}

} // namespace post2::player
