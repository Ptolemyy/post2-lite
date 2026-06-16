#include "post2/core/io.hpp"

#include "post2/core/projection.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace post2::core {

namespace {

constexpr const char* kTrajectoryCsvHeader =
    "time_s,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,altitude_m,speed_mps,"
    "total_mass_kg,propellant_mass_kg,engine_thrust_n,engine_mass_flow_kgps,"
    "throttle,engine_direction_eci_x,engine_direction_eci_y,engine_direction_eci_z,"
    "ambient_pressure_pa,atmosphere_density_kgpm3,dynamic_pressure_pa,mach_number,"
    "hold_down_clamp_active,phase_index,phase_name";

bool parse_double(const std::string& text, double* value)
{
    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        *value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

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

void write_trajectory_csv_row(std::ostream& output, const LaunchVehicleStateLogEntry& point)
{
    output
        << point.time_s << ','
        << point.state.position_m.x << ','
        << point.state.position_m.y << ','
        << point.state.position_m.z << ','
        << point.state.velocity_mps.x << ','
        << point.state.velocity_mps.y << ','
        << point.state.velocity_mps.z << ','
        << point.altitude_m << ','
        << point.speed_mps << ','
        << point.total_mass_kg << ','
        << point.propellant_mass_kg << ','
        << point.engine_thrust_n << ','
        << point.engine_mass_flow_kgps << ','
        << point.throttle << ','
        << point.engine_direction_eci.x << ','
        << point.engine_direction_eci.y << ','
        << point.engine_direction_eci.z << ','
        << point.ambient_pressure_pa << ','
        << point.atmosphere_density_kgpm3 << ','
        << point.dynamic_pressure_pa << ','
        << point.mach_number << ','
        << (point.hold_down_clamp_active ? 1 : 0) << ','
        << point.phase_index << ','
        << point.phase_name;
}

std::vector<double> configured_phase_start_times(const CaseConfig& case_config)
{
    std::vector<double> starts;
    starts.reserve(case_config.phases.size());
    double time_s = 0.0;
    for (const auto& phase : case_config.phases) {
        starts.push_back(time_s);
        // Non-time terminations do not contribute a known duration here;
        // callers fall back to the state-log time for actual phase boundaries.
        if (phase.termination.type == "time") {
            time_s += phase.termination.value;
        }
    }
    return starts;
}

double phase_start_time_s(
    const LaunchVehicleStateLogEntry& point,
    const std::vector<double>& phase_starts)
{
    if (point.phase_index >= 0 &&
        static_cast<std::size_t>(point.phase_index) < phase_starts.size()) {
        return phase_starts[static_cast<std::size_t>(point.phase_index)];
    }
    return point.time_s;
}

const PhaseConfig* phase_for_point(
    const LaunchVehicleStateLogEntry& point,
    const CaseConfig& case_config)
{
    if (point.phase_index < 0 ||
        static_cast<std::size_t>(point.phase_index) >= case_config.phases.size()) {
        return nullptr;
    }
    return &case_config.phases[static_cast<std::size_t>(point.phase_index)];
}

bool is_generic_poly_steering(const PhaseConfig& phase)
{
    return lowercase(phase.steering_model.type) == "generic_poly";
}

bool is_poly_throttle(const PhaseConfig& phase)
{
    return lowercase(phase.throttle_model.type) == "poly";
}

bool is_segmented_poly_type(const std::string& type)
{
    const std::string normalized = lowercase(type);
    return normalized == "segmented_poly" ||
        normalized == "generic_segmented_poly" ||
        normalized == "piecewise_poly";
}

double coefficient_or_zero(const std::vector<double>& coefficients, std::size_t index)
{
    return index < coefficients.size() ? coefficients[index] : 0.0;
}

Poly2Config coefficients_to_phase_poly2(
    const std::vector<double>& coefficients,
    double segment_start_time_s)
{
    const double a0 = coefficient_or_zero(coefficients, 0);
    const double a1 = coefficient_or_zero(coefficients, 1);
    const double a2 = coefficient_or_zero(coefficients, 2);
    Poly2Config out;
    out.c0 = a0 - a1 * segment_start_time_s + a2 * segment_start_time_s * segment_start_time_s;
    out.c1 = a1 - 2.0 * a2 * segment_start_time_s;
    out.c2 = a2;
    return out;
}

const SegmentedPolySegmentConfig* selected_segment(
    const std::vector<SegmentedPolySegmentConfig>& segments,
    double phase_time_s)
{
    if (segments.empty()) {
        return nullptr;
    }
    const auto* selected = &segments.front();
    for (const auto& segment : segments) {
        if (phase_time_s >= segment.start_time_s) {
            selected = &segment;
        }
    }
    return selected;
}

const SegmentedSteeringPolySegmentConfig* selected_segment(
    const std::vector<SegmentedSteeringPolySegmentConfig>& segments,
    double phase_time_s)
{
    if (segments.empty()) {
        return nullptr;
    }
    const auto* selected = &segments.front();
    for (const auto& segment : segments) {
        if (phase_time_s >= segment.start_time_s) {
            selected = &segment;
        }
    }
    return selected;
}

bool steering_poly_for_time(
    const PhaseConfig& phase,
    double phase_time_s,
    Poly2Config* azimuth,
    Poly2Config* elevation,
    Poly2Config* roll)
{
    if (is_generic_poly_steering(phase)) {
        *azimuth = phase.steering_model.azimuth_deg;
        *elevation = phase.steering_model.elevation_deg;
        *roll = phase.steering_model.roll_deg;
        return true;
    }
    if (is_segmented_poly_type(phase.steering_model.type)) {
        const auto* segment = selected_segment(
            phase.steering_model.segmented_poly.segments,
            phase_time_s);
        if (!segment) {
            return false;
        }
        *azimuth = coefficients_to_phase_poly2(
            segment->azimuth_coefficients,
            segment->start_time_s);
        *elevation = coefficients_to_phase_poly2(
            segment->elevation_coefficients,
            segment->start_time_s);
        *roll = {};
        return true;
    }
    return false;
}

bool throttle_poly_for_time(
    const PhaseConfig& phase,
    double phase_time_s,
    Poly2Config* throttle)
{
    if (is_poly_throttle(phase)) {
        throttle->c0 = phase.throttle_model.c0;
        throttle->c1 = phase.throttle_model.c1;
        throttle->c2 = phase.throttle_model.c2;
        return true;
    }
    if (is_segmented_poly_type(phase.throttle_model.type)) {
        const auto* segment = selected_segment(
            phase.throttle_model.segmented_poly.segments,
            phase_time_s);
        if (!segment) {
            return false;
        }
        *throttle = coefficients_to_phase_poly2(segment->coefficients, segment->start_time_s);
        return true;
    }
    return false;
}

bool is_nonzero_poly(const Poly2Config& poly)
{
    constexpr double kCoeffEpsilon = 1.0e-12;
    return std::abs(poly.c0) > kCoeffEpsilon ||
        std::abs(poly.c1) > kCoeffEpsilon ||
        std::abs(poly.c2) > kCoeffEpsilon;
}

bool detached_stage_since(
    const LaunchVehicleStateLogEntry& previous,
    const LaunchVehicleStateLogEntry& current)
{
    const std::size_t count = std::min(previous.runtime.stages.size(), current.runtime.stages.size());
    for (std::size_t i = 0; i < count; ++i) {
        if (previous.runtime.stages[i].attached && !current.runtime.stages[i].attached) {
            return true;
        }
    }
    return false;
}

bool deactivated_stage_since(
    const LaunchVehicleStateLogEntry& previous,
    const LaunchVehicleStateLogEntry& current)
{
    const std::size_t count = std::min(previous.runtime.stages.size(), current.runtime.stages.size());
    for (std::size_t i = 0; i < count; ++i) {
        if (previous.runtime.stages[i].active && !current.runtime.stages[i].active) {
            return true;
        }
    }
    return false;
}

bool activated_stage_since(
    const LaunchVehicleStateLogEntry& previous,
    const LaunchVehicleStateLogEntry& current)
{
    const std::size_t count = std::min(previous.runtime.stages.size(), current.runtime.stages.size());
    for (std::size_t i = 0; i < count; ++i) {
        if (!previous.runtime.stages[i].active &&
            current.runtime.stages[i].active &&
            current.runtime.stages[i].attached) {
            return true;
        }
    }
    return false;
}

struct KosNavballAttitude {
    double yaw_deg = 90.0;    // navball heading: 0 = north, 90 = east
    double pitch_deg = 90.0;  // elevation above the local horizon: 90 = straight up
};

// Navball yaw/pitch the kOS player steers to. PITCH is reconstructed from
// engine_direction_eci (the actual flown attitude, robust to phase-boundary
// continuity which rewrites the stored elevation poly at runtime). AZIMUTH, for
// poly steering, is taken DIRECTLY from the commanded azimuth polynomial, NOT
// reconstructed: reconstructing azimuth from the thrust vector flips ~180 deg
// whenever the commanded elevation tips past vertical (a pure artifact of
// forcing pitch into [-90,90]) -- e.g. min-Q lofting produced yaw ~242 deg and
// flew the vehicle west. The commanded azimuth (constant 90 = due east here) has
// no such ambiguity. Non-poly phases fall back to reconstruction. Unsteered rows
// (pre-launch / hold-down / coast) are skipped so their default body axis cannot
// pollute the carry-forward that fills any remaining gaps.
std::vector<KosNavballAttitude> compute_kos_navball_attitude(
    const StateLog& state_log,
    const CaseConfig& case_config)
{
    const auto& entries = state_log.entries();
    const std::size_t n = entries.size();
    std::vector<KosNavballAttitude> out(n);
    std::vector<char> yaw_valid(n, 0);
    std::vector<char> pitch_valid(n, 0);
    const std::vector<double> phase_starts = configured_phase_start_times(case_config);

    constexpr double kRadToDeg = 57.295779513082320876798154814105;
    // |horizontal| below this => a reconstructed azimuth is numeric noise; carry
    // the last good one (only used for the non-poly azimuth fallback).
    constexpr double kHorizEps = 0.087;
    constexpr double kMinThrottle = 1.0e-3;

    for (std::size_t i = 0; i < n; ++i) {
        const auto& point = entries[i];

        // Commanded azimuth from the steering program (poly steering): explicit,
        // unambiguous, and never flips past vertical.
        const PhaseConfig* phase = phase_for_point(point, case_config);
        const double phase_t = point.time_s - phase_start_time_s(point, phase_starts);
        Poly2Config az;
        Poly2Config el;
        Poly2Config roll;
        const bool poly_azimuth = phase && steering_poly_for_time(*phase, phase_t, &az, &el, &roll);
        if (poly_azimuth) {
            double yaw = az.c0 + az.c1 * phase_t + az.c2 * phase_t * phase_t;
            while (yaw < 0.0) { yaw += 360.0; }
            while (yaw >= 360.0) { yaw -= 360.0; }
            out[i].yaw_deg = yaw;
            yaw_valid[i] = 1;
        }

        // Pitch (and, for non-poly phases, azimuth) come from the flown thrust
        // direction. Skip unsteered rows: pre-launch, hold-down and shutdown/coast
        // carry a default body axis, not a real command.
        if (point.throttle <= kMinThrottle || point.hold_down_clamp_active) {
            continue;
        }

        // UP = normalized position (fallback +X).
        double ux = point.state.position_m.x;
        double uy = point.state.position_m.y;
        double uz = point.state.position_m.z;
        double un = std::sqrt(ux * ux + uy * uy + uz * uz);
        if (un < 1.0e-9) { ux = 1.0; uy = 0.0; uz = 0.0; un = 1.0; }
        ux /= un; uy /= un; uz /= un;

        // EAST = normalize(Z x UP), Z = (0,0,1)  ->  (-uy, ux, 0) (fallback +Y).
        double ex = -uy, ey = ux, ez = 0.0;
        double en = std::sqrt(ex * ex + ey * ey + ez * ez);
        if (en < 1.0e-9) { ex = 0.0; ey = 1.0; ez = 0.0; en = 1.0; }
        ex /= en; ey /= en; ez /= en;

        // NORTH = UP x EAST (fallback +Z).
        double nx = uy * ez - uz * ey;
        double ny = uz * ex - ux * ez;
        double nz = ux * ey - uy * ex;
        double nn = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (nn < 1.0e-9) { nx = 0.0; ny = 0.0; nz = 1.0; nn = 1.0; }
        nx /= nn; ny /= nn; nz /= nn;

        // Commanded thrust direction (fallback to UP so pitch reads vertical).
        double dx = point.engine_direction_eci.x;
        double dy = point.engine_direction_eci.y;
        double dz = point.engine_direction_eci.z;
        double dn = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dn < 1.0e-9) { dx = ux; dy = uy; dz = uz; dn = 1.0; }
        dx /= dn; dy /= dn; dz /= dn;

        const double east_c = dx * ex + dy * ey + dz * ez;
        const double north_c = dx * nx + dy * ny + dz * nz;
        const double up_c = dx * ux + dy * uy + dz * uz;
        const double horizontal = std::sqrt(east_c * east_c + north_c * north_c);

        out[i].pitch_deg = std::atan2(up_c, horizontal) * kRadToDeg;
        pitch_valid[i] = 1;
        if (!poly_azimuth && horizontal > kHorizEps) {
            double yaw = std::atan2(east_c, north_c) * kRadToDeg;
            if (yaw < 0.0) {
                yaw += 360.0;
            }
            out[i].yaw_deg = yaw;
            yaw_valid[i] = 1;
        }
    }

    // Fill invalid samples (hold-down or near-vertical) from the nearest valid
    // one: leading invalids inherit the first good value; later invalids hold the
    // previous good value. With no valid sample at all, fall back to the defaults
    // (vertical, due east).
    auto carry_forward = [&](const std::vector<char>& valid, double KosNavballAttitude::* field, double fallback) {
        std::size_t first = n;
        for (std::size_t i = 0; i < n; ++i) {
            if (valid[i]) { first = i; break; }
        }
        if (first == n) {
            for (std::size_t i = 0; i < n; ++i) {
                out[i].*field = fallback;
            }
            return;
        }
        const double lead = out[first].*field;
        for (std::size_t i = 0; i < first; ++i) {
            out[i].*field = lead;
        }
        double last = lead;
        for (std::size_t i = first; i < n; ++i) {
            if (valid[i]) {
                last = out[i].*field;
            } else {
                out[i].*field = last;
            }
        }
    };
    carry_forward(pitch_valid, &KosNavballAttitude::pitch_deg, 90.0);
    carry_forward(yaw_valid, &KosNavballAttitude::yaw_deg, 90.0);
    return out;
}

double sx(double x_m, double scale, double center_x)
{
    return center_x + x_m * scale;
}

double sy(double y_m, double scale, double center_y)
{
    return center_y - y_m * scale;
}

} // namespace

std::string trajectory_to_csv(const StateLog& state_log)
{
    std::ostringstream output;
    output << std::setprecision(17);
    output << kTrajectoryCsvHeader << '\n';
    for (const auto& point : state_log.entries()) {
        write_trajectory_csv_row(output, point);
        output << '\n';
    }
    return output.str();
}

std::string kos_trajectory_to_csv(const StateLog& state_log, const CaseConfig& case_config)
{
    std::ostringstream output;
    output << std::setprecision(17);
    output << kTrajectoryCsvHeader
           << ",kos_phase_start_time_s,kos_phase_time_s,"
              "kos_steering_poly_available,"
              "kos_azimuth_c0,kos_azimuth_c1,kos_azimuth_c2,"
              "kos_elevation_c0,kos_elevation_c1,kos_elevation_c2,"
              "kos_roll_available,kos_roll_c0,kos_roll_c1,kos_roll_c2,"
              "kos_throttle_poly_available,"
              "kos_throttle_c0,kos_throttle_c1,kos_throttle_c2,"
              "kos_stage_command,kos_stage_plan_time_s,"
              "kos_stage_pulse_count,kos_shutdown_before_stage,"
              "kos_yaw_deg,kos_pitch_deg,kos_roll_deg,kos_roll_lock\n";

    const std::vector<double> phase_starts = configured_phase_start_times(case_config);
    const std::vector<KosNavballAttitude> navball = compute_kos_navball_attitude(state_log, case_config);
    const auto& entries = state_log.entries();
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const LaunchVehicleStateLogEntry& point = entries[i];
        const LaunchVehicleStateLogEntry* previous = (i > 0) ? &entries[i - 1] : nullptr;
        const double phase_start_s = phase_start_time_s(point, phase_starts);
        const double phase_time_s = point.time_s - phase_start_s;
        const PhaseConfig* phase = phase_for_point(point, case_config);

        Poly2Config azimuth;
        Poly2Config elevation;
        Poly2Config roll;
        const bool steering_poly_available =
            phase && steering_poly_for_time(*phase, phase_time_s, &azimuth, &elevation, &roll);
        const bool roll_poly_available = steering_poly_available && is_nonzero_poly(roll);
        const double roll_deg = roll_poly_available
            ? roll.c0 + roll.c1 * phase_time_s + roll.c2 * phase_time_s * phase_time_s
            : 0.0;

        Poly2Config throttle_poly;
        const bool throttle_poly_available =
            phase && throttle_poly_for_time(*phase, phase_time_s, &throttle_poly);
        double throttle_c0 = 0.0;
        double throttle_c1 = 0.0;
        double throttle_c2 = 0.0;
        if (throttle_poly_available) {
            throttle_c0 = throttle_poly.c0;
            throttle_c1 = throttle_poly.c1;
            throttle_c2 = throttle_poly.c2;
        }

        const bool stage_command = previous && detached_stage_since(*previous, point);
        const bool shutdown_before_stage = previous && stage_command && deactivated_stage_since(*previous, point);
        const bool ignition_after_separation = previous && stage_command && activated_stage_since(*previous, point);
        const int stage_pulse_count = stage_command
            ? (ignition_after_separation ? 2 : 1)
            : 0;
        double stage_plan_time_s = stage_command ? point.time_s : 0.0;
        if (stage_command && previous->phase_index != point.phase_index) {
            stage_plan_time_s = phase_start_s;
        }

        write_trajectory_csv_row(output, point);
        output
            << ',' << phase_start_s
            << ',' << phase_time_s
            << ',' << (steering_poly_available ? 1 : 0)
            << ',' << azimuth.c0
            << ',' << azimuth.c1
            << ',' << azimuth.c2
            << ',' << elevation.c0
            << ',' << elevation.c1
            << ',' << elevation.c2
            << ',' << (roll_poly_available ? 1 : 0)
            << ',' << roll.c0
            << ',' << roll.c1
            << ',' << roll.c2
            << ',' << (throttle_poly_available ? 1 : 0)
            << ',' << throttle_c0
            << ',' << throttle_c1
            << ',' << throttle_c2
            << ',' << (stage_command ? 1 : 0)
            << ',' << stage_plan_time_s
            << ',' << stage_pulse_count
            << ',' << (shutdown_before_stage ? 1 : 0)
            << ',' << navball[i].yaw_deg
            << ',' << navball[i].pitch_deg
            << ',' << roll_deg
            << ',' << (roll_poly_available ? 1 : 0)
            << '\n';
    }
    return output.str();
}

SimulationResult trajectory_from_csv(const std::string& csv)
{
    std::istringstream input(csv);
    std::string line;

    if (!std::getline(input, line)) {
        return {false, "empty CSV response", {}};
    }

    StateLog state_log(kEarthRadiusM);
    int line_number = 1;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.empty()) {
            continue;
        }

        const auto parts = split_csv_line(line);
        // Supported column counts:
        //   9          : legacy state-only (time + position + velocity + alt + speed)
        //   13         : adds total/prop mass + thrust + mass flow
        //   14         : 13 + hold_down_clamp
        //   15, 16     : 14 + phase_index (+ phase_name)
        //   23, 24     : current full schema (+ throttle, direction_eci, env, q, mach,
        //                hold_down_clamp, phase_index, [phase_name])
        if (parts.size() != 9 && parts.size() != 13 && parts.size() != 14 &&
            parts.size() != 15 && parts.size() != 16 &&
            parts.size() != 23 && parts.size() != 24) {
            return {false, "invalid CSV column count at line " + std::to_string(line_number), {}};
        }

        double values[23] = {};
        std::size_t numeric_parts = parts.size();
        if (parts.size() == 16) numeric_parts = 15;
        if (parts.size() == 24) numeric_parts = 23;
        for (std::size_t i = 0; i < numeric_parts; ++i) {
            if (!parse_double(parts[i], &values[i])) {
                return {false, "invalid CSV number at line " + std::to_string(line_number), {}};
            }
        }

        State state;
        state.position_m = {values[1], values[2], values[3]};
        state.velocity_mps = {values[4], values[5], values[6]};
        auto runtime = post2::vehicle::make_initial_runtime_state(state_log.vehicle_config(), state, values[0]);
        LaunchVehicleStateLogEntry entry;
        entry.time_s = values[0];
        entry.runtime = runtime;
        entry.state = state;
        entry.radius_m = norm(state.position_m);
        entry.altitude_m = values[7];
        entry.speed_mps = values[8];
        entry.total_mass_kg = runtime.vehicle.total_mass_kg;
        entry.propellant_mass_kg = runtime.vehicle.propellant_mass_kg;
        entry.engine_thrust_n = runtime.engine.actual_thrust_n;
        entry.engine_mass_flow_kgps = runtime.engine.mass_flow_kgps;
        entry.hold_down_clamp_active = runtime.hold_down_clamp.active;
        auto apply_mass_thrust = [&]() {
            entry.total_mass_kg = values[9];
            entry.propellant_mass_kg = values[10];
            entry.engine_thrust_n = values[11];
            entry.engine_mass_flow_kgps = values[12];
            entry.runtime.vehicle.total_mass_kg = values[9];
            entry.runtime.vehicle.propellant_mass_kg = values[10];
            entry.runtime.engine.actual_thrust_n = values[11];
            entry.runtime.engine.mass_flow_kgps = values[12];
        };
        if (parts.size() == 13) {
            apply_mass_thrust();
        } else if (parts.size() == 14) {
            apply_mass_thrust();
            entry.hold_down_clamp_active = values[13] != 0.0;
            entry.runtime.hold_down_clamp.active = entry.hold_down_clamp_active;
        } else if (parts.size() == 15 || parts.size() == 16) {
            apply_mass_thrust();
            entry.hold_down_clamp_active = values[13] != 0.0;
            entry.phase_index = static_cast<int>(values[14]);
            entry.phase_name = parts.size() == 16 ? parts[15] : "";
            entry.runtime.hold_down_clamp.active = entry.hold_down_clamp_active;
        } else if (parts.size() == 23 || parts.size() == 24) {
            apply_mass_thrust();
            entry.throttle = values[13];
            entry.engine_direction_eci = {values[14], values[15], values[16]};
            entry.ambient_pressure_pa = values[17];
            entry.atmosphere_density_kgpm3 = values[18];
            entry.dynamic_pressure_pa = values[19];
            entry.mach_number = values[20];
            entry.hold_down_clamp_active = values[21] != 0.0;
            entry.phase_index = static_cast<int>(values[22]);
            entry.phase_name = parts.size() == 24 ? parts[23] : "";
            entry.runtime.engine.throttle = entry.throttle;
            entry.runtime.engine.direction_body = entry.engine_direction_eci;
            entry.runtime.hold_down_clamp.active = entry.hold_down_clamp_active;
        }
        state_log.append(entry);
    }

    return {!state_log.empty(), state_log.empty() ? "CSV response has no points" : "", state_log};
}

bool write_csv_file(const std::string& path, const StateLog& state_log, std::string* error)
{
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        if (error) {
            *error = "failed to open CSV file: " + path;
        }
        return false;
    }

    output << trajectory_to_csv(state_log);
    return true;
}

bool write_kos_csv_file(
    const std::string& path,
    const StateLog& state_log,
    const CaseConfig& case_config,
    std::string* error)
{
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        if (error) {
            *error = "failed to open kOS CSV file: " + path;
        }
        return false;
    }

    output << kos_trajectory_to_csv(state_log, case_config);
    return true;
}

bool write_svg_file(
    const std::string& path,
    const StateLog& state_log,
    const StateLog* predicted_orbit,
    std::string* error)
{
    if (state_log.empty()) {
        if (error) {
            *error = "StateLog is empty";
        }
        return false;
    }

    std::ofstream output(path, std::ios::binary);
    if (!output) {
        if (error) {
            *error = "failed to open SVG file: " + path;
        }
        return false;
    }

    const bool has_predicted = predicted_orbit && !predicted_orbit->empty();

    constexpr int width = 1000;
    constexpr int height = 760;
    constexpr double margin = 70.0;
    const double center_x = width * 0.5;
    const double center_y = height * 0.52;
    const OrbitPlaneProjector projector = make_orbit_plane_projector(state_log);
    // The predicted orbit reaches the far side of Earth, so the drawing must
    // scale to whichever extent is larger (ascent or full orbit).
    double extent_m = max_abs_projected_xy(state_log, projector);
    if (has_predicted) {
        extent_m = std::max(extent_m, max_abs_projected_xy(*predicted_orbit, projector));
    }
    const double scale = (std::min(width, height) * 0.5 - margin) / extent_m;
    const double earth_radius_px = kEarthRadiusM * scale;

    output << std::fixed << std::setprecision(3);
    output << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
           << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
    output << "<rect width=\"100%\" height=\"100%\" fill=\"#f8fafc\"/>\n";
    output << "<line x1=\"" << margin << "\" y1=\"" << center_y << "\" x2=\"" << width - margin
           << "\" y2=\"" << center_y << "\" stroke=\"#cbd5e1\" stroke-width=\"1\"/>\n";
    output << "<line x1=\"" << center_x << "\" y1=\"" << margin << "\" x2=\"" << center_x
           << "\" y2=\"" << height - margin << "\" stroke=\"#cbd5e1\" stroke-width=\"1\"/>\n";
    output << "<circle cx=\"" << center_x << "\" cy=\"" << center_y << "\" r=\"" << earth_radius_px
           << "\" fill=\"#2563eb\" opacity=\"0.22\" stroke=\"#1d4ed8\" stroke-width=\"2\"/>\n";

    // Integrated predicted orbit (drawn behind the ascent), in teal and dashed
    // to set it apart from the solid red powered ascent.
    if (has_predicted) {
        output << "<polyline fill=\"none\" stroke=\"#0d9488\" stroke-width=\"2\" "
                  "stroke-dasharray=\"7 5\" points=\"";
        for (const auto& point : predicted_orbit->entries()) {
            const PlanePoint projected = projector.project(point.state.position_m);
            output << sx(projected.x_m, scale, center_x) << ','
                   << sy(projected.y_m, scale, center_y) << ' ';
        }
        output << "\"/>\n";
    }

    output << "<polyline fill=\"none\" stroke=\"#dc2626\" stroke-width=\"2.5\" points=\"";

    for (const auto& point : state_log.entries()) {
        const PlanePoint projected = projector.project(point.state.position_m);
        output << sx(projected.x_m, scale, center_x) << ','
               << sy(projected.y_m, scale, center_y) << ' ';
    }
    output << "\"/>\n";

    const auto& first = state_log.front();
    const auto& last = state_log.back();
    const PlanePoint projected_first = projector.project(first.state.position_m);
    const PlanePoint projected_last = projector.project(last.state.position_m);
    output << "<circle cx=\"" << sx(projected_first.x_m, scale, center_x)
           << "\" cy=\"" << sy(projected_first.y_m, scale, center_y)
           << "\" r=\"5\" fill=\"#16a34a\"/>\n";
    output << "<circle cx=\"" << sx(projected_last.x_m, scale, center_x)
           << "\" cy=\"" << sy(projected_last.y_m, scale, center_y)
           << "\" r=\"5\" fill=\"#111827\"/>\n";

    // Apoapsis / periapsis = the max / min geodetic-altitude points of the
    // integrated predicted orbit, marked with labels.
    double apoapsis_km = 0.0;
    double periapsis_km = 0.0;
    if (has_predicted) {
        const LaunchVehicleStateLogEntry* apo = nullptr;
        const LaunchVehicleStateLogEntry* peri = nullptr;
        for (const auto& point : predicted_orbit->entries()) {
            if (!apo || point.altitude_m > apo->altitude_m) {
                apo = &point;
            }
            if (!peri || point.altitude_m < peri->altitude_m) {
                peri = &point;
            }
        }
        if (apo && peri) {
            apoapsis_km = apo->altitude_m / 1000.0;
            periapsis_km = peri->altitude_m / 1000.0;
            auto mark = [&](const LaunchVehicleStateLogEntry& e, const char* fill, const char* label) {
                const PlanePoint p = projector.project(e.state.position_m);
                const double px = sx(p.x_m, scale, center_x);
                const double py = sy(p.y_m, scale, center_y);
                output << "<circle cx=\"" << px << "\" cy=\"" << py << "\" r=\"6\" fill=\""
                       << fill << "\" stroke=\"#ffffff\" stroke-width=\"1.5\"/>\n";
                output << "<text x=\"" << px + 9.0 << "\" y=\"" << py - 8.0
                       << "\" fill=\"" << fill
                       << "\" font-family=\"Segoe UI, Arial\" font-size=\"16\" font-weight=\"bold\">"
                       << label << "</text>\n";
            };
            mark(*apo, "#2563eb", "A");
            mark(*peri, "#db2777", "P");
        }
    }

    output << "<text x=\"32\" y=\"40\" fill=\"#0f172a\" font-family=\"Segoe UI, Arial\" font-size=\"22\">POST2 Lite trajectory</text>\n";
    output << "<text x=\"32\" y=\"70\" fill=\"#475569\" font-family=\"Segoe UI, Arial\" font-size=\"14\">"
           << "state log entries: " << state_log.size()
           << " | end t: " << last.time_s
           << " s | end altitude: " << last.altitude_m / 1000.0
           << " km | end speed: " << last.speed_mps
           << " m/s</text>\n";
    if (has_predicted) {
        output << "<text x=\"32\" y=\"92\" fill=\"#0d9488\" font-family=\"Segoe UI, Arial\" font-size=\"14\">"
               << "predicted orbit (integrated)  |  A apoapsis: " << apoapsis_km
               << " km  |  P periapsis: " << periapsis_km << " km</text>\n";
    }
    output << "</svg>\n";

    return true;
}

} // namespace post2::core
