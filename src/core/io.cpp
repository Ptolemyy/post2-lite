#include "post2/core/io.hpp"

#include "post2/core/projection.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace post2::core {

namespace {

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
    output << "time_s,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,altitude_m,speed_mps,"
              "total_mass_kg,propellant_mass_kg,engine_thrust_n,engine_mass_flow_kgps,"
              "throttle,engine_direction_eci_x,engine_direction_eci_y,engine_direction_eci_z,"
              "ambient_pressure_pa,atmosphere_density_kgpm3,dynamic_pressure_pa,mach_number,"
              "hold_down_clamp_active,phase_index,phase_name\n";
    for (const auto& point : state_log.entries()) {
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
            << point.phase_name << '\n';
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

bool write_svg_file(const std::string& path, const StateLog& state_log, std::string* error)
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

    constexpr int width = 1000;
    constexpr int height = 760;
    constexpr double margin = 70.0;
    const double center_x = width * 0.5;
    const double center_y = height * 0.52;
    const OrbitPlaneProjector projector = make_orbit_plane_projector(state_log);
    const double scale = (std::min(width, height) * 0.5 - margin) / max_abs_projected_xy(state_log, projector);
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

    output << "<text x=\"32\" y=\"40\" fill=\"#0f172a\" font-family=\"Segoe UI, Arial\" font-size=\"22\">POST2 Lite trajectory</text>\n";
    output << "<text x=\"32\" y=\"70\" fill=\"#475569\" font-family=\"Segoe UI, Arial\" font-size=\"14\">"
           << "state log entries: " << state_log.size()
           << " | end t: " << last.time_s
           << " s | end altitude: " << last.altitude_m / 1000.0
           << " km | end speed: " << last.speed_mps
           << " m/s</text>\n";
    output << "</svg>\n";

    return true;
}

} // namespace post2::core
