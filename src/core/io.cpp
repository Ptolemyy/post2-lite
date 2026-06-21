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
#include <utility>
#include <vector>

namespace post2::core {

namespace {

constexpr const char* kTrajectoryCsvHeader =
    "time_s,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,altitude_m,speed_mps,"
    "total_mass_kg,propellant_mass_kg,engine_thrust_n,engine_mass_flow_kgps,"
    "throttle,engine_direction_eci_x,engine_direction_eci_y,engine_direction_eci_z,"
    "ambient_pressure_pa,atmosphere_density_kgpm3,dynamic_pressure_pa,mach_number,"
    "hold_down_clamp_active,phase_index,rigid_body_attitude_rad,"
    "rigid_body_angular_velocity_radps,rigid_body_moment_of_inertia_kgm2,phase_name";

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
        << point.rigid_body_attitude_rad << ','
        << point.rigid_body_angular_velocity_radps << ','
        << point.rigid_body_moment_of_inertia_kgm2 << ','
        << point.phase_name;
}

// Standard gravity used to turn vacuum Isp into exhaust velocity, matching
// build_upfg_stages in control_models.cpp.
constexpr double kStandardGravityMps2 = 9.80665;

int sanitized_order(int order)
{
    return std::max(0, std::min(order, 8));
}

double coefficient_or_zero(const std::vector<double>& coefficients, std::size_t index)
{
    return index < coefficients.size() ? coefficients[index] : 0.0;
}

bool is_segmented_poly_type(const std::string& type)
{
    const std::string normalized = lowercase(type);
    return normalized == "segmented_poly" ||
        normalized == "generic_segmented_poly" ||
        normalized == "piecewise_poly";
}

bool is_tangent_type(const std::string& type)
{
    const std::string normalized = lowercase(type);
    return normalized == "linear_tangent" || normalized == "bilinear_tangent";
}

bool is_rpy_type(const std::string& type)
{
    const std::string normalized = lowercase(type);
    return normalized == "rpy_poly" || normalized == "roll_pitch_yaw_poly";
}

// Free-text fields (names) are written inside a comma-delimited record, so strip
// the delimiter and newlines to keep the row splittable on ','.
std::string sanitize_field(std::string text)
{
    for (char& ch : text) {
        if (ch == ',' || ch == '\n' || ch == '\r') {
            ch = ' ';
        }
    }
    return text;
}

double sx(double x_m, double scale, double center_x)
{
    return center_x + x_m * scale;
}

double sy(double y_m, double scale, double center_y)
{
    return center_y - y_m * scale;
}

std::string color_to_hex(ColorRgb color)
{
    color.red = std::clamp(color.red, 0, 255);
    color.green = std::clamp(color.green, 0, 255);
    color.blue = std::clamp(color.blue, 0, 255);

    std::ostringstream output;
    output << '#'
           << std::hex << std::setfill('0') << std::nouppercase
           << std::setw(2) << color.red
           << std::setw(2) << color.green
           << std::setw(2) << color.blue;
    return output.str();
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

std::string guidance_script_to_csv(const CaseConfig& case_config)
{
    std::ostringstream output;
    output << std::setprecision(17);

    // META: body + launch-site constants so the player needs no hardcoded
    // mu/radius (KSP body values flow through CaseConfig).
    output << "META," << sanitize_field(case_config.name)
           << ',' << case_config.earth_mu_m3s2
           << ',' << case_config.earth_radius_m
           << ',' << case_config.earth_rotation_rad_per_s
           << ',' << case_config.earth_rotation_at_epoch_rad
           << ',' << case_config.launch_site.latitude_deg
           << ',' << case_config.launch_site.longitude_deg
           << ',' << case_config.launch_site.altitude_m
           << '\n';

    // STAGE: one row per configured stage, in burn order, carrying the
    // propulsion params the player needs to rebuild UpfgStages (live total mass
    // comes from KSP; mirrors build_upfg_stages in control_models.cpp).
    for (std::size_t i = 0; i < case_config.vehicle.stages.size(); ++i) {
        const auto& stage = case_config.vehicle.stages[i];
        const double thrust_n =
            static_cast<double>(stage.engine.engine_count) * stage.engine.thrust_vac_n;
        const double exhaust_velocity_mps = stage.engine.isp_vac_s * kStandardGravityMps2;
        const double mdot_kgps =
            exhaust_velocity_mps > 0.0 ? thrust_n / exhaust_velocity_mps : 0.0;
        double propellant_kg = 0.0;
        for (const auto& tank : stage.tanks) {
            propellant_kg += tank.initial_kg;
        }
        output << "STAGE," << i
               << ',' << sanitize_field(stage.name)
               << ',' << thrust_n
               << ',' << exhaust_velocity_mps
               << ',' << mdot_kgps
               << ',' << propellant_kg
               << ',' << stage.dry_mass_kg
               << '\n';
    }

    for (std::size_t i = 0; i < case_config.phases.size(); ++i) {
        const PhaseConfig& phase = case_config.phases[i];
        const SteeringModelConfig& steering = phase.steering_model;
        const std::string steering_type = lowercase(steering.type);

        const std::string throttle_type = lowercase(phase.throttle_model.type);
        const bool poly_throttle = throttle_type == "poly";
        const double thr_c0 = poly_throttle ? phase.throttle_model.c0 : 0.0;
        const double thr_c1 = poly_throttle ? phase.throttle_model.c1 : 0.0;
        const double thr_c2 = poly_throttle ? phase.throttle_model.c2 : 0.0;

        output << "PHASE," << i
               << ',' << sanitize_field(phase.name)
               << ',' << steering_type
               << ',' << phase.termination.type
               << ',' << phase.termination.comparison
               << ',' << phase.termination.value
               << ',' << throttle_type
               << ',' << thr_c0 << ',' << thr_c1 << ',' << thr_c2
               << ',' << (phase.hold_down_clamp_initial_active ? 1 : 0)
               << '\n';

        if (steering_type == "upfg") {
            // The only steering data a UPFG phase needs: the orbit target.
            output << "UPFG," << i
                   << ',' << steering.upfg.periapsis_km
                   << ',' << steering.upfg.apoapsis_km
                   << ',' << steering.upfg.inclination_deg
                   << '\n';
        } else if (is_segmented_poly_type(steering_type)) {
            const int order = sanitized_order(steering.segmented_poly.order);
            for (const auto& segment : steering.segmented_poly.segments) {
                output << "SEG," << i << ',' << segment.start_time_s << ',' << order;
                for (int k = 0; k <= order; ++k) {
                    output << ',' << coefficient_or_zero(
                        segment.azimuth_coefficients, static_cast<std::size_t>(k));
                }
                for (int k = 0; k <= order; ++k) {
                    output << ',' << coefficient_or_zero(
                        segment.elevation_coefficients, static_cast<std::size_t>(k));
                }
                output << '\n';
            }
        } else if (is_tangent_type(steering_type)) {
            // Azimuth is still a poly; elevation follows the tangent law.
            const Poly2Config& az = steering.azimuth_deg;
            output << "POLY," << i
                   << ',' << az.c0 << ',' << az.c1 << ',' << az.c2
                   << ",0,0,0,0,0,0\n";
            const LinearTangentConfig& t = steering.tangent;
            output << "TAN," << i
                   << ',' << t.a << ',' << t.a_dot
                   << ',' << t.b << ',' << t.b_dot
                   << ',' << t.t_offset_s
                   << ',' << (steering_type == "bilinear_tangent" ? 1 : 0)
                   << '\n';
        } else {
            // generic_poly / rpy_poly / default: emit az/el/roll polynomials.
            const bool rpy = is_rpy_type(steering_type);
            const Poly2Config& az = rpy ? steering.yaw_deg : steering.azimuth_deg;
            const Poly2Config& el = rpy ? steering.pitch_deg : steering.elevation_deg;
            const Poly2Config& roll = steering.roll_deg;
            output << "POLY," << i
                   << ',' << az.c0 << ',' << az.c1 << ',' << az.c2
                   << ',' << el.c0 << ',' << el.c1 << ',' << el.c2
                   << ',' << roll.c0 << ',' << roll.c1 << ',' << roll.c2
                   << '\n';
        }

        for (const auto& action : phase.actions) {
            output << "ACTION," << i
                   << ',' << action.time_s
                   << ',' << action.type
                   << ',' << (action.value ? 1 : 0)
                   << ',' << action.stage_index
                   << ',' << sanitize_field(action.stage_name)
                   << '\n';
        }
    }

    for (const auto& event : case_config.events) {
        if (!event.enabled) {
            continue;
        }
        output << "EVENT," << sanitize_field(event.name)
               << ',' << event.trigger.type
               << ',' << event.trigger.comparison
               << ',' << event.trigger.value;
        for (const auto& action : event.actions) {
            output << ',' << action.time_s
                   << ',' << action.type
                   << ',' << (action.value ? 1 : 0)
                   << ',' << action.stage_index
                   << ',' << sanitize_field(action.stage_name);
        }
        output << '\n';
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
        //   23, 24     : full schema before rigid-body state columns.
        //   26, 27     : current full schema (+ rigid-body attitude/rate/inertia,
        //                [phase_name])
        if (parts.size() != 9 && parts.size() != 13 && parts.size() != 14 &&
            parts.size() != 15 && parts.size() != 16 &&
            parts.size() != 23 && parts.size() != 24 &&
            parts.size() != 26 && parts.size() != 27) {
            return {false, "invalid CSV column count at line " + std::to_string(line_number), {}};
        }

        double values[26] = {};
        std::size_t numeric_parts = parts.size();
        if (parts.size() == 16) numeric_parts = 15;
        if (parts.size() == 24) numeric_parts = 23;
        if (parts.size() == 27) numeric_parts = 26;
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
        } else if (parts.size() == 23 || parts.size() == 24 ||
                   parts.size() == 26 || parts.size() == 27) {
            apply_mass_thrust();
            entry.throttle = values[13];
            entry.engine_direction_eci = {values[14], values[15], values[16]};
            entry.ambient_pressure_pa = values[17];
            entry.atmosphere_density_kgpm3 = values[18];
            entry.dynamic_pressure_pa = values[19];
            entry.mach_number = values[20];
            entry.hold_down_clamp_active = values[21] != 0.0;
            entry.phase_index = static_cast<int>(values[22]);
            if (parts.size() == 26 || parts.size() == 27) {
                entry.rigid_body_attitude_rad = values[23];
                entry.rigid_body_angular_velocity_radps = values[24];
                entry.rigid_body_moment_of_inertia_kgm2 = values[25];
                entry.runtime.vehicle.rigid_body.attitude_rad = values[23];
                entry.runtime.vehicle.rigid_body.angular_velocity_radps = values[24];
                entry.runtime.vehicle.rigid_body.moment_of_inertia_kgm2 = values[25];
                entry.phase_name = parts.size() == 27 ? parts[26] : "";
            } else {
                entry.phase_name = parts.size() == 24 ? parts[23] : "";
            }
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

bool write_guidance_script_file(
    const std::string& path,
    const CaseConfig& case_config,
    std::string* error)
{
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        if (error) {
            *error = "failed to open guidance script file: " + path;
        }
        return false;
    }

    output << guidance_script_to_csv(case_config);
    return true;
}

bool write_svg_file(
    const std::string& path,
    const StateLog& state_log,
    const StateLog* predicted_orbit,
    std::string* error)
{
    std::vector<PredictedTrajectoryPath> predicted_paths;
    if (predicted_orbit && !predicted_orbit->empty()) {
        PredictedTrajectoryPath predicted;
        predicted.state_log = *predicted_orbit;
        predicted.source_phase_index = predicted_orbit->front().phase_index - 1;
        predicted.source_phase_name = "terminal";
        predicted.controller_name = "root stack";
        predicted.color = {13, 148, 136};
        predicted_paths.push_back(std::move(predicted));
    }
    return write_svg_file(path, state_log, predicted_paths, error);
}

bool write_svg_file(
    const std::string& path,
    const StateLog& state_log,
    const std::vector<PredictedTrajectoryPath>& predicted_paths,
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

    const PredictedTrajectoryPath* marker_prediction = nullptr;
    int predicted_count = 0;
    for (const auto& predicted : predicted_paths) {
        if (!predicted.state_log.empty()) {
            marker_prediction = &predicted;
            ++predicted_count;
        }
    }
    const bool has_predicted = marker_prediction != nullptr;

    constexpr int width = 1000;
    constexpr int height = 760;
    constexpr double margin = 70.0;
    const double center_x = width * 0.5;
    const double center_y = height * 0.52;
    const OrbitPlaneProjector projector = make_orbit_plane_projector(state_log);
    // The predicted orbit reaches the far side of Earth, so the drawing must
    // scale to whichever extent is larger (ascent or full orbit).
    double extent_m = max_abs_projected_xy(state_log, projector);
    for (const auto& predicted : predicted_paths) {
        if (!predicted.state_log.empty()) {
            extent_m = std::max(extent_m, max_abs_projected_xy(predicted.state_log, projector));
        }
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

    // Integrated phase-end predictions are drawn behind the ascent as dashed
    // paths, with one colour per controller.
    for (const auto& predicted : predicted_paths) {
        if (predicted.state_log.empty()) {
            continue;
        }
        output << "<polyline fill=\"none\" stroke=\"" << color_to_hex(predicted.color)
               << "\" stroke-width=\"2\" stroke-opacity=\"0.82\" "
                  "stroke-dasharray=\"7 5\" points=\"";
        for (const auto& point : predicted.state_log.entries()) {
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
        for (const auto& point : marker_prediction->state_log.entries()) {
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
               << "phase-end predictions: " << predicted_count
               << " dashed path(s)  |  A apoapsis: " << apoapsis_km
               << " km  |  P periapsis: " << periapsis_km << " km</text>\n";
        int legend_y = 116;
        std::vector<std::string> legend_keys;
        for (const auto& predicted : predicted_paths) {
            if (predicted.state_log.empty()) {
                continue;
            }
            const std::string key = predicted.controller_name + "|" + color_to_hex(predicted.color);
            if (std::find(legend_keys.begin(), legend_keys.end(), key) != legend_keys.end()) {
                continue;
            }
            legend_keys.push_back(key);
            const std::string color = color_to_hex(predicted.color);
            output << "<line x1=\"32\" y1=\"" << legend_y - 5
                   << "\" x2=\"64\" y2=\"" << legend_y - 5
                   << "\" stroke=\"" << color
                   << "\" stroke-width=\"2\" stroke-dasharray=\"7 5\"/>\n";
            output << "<text x=\"72\" y=\"" << legend_y
                   << "\" fill=\"" << color
                   << "\" font-family=\"Segoe UI, Arial\" font-size=\"12\">"
                   << sanitize_field(predicted.controller_name)
                   << "</text>\n";
            legend_y += 16;
            if (legend_y > 184) {
                break;
            }
        }
    }
    output << "</svg>\n";

    return true;
}

} // namespace post2::core
