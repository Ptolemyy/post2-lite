#include "post2/core/io.hpp"
#include "post2/core/case_config_io.hpp"
#include "post2/core/ksp_vehicle_site_import.hpp"
#include "post2/core/optimization.hpp"
#include "post2/core/simulation_driver.hpp"
#include "post2/core/trajectory_service.hpp"
#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle_config_io.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

enum class Command {
    Run,
    Optimize,
};

struct PhaseOverride {
    int index = -1;
    bool enabled = true;
};

struct Options {
    Command command = Command::Run;
    post2::core::CoreMode mode = post2::core::CoreMode::Local;
    std::string host = "127.0.0.1";
    int port = 5050;
    post2::core::SimulationConfig config;
    post2::core::CaseConfig case_config;
    bool has_case_config = false;
    post2::core::OptimizationConfig optimization_overrides;
    bool has_optimization_overrides = false;
    bool optimizer_set = false;
    bool qp_solver_set = false;
    bool fd_mode_set = false;
    bool parallel_fd_set = false;
    bool max_iterations_set = false;
    bool tolerance_set = false;
    bool constraint_tolerance_set = false;
    bool stationarity_tolerance_set = false;
    bool max_restoration_iterations_set = false;
    bool initial_step_fraction_set = false;
    bool objective_set = false;
    std::vector<PhaseOverride> phase_overrides;
    std::string csv_path = "trajectory.csv";
    std::string svg_path = "trajectory.svg";
    std::string guidance_csv_path;
    std::string save_vehicle_config_path;
    std::string save_case_config_path;
    bool write_csv = true;
    bool write_svg = true;
};

void print_help()
{
    std::cout
        << "Usage: post2_shell run [options]\n"
        << "       post2_shell simulate [options]\n"
        << "       post2_shell optimize --case input.json --save-case optimized.json [options]\n\n"
        << "Options:\n"
        << "  --mode local|remote       Core execution mode (default: local)\n"
        << "  --host HOST               Remote host (default: 127.0.0.1)\n"
        << "  --port PORT               Remote port (default: 5050)\n"
        << "  --duration SECONDS        Propagation duration (default: 5400)\n"
        << "  --dt SECONDS              ODE step size (default: 10)\n"
        << "  --altitude METERS         Initial circular LEO altitude (default: 200000)\n"
        << "  --inclination DEGREES     Initial orbital plane inclination (default: 28.5)\n"
        << "  --speed MPS               Initial speed; <=0 uses circular speed\n"
        << "  --launch-lat DEGREES      Launch site latitude for ground/HDC runs\n"
        << "  --launch-lon DEGREES      Launch site longitude for ground/HDC runs\n"
        << "  --launch-altitude METERS  Launch site altitude above spherical Earth\n"
        << "  --hdc-enabled true|false  Enable Hold Down Clamp before release\n"
        << "  --hdc-release-time SEC    Hold Down Clamp release time\n"
        << "  --normal-force-enabled true|false\n"
        << "                            Enable surface normal contact force\n"
        << "  --case PATH               Load a case JSON file\n"
        << "  --save-case PATH          Save the current case JSON to PATH\n"
        << "  --vehicle-config PATH     Load vehicle config key-value text file\n"
        << "  --import-ksp-vehicle-site PATH\n"
        << "                            Import vehicle, payload, launch site, and body constants from kOS JSON\n"
        << "  --save-vehicle-config PATH\n"
        << "                            Save the current vehicle config to PATH\n"
        << "  --vehicle-name NAME       Override vehicle name\n"
        << "  --dry-mass KG             Override vehicle dry mass\n"
        << "  --engine-enabled true|false\n"
        << "                            Enable or disable engine config\n"
        << "  --engine-max-thrust N     Override max engine thrust in newtons\n"
        << "  --engine-isp S            Override engine Isp in seconds\n"
        << "  --engine-dir-x VALUE      Override engine body direction X\n"
        << "  --engine-dir-y VALUE      Override engine body direction Y\n"
        << "  --engine-dir-z VALUE      Override engine body direction Z\n"
        << "  --aero-enabled true|false Enable or disable constant-Cd aero drag\n"
        << "  --reference-area M2       Override aero reference area\n"
        << "  --cd VALUE                Override drag coefficient\n"
        << "  --cl VALUE                Override lift coefficient (stored only in v1)\n"
        << "  --aero-table PATH         Store optional aero table path\n"
        << "  --tank-name NAME          Override first tank name\n"
        << "  --tank-propellant NAME    Override first tank propellant\n"
        << "  --tank-capacity KG        Override first tank capacity\n"
        << "  --tank-initial KG         Override first tank initial mass\n"
        << "  --csv PATH                Export CSV path (default: trajectory.csv)\n"
        << "  --svg PATH                Export SVG chart path (default: trajectory.svg)\n"
        << "  --guidance-csv PATH       Export guidance script CSV for post2_player (per-phase\n"
        << "                            poly params; UPFG phases marked + orbit target).\n"
        << "                            Alias: --kos-csv\n"
        << "  --no-csv                  Do not write CSV\n"
        << "  --no-svg                  Do not write SVG\n\n"
        << "Optimize options:\n"
        << "  --optimizer fmincon|sqp\n"
        << "  --qp-solver active-set|kkt-fallback\n"
        << "  --fd-mode forward|central|auto\n"
        << "  --parallel-fd true|false\n"
        << "  --max-iterations N        Override optimizer simulation budget\n"
        << "  --tolerance X             Override KKT / constraint tolerance\n"
        << "  --constraint-tolerance X  Override feasibility tolerance\n"
        << "  --stationarity-tolerance X\n"
        << "                            Override stationarity tolerance\n"
        << "  --max-restoration-iterations N\n"
        << "                            Override SQP restoration budget\n"
        << "  --initial-step-fraction X Override finite-difference base step fraction\n"
        << "  --target metric=value     Add equality target\n"
        << "  --target-range metric=min:max\n"
        << "                            Add range target\n"
        << "  --objective minimize:metric|maximize:metric\n"
        << "                            Add objective and switch to optimize mode\n"
        << "  --var path=min:max        Enable variable and bounds\n"
        << "  --phase INDEX=true|false  Enable/disable phase participation\n\n"
        << "Remote mode expects post2_core_server to be running.\n";
}

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

bool parse_int(const std::string& text, int* value)
{
    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        *value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_bool(const std::string& text, bool* value)
{
    if (text == "true" || text == "1" || text == "yes" || text == "on") {
        *value = true;
        return true;
    }
    if (text == "false" || text == "0" || text == "no" || text == "off") {
        *value = false;
        return true;
    }
    return false;
}

bool split_once(const std::string& text, char delimiter, std::string* lhs, std::string* rhs)
{
    const std::size_t position = text.find(delimiter);
    if (position == std::string::npos) {
        return false;
    }
    *lhs = text.substr(0, position);
    *rhs = text.substr(position + 1);
    return true;
}

bool parse_target_arg(const std::string& text, post2::core::OptimizationTargetConfig* target)
{
    std::string metric;
    std::string value_text;
    if (!split_once(text, '=', &metric, &value_text) || metric.empty()) {
        return false;
    }
    double value = 0.0;
    if (!parse_double(value_text, &value)) {
        return false;
    }
    target->metric = metric;
    target->mode = "equal";
    target->value = value;
    return true;
}

bool parse_target_range_arg(const std::string& text, post2::core::OptimizationTargetConfig* target)
{
    std::string metric;
    std::string range_text;
    std::string min_text;
    std::string max_text;
    if (!split_once(text, '=', &metric, &range_text) ||
        !split_once(range_text, ':', &min_text, &max_text) ||
        metric.empty()) {
        return false;
    }
    double min_value = 0.0;
    double max_value = 0.0;
    if (!parse_double(min_text, &min_value) || !parse_double(max_text, &max_value)) {
        return false;
    }
    target->metric = metric;
    target->mode = "range";
    target->min_value = min_value;
    target->max_value = max_value;
    return true;
}

bool parse_objective_arg(const std::string& text, post2::core::OptimizationObjectiveConfig* objective)
{
    std::string direction;
    std::string metric;
    if (!split_once(text, ':', &direction, &metric) || metric.empty()) {
        return false;
    }
    if (direction != "minimize" && direction != "maximize") {
        return false;
    }
    objective->enabled = true;
    objective->direction = direction;
    objective->metric = metric;
    return true;
}

bool parse_var_arg(const std::string& text, post2::core::OptimizationVariableConfig* variable)
{
    std::string path;
    std::string bounds;
    std::string min_text;
    std::string max_text;
    if (!split_once(text, '=', &path, &bounds) ||
        !split_once(bounds, ':', &min_text, &max_text) ||
        path.empty()) {
        return false;
    }
    double min_value = 0.0;
    double max_value = 0.0;
    if (!parse_double(min_text, &min_value) || !parse_double(max_text, &max_value)) {
        return false;
    }
    variable->path = path;
    variable->enabled = true;
    variable->min_value = min_value;
    variable->max_value = max_value;
    return true;
}

bool parse_phase_arg(const std::string& text, PhaseOverride* override_value)
{
    std::string index_text;
    std::string enabled_text;
    if (!split_once(text, '=', &index_text, &enabled_text)) {
        return false;
    }
    int index = -1;
    bool enabled = false;
    if (!parse_int(index_text, &index) || !parse_bool(enabled_text, &enabled)) {
        return false;
    }
    override_value->index = index;
    override_value->enabled = enabled;
    return true;
}

post2::vehicle::TankConfig& first_tank(post2::vehicle::VehicleConfig& config)
{
    if (config.tanks.empty()) {
        config.tanks.push_back(post2::vehicle::TankConfig{});
    }
    return config.tanks.front();
}

bool require_value(int argc, char** argv, int* index, std::string* value)
{
    if (*index + 1 >= argc) {
        std::cerr << "Missing value for " << argv[*index] << '\n';
        return false;
    }
    *value = argv[++(*index)];
    return true;
}

bool parse_options(int argc, char** argv, Options* options)
{
    int i = 1;
    if (i < argc) {
        const std::string command = argv[i];
        if (command == "simulate" || command == "run") {
            options->command = Command::Run;
            ++i;
        } else if (command == "optimize") {
            options->command = Command::Optimize;
            options->optimization_overrides.mode = "target";
            ++i;
        } else if (!command.empty() && command.front() != '-') {
            std::cerr << "Unknown command: " << command << '\n';
            return false;
        }
    }

    for (; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string value;

        if (arg == "--help" || arg == "-h") {
            print_help();
            std::exit(0);
        }
        if (arg == "--no-csv") {
            options->write_csv = false;
            continue;
        }
        if (arg == "--no-svg") {
            options->write_svg = false;
            continue;
        }

        if (!require_value(argc, argv, &i, &value)) {
            return false;
        }

        if (arg == "--mode") {
            if (!post2::core::parse_core_mode(value, &options->mode)) {
                std::cerr << "Invalid --mode value: " << value << '\n';
                return false;
            }
        } else if (arg == "--host") {
            options->host = value;
        } else if (arg == "--port") {
            if (!parse_int(value, &options->port)) {
                std::cerr << "Invalid --port value: " << value << '\n';
                return false;
            }
        } else if (arg == "--duration") {
            if (!parse_double(value, &options->config.duration_s)) {
                std::cerr << "Invalid --duration value: " << value << '\n';
                return false;
            }
        } else if (arg == "--dt") {
            if (!parse_double(value, &options->config.step_s)) {
                std::cerr << "Invalid --dt value: " << value << '\n';
                return false;
            }
        } else if (arg == "--altitude") {
            if (!parse_double(value, &options->config.initial_altitude_m)) {
                std::cerr << "Invalid --altitude value: " << value << '\n';
                return false;
            }
        } else if (arg == "--inclination") {
            if (!parse_double(value, &options->config.inclination_deg)) {
                std::cerr << "Invalid --inclination value: " << value << '\n';
                return false;
            }
        } else if (arg == "--speed") {
            if (!parse_double(value, &options->config.initial_speed_mps)) {
                std::cerr << "Invalid --speed value: " << value << '\n';
                return false;
            }
        } else if (arg == "--launch-lat") {
            if (!parse_double(value, &options->config.launch_site.latitude_deg)) {
                std::cerr << "Invalid --launch-lat value: " << value << '\n';
                return false;
            }
        } else if (arg == "--launch-lon") {
            if (!parse_double(value, &options->config.launch_site.longitude_deg)) {
                std::cerr << "Invalid --launch-lon value: " << value << '\n';
                return false;
            }
        } else if (arg == "--launch-altitude") {
            if (!parse_double(value, &options->config.launch_site.altitude_m)) {
                std::cerr << "Invalid --launch-altitude value: " << value << '\n';
                return false;
            }
        } else if (arg == "--hdc-enabled") {
            if (!parse_bool(value, &options->config.hold_down_clamp.enabled)) {
                std::cerr << "Invalid --hdc-enabled value: " << value << '\n';
                return false;
            }
        } else if (arg == "--hdc-release-time") {
            if (!parse_double(value, &options->config.hold_down_clamp.release_time_s)) {
                std::cerr << "Invalid --hdc-release-time value: " << value << '\n';
                return false;
            }
        } else if (arg == "--normal-force-enabled") {
            if (!parse_bool(value, &options->config.normal_force.enabled)) {
                std::cerr << "Invalid --normal-force-enabled value: " << value << '\n';
                return false;
            }
        } else if (arg == "--case") {
            std::string error;
            if (!post2::core::load_case_config_file(value, &options->case_config, &error)) {
                std::cerr << "Case load failed: " << error << '\n';
                return false;
            }
            options->config = post2::core::simulation_config_from_case(options->case_config);
            options->has_case_config = true;
        } else if (arg == "--save-case") {
            options->save_case_config_path = value;
        } else if (arg == "--optimizer") {
            options->optimization_overrides.optimizer = value;
            options->has_optimization_overrides = true;
            options->optimizer_set = true;
        } else if (arg == "--qp-solver") {
            options->optimization_overrides.qp_solver = value;
            options->has_optimization_overrides = true;
            options->qp_solver_set = true;
        } else if (arg == "--fd-mode") {
            options->optimization_overrides.fd_mode = value;
            options->has_optimization_overrides = true;
            options->fd_mode_set = true;
        } else if (arg == "--parallel-fd") {
            if (!parse_bool(value, &options->optimization_overrides.parallel_fd)) {
                std::cerr << "Invalid --parallel-fd value: " << value << '\n';
                return false;
            }
            options->has_optimization_overrides = true;
            options->parallel_fd_set = true;
        } else if (arg == "--max-iterations") {
            if (!parse_int(value, &options->optimization_overrides.max_iterations)) {
                std::cerr << "Invalid --max-iterations value: " << value << '\n';
                return false;
            }
            options->has_optimization_overrides = true;
            options->max_iterations_set = true;
        } else if (arg == "--tolerance") {
            if (!parse_double(value, &options->optimization_overrides.tolerance)) {
                std::cerr << "Invalid --tolerance value: " << value << '\n';
                return false;
            }
            options->has_optimization_overrides = true;
            options->tolerance_set = true;
        } else if (arg == "--constraint-tolerance") {
            if (!parse_double(value, &options->optimization_overrides.constraint_tolerance)) {
                std::cerr << "Invalid --constraint-tolerance value: " << value << '\n';
                return false;
            }
            options->has_optimization_overrides = true;
            options->constraint_tolerance_set = true;
        } else if (arg == "--stationarity-tolerance") {
            if (!parse_double(value, &options->optimization_overrides.stationarity_tolerance)) {
                std::cerr << "Invalid --stationarity-tolerance value: " << value << '\n';
                return false;
            }
            options->has_optimization_overrides = true;
            options->stationarity_tolerance_set = true;
        } else if (arg == "--max-restoration-iterations") {
            if (!parse_int(value, &options->optimization_overrides.max_restoration_iterations)) {
                std::cerr << "Invalid --max-restoration-iterations value: " << value << '\n';
                return false;
            }
            options->has_optimization_overrides = true;
            options->max_restoration_iterations_set = true;
        } else if (arg == "--initial-step-fraction") {
            if (!parse_double(value, &options->optimization_overrides.initial_step_fraction)) {
                std::cerr << "Invalid --initial-step-fraction value: " << value << '\n';
                return false;
            }
            options->has_optimization_overrides = true;
            options->initial_step_fraction_set = true;
        } else if (arg == "--target") {
            post2::core::OptimizationTargetConfig target;
            if (!parse_target_arg(value, &target)) {
                std::cerr << "Invalid --target value: " << value << '\n';
                return false;
            }
            options->optimization_overrides.targets.push_back(std::move(target));
            options->has_optimization_overrides = true;
        } else if (arg == "--target-range") {
            post2::core::OptimizationTargetConfig target;
            if (!parse_target_range_arg(value, &target)) {
                std::cerr << "Invalid --target-range value: " << value << '\n';
                return false;
            }
            options->optimization_overrides.targets.push_back(std::move(target));
            options->has_optimization_overrides = true;
        } else if (arg == "--objective") {
            if (!parse_objective_arg(value, &options->optimization_overrides.objective)) {
                std::cerr << "Invalid --objective value: " << value << '\n';
                return false;
            }
            options->optimization_overrides.mode = "optimize";
            options->has_optimization_overrides = true;
            options->objective_set = true;
        } else if (arg == "--var") {
            post2::core::OptimizationVariableConfig variable;
            if (!parse_var_arg(value, &variable)) {
                std::cerr << "Invalid --var value: " << value << '\n';
                return false;
            }
            options->optimization_overrides.variables.push_back(std::move(variable));
            options->has_optimization_overrides = true;
        } else if (arg == "--phase") {
            PhaseOverride phase;
            if (!parse_phase_arg(value, &phase)) {
                std::cerr << "Invalid --phase value: " << value << '\n';
                return false;
            }
            options->phase_overrides.push_back(phase);
        } else if (arg == "--vehicle-config") {
            std::string error;
            if (!post2::vehicle::load_vehicle_config_file(value, &options->config.vehicle, &error)) {
                std::cerr << "Vehicle config load failed: " << error << '\n';
                return false;
            }
        } else if (arg == "--import-ksp-vehicle-site") {
            std::string error;
            const auto preserved_aero = options->has_case_config
                ? options->case_config.vehicle.aero
                : options->config.vehicle.aero;
            post2::core::KspVehicleSiteImport imported;
            if (!post2::core::load_ksp_vehicle_site_import_file(
                    value,
                    preserved_aero,
                    &imported,
                    &error)) {
                std::cerr << "KSP vehicle/site import failed: " << error << '\n';
                return false;
            }
            post2::core::apply_ksp_vehicle_site_import(&options->config, imported);
            if (options->has_case_config) {
                post2::core::apply_ksp_vehicle_site_import(&options->case_config, imported);
            }
        } else if (arg == "--save-vehicle-config") {
            options->save_vehicle_config_path = value;
        } else if (arg == "--vehicle-name") {
            options->config.vehicle.name = value;
        } else if (arg == "--dry-mass") {
            if (!parse_double(value, &options->config.vehicle.dry_mass_kg)) {
                std::cerr << "Invalid --dry-mass value: " << value << '\n';
                return false;
            }
        } else if (arg == "--rigid-body-inertia") {
            if (!parse_double(value, &options->config.vehicle.rigid_body.moment_of_inertia_kgm2)) {
                std::cerr << "Invalid --rigid-body-inertia value: " << value << '\n';
                return false;
            }
        } else if (arg == "--rigid-body-attitude") {
            if (!parse_double(value, &options->config.vehicle.rigid_body.initial_attitude_rad)) {
                std::cerr << "Invalid --rigid-body-attitude value: " << value << '\n';
                return false;
            }
        } else if (arg == "--rigid-body-omega") {
            if (!parse_double(value, &options->config.vehicle.rigid_body.initial_angular_velocity_radps)) {
                std::cerr << "Invalid --rigid-body-omega value: " << value << '\n';
                return false;
            }
        } else if (arg == "--engine-moment-arm") {
            if (!parse_double(value, &options->config.vehicle.rigid_body.engine_moment_arm_m)) {
                std::cerr << "Invalid --engine-moment-arm value: " << value << '\n';
                return false;
            }
        } else if (arg == "--engine-enabled" || arg == "--thrust-enabled") {
            if (!parse_bool(value, &options->config.vehicle.engine.enabled)) {
                std::cerr << "Invalid --engine-enabled value: " << value << '\n';
                return false;
            }
        } else if (arg == "--engine-max-thrust" || arg == "--thrust-max") {
            if (!parse_double(value, &options->config.vehicle.engine.thrust_vac_n)) {
                std::cerr << "Invalid --engine-max-thrust value: " << value << '\n';
                return false;
            }
        } else if (arg == "--engine-isp" || arg == "--thrust-isp") {
            if (!parse_double(value, &options->config.vehicle.engine.isp_vac_s)) {
                std::cerr << "Invalid --engine-isp value: " << value << '\n';
                return false;
            }
        } else if (arg == "--engine-dir-x" || arg == "--thrust-dir-x") {
            if (!parse_double(value, &options->config.vehicle.engine.direction_body.x)) {
                std::cerr << "Invalid --engine-dir-x value: " << value << '\n';
                return false;
            }
        } else if (arg == "--engine-dir-y" || arg == "--thrust-dir-y") {
            if (!parse_double(value, &options->config.vehicle.engine.direction_body.y)) {
                std::cerr << "Invalid --engine-dir-y value: " << value << '\n';
                return false;
            }
        } else if (arg == "--engine-dir-z" || arg == "--thrust-dir-z") {
            if (!parse_double(value, &options->config.vehicle.engine.direction_body.z)) {
                std::cerr << "Invalid --engine-dir-z value: " << value << '\n';
                return false;
            }
        } else if (arg == "--aero-enabled") {
            if (!parse_bool(value, &options->config.vehicle.aero.enabled)) {
                std::cerr << "Invalid --aero-enabled value: " << value << '\n';
                return false;
            }
        } else if (arg == "--reference-area") {
            if (!parse_double(value, &options->config.vehicle.aero.reference_area_m2)) {
                std::cerr << "Invalid --reference-area value: " << value << '\n';
                return false;
            }
        } else if (arg == "--cd") {
            if (!parse_double(value, &options->config.vehicle.aero.cd)) {
                std::cerr << "Invalid --cd value: " << value << '\n';
                return false;
            }
        } else if (arg == "--cl") {
            if (!parse_double(value, &options->config.vehicle.aero.cl)) {
                std::cerr << "Invalid --cl value: " << value << '\n';
                return false;
            }
        } else if (arg == "--aero-table") {
            options->config.vehicle.aero.aero_table_path = value;
        } else if (arg == "--tank-name") {
            first_tank(options->config.vehicle).name = value;
        } else if (arg == "--tank-propellant") {
            first_tank(options->config.vehicle).propellant = value;
        } else if (arg == "--tank-capacity") {
            if (!parse_double(value, &first_tank(options->config.vehicle).capacity_kg)) {
                std::cerr << "Invalid --tank-capacity value: " << value << '\n';
                return false;
            }
        } else if (arg == "--tank-initial") {
            if (!parse_double(value, &first_tank(options->config.vehicle).initial_kg)) {
                std::cerr << "Invalid --tank-initial value: " << value << '\n';
                return false;
            }
        } else if (arg == "--csv") {
            options->csv_path = value;
            options->write_csv = true;
        } else if (arg == "--svg") {
            options->svg_path = value;
            options->write_svg = true;
        } else if (arg == "--kos-csv" || arg == "--guidance-csv") {
            options->guidance_csv_path = value;
        } else {
            std::cerr << "Unknown argument: " << arg << '\n';
            return false;
        }
    }

    return true;
}

void sync_loaded_case_from_legacy_surface(Options* options)
{
    if (!options->has_case_config) {
        return;
    }
    options->case_config.vehicle = options->config.vehicle;
    if (options->case_config.vehicle.stages.empty()) {
        options->case_config.vehicle.stages =
            post2::vehicle::effective_stage_configs(options->case_config.vehicle);
    } else {
        options->case_config.vehicle.stages.front().engine = options->case_config.vehicle.engine;
        options->case_config.vehicle.stages.front().tanks = options->case_config.vehicle.tanks;
    }
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&options->case_config.vehicle);
    options->case_config.launch_site = options->config.launch_site;
    options->case_config.step_s = options->config.step_s;
    if (!options->case_config.phases.empty()) {
        // Mirror the legacy --duration only onto a time-based first phase. A case
        // that deliberately uses a non-time first-phase termination (e.g. a pad
        // hold that ends when thrust is established) is preserved as authored.
        if (options->case_config.phases.front().termination.type == "time") {
            options->case_config.phases.front().termination =
                {"time", ">=", options->config.duration_s};
        }
        options->case_config.phases.front().force_models.normal_force = options->config.normal_force.enabled;
        options->case_config.phases.front().force_models.aerodynamic =
            options->config.vehicle.aero.enabled;
    }
}

bool prepare_optimization_case(Options* options)
{
    if (options->command != Command::Optimize) {
        return true;
    }

    if (!options->has_case_config) {
        options->case_config = post2::core::case_from_simulation_config(options->config);
        options->has_case_config = true;
    }

    auto& optimization = options->case_config.optimization;
    if (options->optimizer_set) {
        optimization.optimizer = options->optimization_overrides.optimizer;
    }
    if (options->qp_solver_set) {
        optimization.qp_solver = options->optimization_overrides.qp_solver;
    }
    if (options->fd_mode_set) {
        optimization.fd_mode = options->optimization_overrides.fd_mode;
    }
    if (options->parallel_fd_set) {
        optimization.parallel_fd = options->optimization_overrides.parallel_fd;
    }
    if (options->max_iterations_set) {
        optimization.max_iterations = options->optimization_overrides.max_iterations;
    }
    if (options->tolerance_set) {
        optimization.tolerance = options->optimization_overrides.tolerance;
    }
    if (options->constraint_tolerance_set) {
        optimization.constraint_tolerance = options->optimization_overrides.constraint_tolerance;
    }
    if (options->stationarity_tolerance_set) {
        optimization.stationarity_tolerance = options->optimization_overrides.stationarity_tolerance;
    }
    if (options->max_restoration_iterations_set) {
        optimization.max_restoration_iterations =
            options->optimization_overrides.max_restoration_iterations;
    }
    if (options->initial_step_fraction_set) {
        optimization.initial_step_fraction = options->optimization_overrides.initial_step_fraction;
    }
    if (!options->optimization_overrides.targets.empty()) {
        optimization.targets.insert(
            optimization.targets.end(),
            options->optimization_overrides.targets.begin(),
            options->optimization_overrides.targets.end());
    }
    if (!options->optimization_overrides.variables.empty()) {
        optimization.variables.insert(
            optimization.variables.end(),
            options->optimization_overrides.variables.begin(),
            options->optimization_overrides.variables.end());
    }
    if (options->objective_set) {
        optimization.objective = options->optimization_overrides.objective;
        optimization.objectives.clear();
        optimization.objectives.push_back(options->optimization_overrides.objective);
        optimization.mode = "optimize";
    }

    for (const auto& phase : options->phase_overrides) {
        if (phase.index < 0 || static_cast<std::size_t>(phase.index) >= options->case_config.phases.size()) {
            std::cerr << "Invalid --phase index: " << phase.index << '\n';
            return false;
        }
        options->case_config.phases[static_cast<std::size_t>(phase.index)].optimize_enabled = phase.enabled;
    }

    return true;
}

void print_run_summary(
    post2::core::CoreMode mode,
    const post2::core::SimulationResult& result,
    const post2::core::CaseConfig* case_config,
    const post2::core::SimulationConfig& config,
    const post2::vehicle::VehicleConfig& active_vehicle)
{
    const auto& last = result.state_log.back();
    const auto launch_site = case_config ? case_config->launch_site : config.launch_site;
    std::cout
        << "mode: " << post2::core::core_mode_name(mode) << '\n'
        << "state_log_entries: " << result.state_log.size() << '\n'
        << "end_time_s: " << last.time_s << '\n'
        << "end_altitude_km: " << last.altitude_m / 1000.0 << '\n'
        << "end_speed_mps: " << last.speed_mps << '\n'
        << "payload_mass_kg: " << post2::vehicle::payload_stage_dry_mass_kg(active_vehicle) << '\n'
        << "final_total_mass_kg: " << last.total_mass_kg << '\n'
        << "final_propellant_kg: " << last.propellant_mass_kg << '\n'
        << "final_engine_thrust_n: " << last.engine_thrust_n << '\n'
        << "final_hold_down_clamp_active: " << (last.hold_down_clamp_active ? "true" : "false") << '\n'
        << "launch_site: lat " << launch_site.latitude_deg
        << " deg | lon " << launch_site.longitude_deg
        << " deg | alt " << launch_site.altitude_m << " m\n"
        << "hold_down_clamp: " << (config.hold_down_clamp.enabled ? "enabled" : "disabled")
        << " | release " << config.hold_down_clamp.release_time_s << " s\n"
        << "vehicle: " << post2::vehicle::vehicle_config_summary(active_vehicle) << '\n';
}

void print_metric_summary(const std::vector<post2::core::OptimizationMetricValue>& metrics)
{
    for (const auto& metric : metrics) {
        std::cout << "metric." << metric.metric << ": " << metric.value << '\n';
    }
}

void print_optimization_summary(const post2::core::OptimizationResult& result)
{
    std::cout
        << "optimization_ok: " << (result.ok ? "true" : "false") << '\n'
        << "found_feasible: " << (result.found_feasible ? "true" : "false") << '\n'
        << "iterations: " << result.iterations << '\n'
        << "evaluations: " << result.evaluations << '\n'
        << "best_score: " << result.best_score << '\n'
        << "max_constraint_violation: " << result.max_constraint_violation << '\n'
        << "l1_constraint_violation: " << result.l1_constraint_violation << '\n';
    for (const auto& change : result.variable_changes) {
        std::cout << "variable: " << change.path
                  << " | " << change.old_value
                  << " -> " << change.new_value << '\n';
    }
    print_metric_summary(result.final_metrics);
    for (const auto& message : result.messages) {
        std::cout << "message: " << message << '\n';
    }
}

bool write_exports(
    const Options& options,
    const post2::core::SimulationResult& result)
{
    std::string error;
    if (options.write_csv) {
        if (!post2::core::write_csv_file(options.csv_path, result.state_log, &error)) {
            std::cerr << "CSV export failed: " << error << '\n';
            return false;
        }
        std::cout << "wrote_csv: " << options.csv_path << '\n';
    }

    if (!options.guidance_csv_path.empty()) {
        const post2::core::CaseConfig guidance_case = options.has_case_config
            ? options.case_config
            : post2::core::case_from_simulation_config(options.config);
        if (!post2::core::write_guidance_script_file(options.guidance_csv_path, guidance_case, &error)) {
            std::cerr << "guidance script export failed: " << error << '\n';
            return false;
        }
        std::cout << "wrote_guidance_csv: " << options.guidance_csv_path << '\n';
    }

    if (options.write_svg) {
        const post2::core::CaseConfig orbit_case = options.has_case_config
            ? options.case_config
            : post2::core::case_from_simulation_config(options.config);
        const std::vector<post2::core::PredictedTrajectoryPath> predicted_paths =
            post2::core::predict_phase_end_trajectory_paths(orbit_case, result.state_log);
        if (!post2::core::write_svg_file(options.svg_path, result.state_log, predicted_paths, &error)) {
            std::cerr << "SVG export failed: " << error << '\n';
            return false;
        }
        std::cout << "wrote_svg: " << options.svg_path << '\n';
    }

    return true;
}

} // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parse_options(argc, argv, &options)) {
        print_help();
        return 2;
    }
    sync_loaded_case_from_legacy_surface(&options);
    if (!prepare_optimization_case(&options)) {
        print_help();
        return 2;
    }

    std::string vehicle_error;
    const auto& active_vehicle = options.has_case_config ? options.case_config.vehicle : options.config.vehicle;
    if (!post2::vehicle::validate_vehicle_config(active_vehicle, &vehicle_error)) {
        std::cerr << "Invalid vehicle config: " << vehicle_error << '\n';
        return 2;
    }

    if (!options.save_vehicle_config_path.empty()) {
        if (!post2::vehicle::save_vehicle_config_file(
                options.save_vehicle_config_path,
                active_vehicle,
                &vehicle_error)) {
            std::cerr << "Vehicle config save failed: " << vehicle_error << '\n';
            return 1;
        }
        std::cout << "wrote_vehicle_config: " << options.save_vehicle_config_path << '\n';
    }

    const auto service = post2::core::make_trajectory_service(options.mode, options.host, options.port);

    if (options.command == Command::Optimize) {
        auto optimize_result = post2::core::optimize_case(&options.case_config, *service);
        if (!optimize_result.ok) {
            print_optimization_summary(optimize_result);
            std::cerr << "Optimization failed: " << optimize_result.error << '\n';
            return 1;
        }

        print_optimization_summary(optimize_result);
        print_run_summary(
            options.mode,
            optimize_result.final_simulation,
            &options.case_config,
            post2::core::simulation_config_from_case(options.case_config),
            options.case_config.vehicle);

        if (!options.save_case_config_path.empty()) {
            std::string error;
            if (!post2::core::save_case_config_file(options.save_case_config_path, options.case_config, &error)) {
                std::cerr << "Case save failed: " << error << '\n';
                return 1;
            }
            std::cout << "wrote_case: " << options.save_case_config_path << '\n';
        }

        if (!write_exports(options, optimize_result.final_simulation)) {
            return 1;
        }
        return 0;
    }

    const auto result = options.has_case_config
        ? service->simulate(options.case_config)
        : service->simulate(options.config);
    if (!result.ok) {
        std::cerr << "Simulation failed: " << result.error << '\n';
        return 1;
    }

    print_run_summary(
        options.mode,
        result,
        options.has_case_config ? &options.case_config : nullptr,
        options.config,
        active_vehicle);

    if (!options.save_case_config_path.empty()) {
        const post2::core::CaseConfig case_to_save = options.has_case_config
            ? options.case_config
            : post2::core::case_from_simulation_config(options.config);
        std::string error;
        if (!post2::core::save_case_config_file(options.save_case_config_path, case_to_save, &error)) {
            std::cerr << "Case save failed: " << error << '\n';
            return 1;
        }
        std::cout << "wrote_case: " << options.save_case_config_path << '\n';
    }

    if (!write_exports(options, result)) {
        return 1;
    }

    return 0;
}
