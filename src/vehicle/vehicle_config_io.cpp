#include "post2/vehicle/vehicle_config_io.hpp"

#include "post2/vehicle/runtime_state.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

namespace post2::vehicle {

namespace {

std::string trim(std::string text)
{
    const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();

    if (first >= last) {
        return "";
    }
    return std::string(first, last);
}

std::string lowercase(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
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

bool parse_size(const std::string& text, std::size_t* value)
{
    try {
        std::size_t consumed = 0;
        const unsigned long parsed = std::stoul(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        *value = static_cast<std::size_t>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_bool(const std::string& text, bool* value)
{
    const std::string normalized = lowercase(trim(text));
    if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
        *value = true;
        return true;
    }
    if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
        *value = false;
        return true;
    }
    return false;
}

bool parse_indexed_tank_key(const std::string& key, std::size_t* index, std::string* field)
{
    constexpr char prefix[] = "tank.";
    if (key.rfind(prefix, 0) != 0) {
        return false;
    }

    const std::size_t index_start = 5;
    const std::size_t dot = key.find('.', index_start);
    if (dot == std::string::npos) {
        return false;
    }

    if (!parse_size(key.substr(index_start, dot - index_start), index)) {
        return false;
    }
    *field = key.substr(dot + 1);
    return true;
}

bool parse_stage_key(const std::string& key, std::size_t* stage_index, std::string* field)
{
    constexpr char prefix[] = "stage.";
    if (key.rfind(prefix, 0) != 0) {
        return false;
    }

    const std::size_t index_start = 6;
    const std::size_t dot = key.find('.', index_start);
    if (dot == std::string::npos) {
        return false;
    }
    if (!parse_size(key.substr(index_start, dot - index_start), stage_index)) {
        return false;
    }
    *field = key.substr(dot + 1);
    return true;
}

bool parse_stage_tank_key(
    const std::string& field,
    std::size_t* tank_index,
    std::string* tank_field)
{
    constexpr char prefix[] = "tank.";
    if (field.rfind(prefix, 0) != 0) {
        return false;
    }

    const std::size_t index_start = 5;
    const std::size_t dot = field.find('.', index_start);
    if (dot == std::string::npos) {
        return false;
    }
    if (!parse_size(field.substr(index_start, dot - index_start), tank_index)) {
        return false;
    }
    *tank_field = field.substr(dot + 1);
    return true;
}

bool parse_engine_feed_tanks(
    const std::unordered_map<std::string, std::string>& values,
    const std::string& prefix,   // e.g. "engine." or "stage.0.engine."
    EngineConfig* engine,
    std::string* error)
{
    const std::string count_key = prefix + "feed_tanks.count";
    const auto count_it = values.find(count_key);
    if (count_it == values.end()) {
        return true;   // absent -> leave empty
    }
    std::size_t count = 0;
    if (!parse_size(count_it->second, &count)) {
        if (error) {
            *error = "invalid number for " + count_key;
        }
        return false;
    }
    engine->feed_tanks.assign(count, TankRef{});
    for (std::size_t k = 0; k < count; ++k) {
        const std::string entry_prefix = prefix + "feed_tanks." + std::to_string(k) + ".";
        if (const auto it = values.find(entry_prefix + "stage"); it != values.end()) {
            engine->feed_tanks[k].stage_name = it->second;
        }
        if (const auto it = values.find(entry_prefix + "tank"); it != values.end()) {
            engine->feed_tanks[k].tank_name = it->second;
        }
    }
    return true;
}

bool parse_t2t_connections(
    const std::unordered_map<std::string, std::string>& values,
    VehicleConfig* config,
    std::string* error)
{
    const auto count_it = values.find("tank_to_tank_connections.count");
    if (count_it == values.end()) {
        return true;
    }
    std::size_t count = 0;
    if (!parse_size(count_it->second, &count)) {
        if (error) {
            *error = "invalid number for tank_to_tank_connections.count";
        }
        return false;
    }
    config->tank_to_tank_connections.assign(count, T2TConnection{});
    for (std::size_t k = 0; k < count; ++k) {
        const std::string p = "tank_to_tank_connections." + std::to_string(k) + ".";
        T2TConnection& c = config->tank_to_tank_connections[k];
        if (const auto it = values.find(p + "source.stage"); it != values.end()) {
            c.source.stage_name = it->second;
        }
        if (const auto it = values.find(p + "source.tank"); it != values.end()) {
            c.source.tank_name = it->second;
        }
        if (const auto it = values.find(p + "dest.stage"); it != values.end()) {
            c.dest.stage_name = it->second;
        }
        if (const auto it = values.find(p + "dest.tank"); it != values.end()) {
            c.dest.tank_name = it->second;
        }
        if (const auto it = values.find(p + "rate_kgps"); it != values.end()) {
            if (!parse_double(it->second, &c.rate_kgps)) {
                if (error) {
                    *error = "invalid number for " + p + "rate_kgps";
                }
                return false;
            }
        }
        if (const auto it = values.find(p + "start_time_s"); it != values.end()) {
            double v = 0.0;
            if (!parse_double(it->second, &v)) {
                if (error) {
                    *error = "invalid number for " + p + "start_time_s";
                }
                return false;
            }
            c.start_time_s = v;
        }
        if (const auto it = values.find(p + "end_time_s"); it != values.end()) {
            double v = 0.0;
            if (!parse_double(it->second, &v)) {
                if (error) {
                    *error = "invalid number for " + p + "end_time_s";
                }
                return false;
            }
            c.end_time_s = v;
        }
    }
    return true;
}

bool validate_engine_config(const EngineConfig& engine, const std::string& prefix, std::string* error)
{
    if (engine.max_thrust_n < 0.0) {
        if (error) {
            *error = prefix + ".max_thrust_n cannot be negative";
        }
        return false;
    }
    if (engine.isp_s < 0.0) {
        if (error) {
            *error = prefix + ".isp_s cannot be negative";
        }
        return false;
    }
    if (engine.enabled && engine.max_thrust_n > 0.0 && engine.isp_s <= 0.0) {
        if (error) {
            *error = "enabled engine with positive thrust requires positive " + prefix + ".isp_s";
        }
        return false;
    }
    if (norm(engine.direction_body) <= 0.0) {
        if (error) {
            *error = prefix + " direction cannot be zero";
        }
        return false;
    }
    return true;
}

bool validate_tanks(const std::vector<TankConfig>& tanks, const std::string& prefix, std::string* error)
{
    for (std::size_t i = 0; i < tanks.size(); ++i) {
        const TankConfig& tank = tanks[i];
        if (tank.name.empty()) {
            if (error) {
                *error = prefix + " tank " + std::to_string(i) + " name cannot be empty";
            }
            return false;
        }
        if (tank.capacity_kg < 0.0 || tank.initial_kg < 0.0) {
            if (error) {
                *error = prefix + " tank " + std::to_string(i) + " masses cannot be negative";
            }
            return false;
        }
        if (tank.capacity_kg > 0.0 && tank.initial_kg > tank.capacity_kg) {
            if (error) {
                *error = prefix + " tank " + std::to_string(i) + " initial_kg exceeds capacity_kg";
            }
            return false;
        }
    }
    return true;
}

} // namespace

VehicleConfig default_vehicle_config()
{
    VehicleConfig config;
    config.name = "default";
    config.dry_mass_kg = 1000.0;
    config.engine.enabled = false;
    config.engine.max_thrust_n = 0.0;
    config.engine.isp_s = 0.0;
    config.engine.direction_body = {1.0, 0.0, 0.0};
    // Ship a non-empty feed_tanks so configs built from defaults pass
    // validation if the user later enables the engine without supplying
    // a feed list. Default stage is "stage 1" with one "main" tank.
    config.engine.feed_tanks = {TankRef{"stage 1", "main"}};
    config.tanks = {TankConfig{}};
    config.stages.clear();
    return config;
}

bool validate_vehicle_config(const VehicleConfig& config, std::string* error)
{
    if (config.name.empty()) {
        if (error) {
            *error = "vehicle name cannot be empty";
        }
        return false;
    }
    if (config.dry_mass_kg <= 0.0) {
        if (error) {
            *error = "dry_mass_kg must be positive";
        }
        return false;
    }
    if (!validate_engine_config(config.engine, "engine", error) ||
        !validate_tanks(config.tanks, "vehicle", error)) {
        return false;
    }

    const std::vector<StageConfig> stages = effective_stage_configs(config);
    if (stages.empty()) {
        if (error) {
            *error = "vehicle must contain at least one stage";
        }
        return false;
    }
    for (std::size_t i = 0; i < stages.size(); ++i) {
        const StageConfig& stage = stages[i];
        if (stage.name.empty()) {
            if (error) {
                *error = "stage " + std::to_string(i) + " name cannot be empty";
            }
            return false;
        }
        if (stage.dry_mass_kg < 0.0) {
            if (error) {
                *error = "stage " + std::to_string(i) + " dry_mass_kg cannot be negative";
            }
            return false;
        }
        const std::string prefix = "stage " + std::to_string(i);
        if (!validate_engine_config(stage.engine, prefix + ".engine", error) ||
            !validate_tanks(stage.tanks, prefix, error)) {
            return false;
        }
        // T2E: every stage with an enabled engine on an active stage with
        // positive thrust must have feed_tanks, and every entry must resolve.
        const bool engine_contributes =
            stage.active && stage.engine.enabled && stage.engine.max_thrust_n > 0.0;
        if (engine_contributes) {
            if (stage.engine.feed_tanks.empty()) {
                if (error) {
                    *error = "stage '" + stage.name + "': active engine has no feed_tanks";
                }
                return false;
            }
            for (std::size_t k = 0; k < stage.engine.feed_tanks.size(); ++k) {
                const TankRef& ref = stage.engine.feed_tanks[k];
                if (!resolve_tank_ref(config, ref).has_value()) {
                    if (error) {
                        *error = "stage '" + stage.name + "': feed_tanks[" +
                            std::to_string(k) + "] references unknown tank '" +
                            ref.stage_name + "/" + ref.tank_name + "'";
                    }
                    return false;
                }
            }
        }
    }

    // T2T connection sanity.
    for (std::size_t i = 0; i < config.tank_to_tank_connections.size(); ++i) {
        const T2TConnection& c = config.tank_to_tank_connections[i];
        if (c.rate_kgps < 0.0) {
            if (error) {
                *error = "tank_to_tank_connections[" + std::to_string(i) +
                    "]: rate_kgps cannot be negative";
            }
            return false;
        }
        if (!resolve_tank_ref(config, c.source).has_value()) {
            if (error) {
                *error = "tank_to_tank_connections[" + std::to_string(i) +
                    "]: unknown source tank '" +
                    c.source.stage_name + "/" + c.source.tank_name + "'";
            }
            return false;
        }
        if (!resolve_tank_ref(config, c.dest).has_value()) {
            if (error) {
                *error = "tank_to_tank_connections[" + std::to_string(i) +
                    "]: unknown dest tank '" +
                    c.dest.stage_name + "/" + c.dest.tank_name + "'";
            }
            return false;
        }
        if (c.start_time_s.has_value() && c.end_time_s.has_value() &&
            *c.start_time_s > *c.end_time_s) {
            if (error) {
                *error = "tank_to_tank_connections[" + std::to_string(i) +
                    "]: start_time_s > end_time_s";
            }
            return false;
        }
    }

    return true;
}

std::string vehicle_config_summary(const VehicleConfig& config)
{
    double propellant_kg = 0.0;
    const std::vector<StageConfig> stages = effective_stage_configs(config);
    for (const auto& stage : stages) {
        for (const auto& tank : stage.tanks) {
            propellant_kg += tank.initial_kg;
        }
    }

    std::ostringstream output;
    output << config.name
           << " | dry mass " << effective_dry_mass_kg(config)
           << " kg | stages " << stages.size()
           << " | payload " << payload_stage_dry_mass_kg(config)
           << " kg"
           << " | prop " << propellant_kg
           << " kg";
    return output.str();
}

namespace {

void write_engine_feed_tanks(std::ostringstream& output, const std::string& prefix, const EngineConfig& engine)
{
    output << prefix << "feed_tanks.count=" << engine.feed_tanks.size() << '\n';
    for (std::size_t k = 0; k < engine.feed_tanks.size(); ++k) {
        output << prefix << "feed_tanks." << k << ".stage=" << engine.feed_tanks[k].stage_name << '\n';
        output << prefix << "feed_tanks." << k << ".tank=" << engine.feed_tanks[k].tank_name << '\n';
    }
}

} // namespace

std::string vehicle_config_to_text(const VehicleConfig& config)
{
    VehicleConfig normalized = config;
    if (normalized.stages.empty()) {
        normalized.stages = effective_stage_configs(normalized);
    }
    sync_legacy_vehicle_fields_from_first_stage(&normalized);

    std::ostringstream output;
    output << std::setprecision(17);
    output << "# POST2 Lite vehicle config\n";
    output << "format=post2_vehicle_config_v1\n";
    output << "name=" << normalized.name << '\n';
    output << "dry_mass_kg=" << normalized.dry_mass_kg << '\n';
    output << "engine.enabled=" << (normalized.engine.enabled ? "true" : "false") << '\n';
    output << "engine.max_thrust_n=" << normalized.engine.max_thrust_n << '\n';
    output << "engine.isp_s=" << normalized.engine.isp_s << '\n';
    output << "engine.direction_x=" << normalized.engine.direction_body.x << '\n';
    output << "engine.direction_y=" << normalized.engine.direction_body.y << '\n';
    output << "engine.direction_z=" << normalized.engine.direction_body.z << '\n';
    write_engine_feed_tanks(output, "engine.", normalized.engine);
    output << "tank.count=" << normalized.tanks.size() << '\n';
    for (std::size_t i = 0; i < normalized.tanks.size(); ++i) {
        const TankConfig& tank = normalized.tanks[i];
        output << "tank." << i << ".name=" << tank.name << '\n';
        output << "tank." << i << ".propellant=" << tank.propellant << '\n';
        output << "tank." << i << ".capacity_kg=" << tank.capacity_kg << '\n';
        output << "tank." << i << ".initial_kg=" << tank.initial_kg << '\n';
    }
    output << "stage.count=" << normalized.stages.size() << '\n';
    for (std::size_t i = 0; i < normalized.stages.size(); ++i) {
        const StageConfig& stage = normalized.stages[i];
        const std::string stage_prefix = "stage." + std::to_string(i) + ".";
        output << stage_prefix << "name=" << stage.name << '\n';
        output << stage_prefix << "active=" << (stage.active ? "true" : "false") << '\n';
        output << stage_prefix << "attached=" << (stage.attached ? "true" : "false") << '\n';
        output << stage_prefix << "dry_mass_kg=" << stage.dry_mass_kg << '\n';
        output << stage_prefix << "engine.enabled=" << (stage.engine.enabled ? "true" : "false") << '\n';
        output << stage_prefix << "engine.max_thrust_n=" << stage.engine.max_thrust_n << '\n';
        output << stage_prefix << "engine.isp_s=" << stage.engine.isp_s << '\n';
        output << stage_prefix << "engine.direction_x=" << stage.engine.direction_body.x << '\n';
        output << stage_prefix << "engine.direction_y=" << stage.engine.direction_body.y << '\n';
        output << stage_prefix << "engine.direction_z=" << stage.engine.direction_body.z << '\n';
        write_engine_feed_tanks(output, stage_prefix + "engine.", stage.engine);
        output << stage_prefix << "tank.count=" << stage.tanks.size() << '\n';
        for (std::size_t j = 0; j < stage.tanks.size(); ++j) {
            const TankConfig& tank = stage.tanks[j];
            output << stage_prefix << "tank." << j << ".name=" << tank.name << '\n';
            output << stage_prefix << "tank." << j << ".propellant=" << tank.propellant << '\n';
            output << stage_prefix << "tank." << j << ".capacity_kg=" << tank.capacity_kg << '\n';
            output << stage_prefix << "tank." << j << ".initial_kg=" << tank.initial_kg << '\n';
        }
    }
    output << "tank_to_tank_connections.count=" << normalized.tank_to_tank_connections.size() << '\n';
    for (std::size_t i = 0; i < normalized.tank_to_tank_connections.size(); ++i) {
        const T2TConnection& c = normalized.tank_to_tank_connections[i];
        const std::string conn_prefix = "tank_to_tank_connections." + std::to_string(i) + ".";
        output << conn_prefix << "source.stage=" << c.source.stage_name << '\n';
        output << conn_prefix << "source.tank=" << c.source.tank_name << '\n';
        output << conn_prefix << "dest.stage=" << c.dest.stage_name << '\n';
        output << conn_prefix << "dest.tank=" << c.dest.tank_name << '\n';
        output << conn_prefix << "rate_kgps=" << c.rate_kgps << '\n';
        if (c.start_time_s.has_value()) {
            output << conn_prefix << "start_time_s=" << *c.start_time_s << '\n';
        }
        if (c.end_time_s.has_value()) {
            output << conn_prefix << "end_time_s=" << *c.end_time_s << '\n';
        }
    }
    return output.str();
}

bool vehicle_config_from_text(const std::string& text, VehicleConfig* config, std::string* error)
{
    std::istringstream input(text);
    std::string line;
    std::unordered_map<std::string, std::string> values;
    int line_number = 0;

    while (std::getline(input, line)) {
        ++line_number;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }

        const std::size_t equals = line.find('=');
        if (equals == std::string::npos) {
            if (error) {
                *error = "invalid vehicle config line " + std::to_string(line_number);
            }
            return false;
        }

        values[trim(line.substr(0, equals))] = trim(line.substr(equals + 1));
    }

    VehicleConfig parsed = default_vehicle_config();
    auto set_double = [&](const char* key, double* target) -> bool {
        const auto it = values.find(key);
        if (it == values.end()) {
            return true;
        }
        if (!parse_double(it->second, target)) {
            if (error) {
                *error = std::string("invalid number for ") + key;
            }
            return false;
        }
        return true;
    };

    auto set_bool = [&](const char* key, bool* target) -> bool {
        const auto it = values.find(key);
        if (it == values.end()) {
            return true;
        }
        if (!parse_bool(it->second, target)) {
            if (error) {
                *error = std::string("invalid bool for ") + key;
            }
            return false;
        }
        return true;
    };

    auto set_double_alias = [&](const char* primary, const char* legacy, double* target) -> bool {
        if (values.find(primary) != values.end()) {
            return set_double(primary, target);
        }
        return set_double(legacy, target);
    };

    auto set_bool_alias = [&](const char* primary, const char* legacy, bool* target) -> bool {
        if (values.find(primary) != values.end()) {
            return set_bool(primary, target);
        }
        return set_bool(legacy, target);
    };

    if (const auto it = values.find("name"); it != values.end()) {
        parsed.name = it->second;
    }
    if (!set_double("dry_mass_kg", &parsed.dry_mass_kg) ||
        !set_bool_alias("engine.enabled", "thrust.enabled", &parsed.engine.enabled) ||
        !set_double_alias("engine.max_thrust_n", "thrust.max_thrust_n", &parsed.engine.max_thrust_n) ||
        !set_double_alias("engine.isp_s", "thrust.isp_s", &parsed.engine.isp_s) ||
        !set_double_alias("engine.direction_x", "thrust.direction_x", &parsed.engine.direction_body.x) ||
        !set_double_alias("engine.direction_y", "thrust.direction_y", &parsed.engine.direction_body.y) ||
        !set_double_alias("engine.direction_z", "thrust.direction_z", &parsed.engine.direction_body.z) ||
        !parse_engine_feed_tanks(values, "engine.", &parsed.engine, error)) {
        return false;
    }

    std::size_t tank_count = parsed.tanks.size();
    if (const auto it = values.find("tank.count"); it != values.end()) {
        if (!parse_size(it->second, &tank_count)) {
            if (error) {
                *error = "invalid number for tank.count";
            }
            return false;
        }
    }

    parsed.tanks.assign(tank_count, TankConfig{});
    for (const auto& [key, value] : values) {
        std::size_t tank_index = 0;
        std::string field;
        if (!parse_indexed_tank_key(key, &tank_index, &field)) {
            continue;
        }
        if (tank_index >= parsed.tanks.size()) {
            if (error) {
                *error = "tank index out of range in key " + key;
            }
            return false;
        }

        TankConfig& tank = parsed.tanks[tank_index];
        if (field == "name") {
            tank.name = value;
        } else if (field == "propellant") {
            tank.propellant = value;
        } else if (field == "capacity_kg") {
            if (!parse_double(value, &tank.capacity_kg)) {
                if (error) {
                    *error = "invalid number for " + key;
                }
                return false;
            }
        } else if (field == "initial_kg") {
            if (!parse_double(value, &tank.initial_kg)) {
                if (error) {
                    *error = "invalid number for " + key;
                }
                return false;
            }
        }
    }

    std::size_t stage_count = 0;
    if (const auto it = values.find("stage.count"); it != values.end()) {
        if (!parse_size(it->second, &stage_count)) {
            if (error) {
                *error = "invalid number for stage.count";
            }
            return false;
        }
    }
    if (stage_count > 0) {
        parsed.stages.assign(stage_count, StageConfig{});
        for (std::size_t i = 0; i < parsed.stages.size(); ++i) {
            parsed.stages[i].name = "stage " + std::to_string(i + 1);
        }

        std::vector<std::size_t> stage_tank_counts(stage_count, 1);
        for (const auto& [key, value] : values) {
            std::size_t stage_index = 0;
            std::string field;
            if (!parse_stage_key(key, &stage_index, &field)) {
                continue;
            }
            if (stage_index >= parsed.stages.size()) {
                if (error) {
                    *error = "stage index out of range in key " + key;
                }
                return false;
            }
            if (field == "tank.count") {
                if (!parse_size(value, &stage_tank_counts[stage_index])) {
                    if (error) {
                        *error = "invalid number for " + key;
                    }
                    return false;
                }
            }
        }

        for (std::size_t i = 0; i < parsed.stages.size(); ++i) {
            parsed.stages[i].tanks.assign(stage_tank_counts[i], TankConfig{});
        }

        for (const auto& [key, value] : values) {
            std::size_t stage_index = 0;
            std::string field;
            if (!parse_stage_key(key, &stage_index, &field)) {
                continue;
            }
            if (stage_index >= parsed.stages.size()) {
                if (error) {
                    *error = "stage index out of range in key " + key;
                }
                return false;
            }

            StageConfig& stage = parsed.stages[stage_index];
            std::size_t tank_index = 0;
            std::string tank_field;
            if (parse_stage_tank_key(field, &tank_index, &tank_field)) {
                if (tank_index >= stage.tanks.size()) {
                    if (error) {
                        *error = "stage tank index out of range in key " + key;
                    }
                    return false;
                }
                TankConfig& tank = stage.tanks[tank_index];
                if (tank_field == "name") {
                    tank.name = value;
                } else if (tank_field == "propellant") {
                    tank.propellant = value;
                } else if (tank_field == "capacity_kg") {
                    if (!parse_double(value, &tank.capacity_kg)) {
                        if (error) {
                            *error = "invalid number for " + key;
                        }
                        return false;
                    }
                } else if (tank_field == "initial_kg") {
                    if (!parse_double(value, &tank.initial_kg)) {
                        if (error) {
                            *error = "invalid number for " + key;
                        }
                        return false;
                    }
                }
                continue;
            }

            if (field == "name") {
                stage.name = value;
            } else if (field == "active") {
                if (!parse_bool(value, &stage.active)) {
                    if (error) {
                        *error = "invalid bool for " + key;
                    }
                    return false;
                }
            } else if (field == "attached") {
                if (!parse_bool(value, &stage.attached)) {
                    if (error) {
                        *error = "invalid bool for " + key;
                    }
                    return false;
                }
            } else if (field == "dry_mass_kg") {
                if (!parse_double(value, &stage.dry_mass_kg)) {
                    if (error) {
                        *error = "invalid number for " + key;
                    }
                    return false;
                }
            } else if (field == "engine.enabled") {
                if (!parse_bool(value, &stage.engine.enabled)) {
                    if (error) {
                        *error = "invalid bool for " + key;
                    }
                    return false;
                }
            } else if (field == "engine.max_thrust_n") {
                if (!parse_double(value, &stage.engine.max_thrust_n)) {
                    if (error) {
                        *error = "invalid number for " + key;
                    }
                    return false;
                }
            } else if (field == "engine.isp_s") {
                if (!parse_double(value, &stage.engine.isp_s)) {
                    if (error) {
                        *error = "invalid number for " + key;
                    }
                    return false;
                }
            } else if (field == "engine.direction_x") {
                if (!parse_double(value, &stage.engine.direction_body.x)) {
                    if (error) {
                        *error = "invalid number for " + key;
                    }
                    return false;
                }
            } else if (field == "engine.direction_y") {
                if (!parse_double(value, &stage.engine.direction_body.y)) {
                    if (error) {
                        *error = "invalid number for " + key;
                    }
                    return false;
                }
            } else if (field == "engine.direction_z") {
                if (!parse_double(value, &stage.engine.direction_body.z)) {
                    if (error) {
                        *error = "invalid number for " + key;
                    }
                    return false;
                }
            }
        }
        for (std::size_t i = 0; i < parsed.stages.size(); ++i) {
            const std::string prefix = "stage." + std::to_string(i) + ".engine.";
            if (!parse_engine_feed_tanks(values, prefix, &parsed.stages[i].engine, error)) {
                return false;
            }
        }
        sync_legacy_vehicle_fields_from_first_stage(&parsed);
    } else {
        parsed.stages = effective_stage_configs(parsed);
    }

    if (!parse_t2t_connections(values, &parsed, error)) {
        return false;
    }

    if (!validate_vehicle_config(parsed, error)) {
        return false;
    }

    *config = std::move(parsed);
    return true;
}

bool load_vehicle_config_file(const std::string& path, VehicleConfig* config, std::string* error)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        if (error) {
            *error = "failed to open vehicle config: " + path;
        }
        return false;
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return vehicle_config_from_text(buffer.str(), config, error);
}

bool save_vehicle_config_file(const std::string& path, const VehicleConfig& config, std::string* error)
{
    if (!validate_vehicle_config(config, error)) {
        return false;
    }

    std::ofstream output(path, std::ios::binary);
    if (!output) {
        if (error) {
            *error = "failed to open vehicle config for writing: " + path;
        }
        return false;
    }

    output << vehicle_config_to_text(config);
    return true;
}

} // namespace post2::vehicle
