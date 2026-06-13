#include "post2/core/ksp_vehicle_site_import.hpp"

#include "post2/core/json.hpp"
#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle_config_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace post2::core {

namespace {

constexpr int kUnknownStage = -999999;
constexpr double kG0 = 9.80665;
constexpr double kPi = 3.141592653589793238462643383279502884;

bool fail(std::string* error, const std::string& message)
{
    if (error) {
        *error = message;
    }
    return false;
}

std::string lowercase(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

bool contains_case_insensitive(const std::string& text, const std::string& needle)
{
    return lowercase(text).find(lowercase(needle)) != std::string::npos;
}

const JsonValue* member(const JsonValue& object, const std::string& key)
{
    return find_member(object, key);
}

bool read_string(const JsonValue& object, const std::string& key, std::string* target)
{
    const JsonValue* value = member(object, key);
    if (!value || !value->is_string()) {
        return false;
    }
    *target = value->string_value;
    return true;
}

bool read_number(const JsonValue& object, const std::string& key, double* target)
{
    const JsonValue* value = member(object, key);
    if (!value || !value->is_number()) {
        return false;
    }
    *target = value->number_value;
    return true;
}

bool read_int(const JsonValue& object, const std::string& key, int* target)
{
    double value = 0.0;
    if (!read_number(object, key, &value)) {
        return false;
    }
    *target = static_cast<int>(value);
    return true;
}

bool read_bool(const JsonValue& object, const std::string& key, bool* target)
{
    const JsonValue* value = member(object, key);
    if (!value) {
        return false;
    }
    if (value->is_bool()) {
        *target = value->bool_value;
        return true;
    }
    if (value->is_number()) {
        *target = value->number_value != 0.0;
        return true;
    }
    return false;
}

std::string json_key_string(const JsonValue& value)
{
    if (value.is_string()) {
        return value.string_value;
    }
    if (value.is_number()) {
        std::ostringstream out;
        out << std::setprecision(17) << value.number_value;
        return out.str();
    }
    if (value.is_bool()) {
        return value.bool_value ? "true" : "false";
    }
    return "";
}

bool object_type_contains(const JsonValue& value, const std::string& needle)
{
    if (!value.is_object()) {
        return false;
    }
    const JsonValue* type = member(value, "$type");
    return type && type->is_string() && type->string_value.find(needle) != std::string::npos;
}

JsonValue unwrap_kos_json(const JsonValue& value)
{
    if (value.is_object()) {
        if (object_type_contains(value, "Lexicon")) {
            const JsonValue* entries = member(value, "entries");
            if (entries && entries->is_array()) {
                JsonValue::Object object;
                const auto& array = entries->array_value;
                for (std::size_t i = 0; i + 1 < array.size(); i += 2) {
                    object[json_key_string(unwrap_kos_json(array[i]))] = unwrap_kos_json(array[i + 1]);
                }
                return JsonValue::object(std::move(object));
            }
        }
        if (object_type_contains(value, "ListValue")) {
            const JsonValue* items = member(value, "items");
            if (items && items->is_array()) {
                JsonValue::Array array;
                array.reserve(items->array_value.size());
                for (const auto& item : items->array_value) {
                    array.push_back(unwrap_kos_json(item));
                }
                return JsonValue::array(std::move(array));
            }
        }

        JsonValue::Object object;
        for (const auto& [key, item] : value.object_value) {
            if (key == "$type") {
                continue;
            }
            object[key] = unwrap_kos_json(item);
        }
        return JsonValue::object(std::move(object));
    }

    if (value.is_array()) {
        JsonValue::Array array;
        array.reserve(value.array_value.size());
        for (const auto& item : value.array_value) {
            array.push_back(unwrap_kos_json(item));
        }
        return JsonValue::array(std::move(array));
    }

    return value;
}

struct KspResource {
    std::string name;
    double amount_kg = 0.0;
    double capacity_kg = 0.0;
};

struct KspEngine {
    bool present = false;
    double thrust_vac_n = 0.0;
    double thrust_sl_n = 0.0;
    double isp_vac_s = 0.0;
    double isp_sl_s = 0.0;
    double max_mass_flow_kgps = 0.0;
    double min_throttle = 0.0;
};

struct KspPart {
    int index = -1;
    int ksp_stage = kUnknownStage;
    int decoupled_in = kUnknownStage;
    int physical_stage = kUnknownStage;
    std::string physical_stage_name;
    std::string name;
    std::string title;
    std::string uid;
    std::string parent_uid;
    double dry_mass_kg = 0.0;
    double wet_mass_kg = 0.0;
    double current_mass_kg = 0.0;
    bool has_engine = false;
    std::vector<std::string> module_names;
    std::vector<KspResource> resources;
    KspEngine engine;
};

struct KspHints {
    int launch_engine_stage = kUnknownStage;
    int upper_engine_stage = kUnknownStage;
    int booster_separation_stage = kUnknownStage;
};

struct StageAccum {
    std::string name;
    double dry_mass_kg = 0.0;
    double current_mass_kg = 0.0;
    double resource_initial_kg = 0.0;
    double resource_capacity_kg = 0.0;
    double thrust_vac_n = 0.0;
    double thrust_sl_n = 0.0;
    double isp_vac_weighted = 0.0;
    double isp_sl_weighted = 0.0;
    double max_mass_flow_kgps = 0.0;
    int engine_part_count = 0;
    std::set<std::string> resource_names;
};

bool has_module(const KspPart& part, const std::string& module_name)
{
    return std::find(part.module_names.begin(), part.module_names.end(), module_name) != part.module_names.end();
}

bool is_launch_support_part(const KspPart& part)
{
    return has_module(part, "LaunchClamp");
}

bool is_decoupler_or_fairing_part(const KspPart& part)
{
    return has_module(part, "ModuleDecouple") ||
        has_module(part, "ModuleAnchoredDecoupler") ||
        has_module(part, "ModuleCargoBay");
}

enum class PhysicalClass {
    Support,
    Booster,
    Upper,
};

PhysicalClass classify_part(const KspPart& part, const KspHints& hints)
{
    if (is_launch_support_part(part)) {
        return PhysicalClass::Support;
    }
    if (hints.booster_separation_stage > kUnknownStage) {
        if (part.ksp_stage >= hints.booster_separation_stage ||
            part.decoupled_in >= hints.booster_separation_stage) {
            return PhysicalClass::Booster;
        }
        return PhysicalClass::Upper;
    }

    const std::string physical_name = lowercase(part.physical_stage_name);
    if (part.physical_stage == 0 || physical_name == "booster") {
        return PhysicalClass::Booster;
    }
    if (part.physical_stage < 0 || physical_name == "ground_support") {
        return PhysicalClass::Support;
    }
    return PhysicalClass::Upper;
}

bool is_payload_candidate(const KspPart& part, PhysicalClass physical_class)
{
    return physical_class == PhysicalClass::Upper &&
        part.decoupled_in < 0 &&
        !is_launch_support_part(part) &&
        !is_decoupler_or_fairing_part(part);
}

void add_part_to_stage(const KspPart& part, StageAccum* stage)
{
    stage->dry_mass_kg += std::max(0.0, part.dry_mass_kg);
    stage->current_mass_kg += std::max(0.0, part.current_mass_kg);
    for (const auto& resource : part.resources) {
        const double amount = std::max(0.0, resource.amount_kg);
        const double capacity = std::max(0.0, resource.capacity_kg);
        if (amount > 0.0 || capacity > 0.0) {
            stage->resource_initial_kg += amount;
            stage->resource_capacity_kg += capacity;
            if (!resource.name.empty()) {
                stage->resource_names.insert(resource.name);
            }
        }
    }
    if (part.engine.present && part.engine.thrust_vac_n > 0.0) {
        stage->engine_part_count += 1;
        stage->thrust_vac_n += part.engine.thrust_vac_n;
        stage->thrust_sl_n += std::max(0.0, part.engine.thrust_sl_n);
        stage->isp_vac_weighted += part.engine.thrust_vac_n * std::max(0.0, part.engine.isp_vac_s);
        stage->isp_sl_weighted += std::max(0.0, part.engine.thrust_sl_n) * std::max(0.0, part.engine.isp_sl_s);
        stage->max_mass_flow_kgps += std::max(0.0, part.engine.max_mass_flow_kgps);
    }
}

std::string stage_propellant_name(const StageAccum& stage)
{
    if (stage.resource_names.size() == 1) {
        return *stage.resource_names.begin();
    }
    return stage.resource_names.empty() ? "generic" : "KSP resources";
}

post2::vehicle::StageConfig stage_config_from_accum(const StageAccum& accum, bool active)
{
    post2::vehicle::StageConfig stage;
    stage.name = accum.name;
    stage.active = active;
    stage.attached = true;
    stage.dry_mass_kg = std::max(0.0, accum.dry_mass_kg);
    stage.tanks.clear();

    const bool has_engine = accum.thrust_vac_n > 0.0;
    const bool has_propellant = accum.resource_initial_kg > 0.0 || accum.resource_capacity_kg > 0.0;
    if (has_engine || has_propellant) {
        post2::vehicle::TankConfig tank;
        tank.name = "main";
        tank.propellant = stage_propellant_name(accum);
        tank.initial_kg = std::max(0.0, accum.resource_initial_kg);
        tank.capacity_kg = std::max(tank.initial_kg, accum.resource_capacity_kg);
        stage.tanks.push_back(std::move(tank));
    }

    if (has_engine) {
        stage.engine.enabled = true;
        stage.engine.thrust_vac_n = accum.thrust_vac_n;
        stage.engine.thrust_sl_n = accum.thrust_sl_n;
        stage.engine.isp_vac_s = accum.isp_vac_weighted > 0.0
            ? accum.isp_vac_weighted / accum.thrust_vac_n
            : (accum.max_mass_flow_kgps > 0.0 ? accum.thrust_vac_n / (accum.max_mass_flow_kgps * kG0) : 0.0);
        stage.engine.isp_sl_s = accum.thrust_sl_n > 0.0 && accum.isp_sl_weighted > 0.0
            ? accum.isp_sl_weighted / accum.thrust_sl_n
            : stage.engine.isp_vac_s;
        stage.engine.engine_count = 1;
        stage.engine.direction_body = {1.0, 0.0, 0.0};
        if (stage.tanks.empty()) {
            post2::vehicle::TankConfig tank;
            tank.name = "main";
            stage.tanks.push_back(std::move(tank));
        }
        stage.engine.feed_tanks = {{stage.name, stage.tanks.front().name}};
    } else {
        stage.engine.enabled = false;
        stage.engine.thrust_vac_n = 0.0;
        stage.engine.isp_vac_s = 0.0;
        stage.engine.feed_tanks.clear();
    }

    return stage;
}

std::vector<std::string> parse_string_array(const JsonValue& value)
{
    std::vector<std::string> result;
    if (!value.is_array()) {
        return result;
    }
    for (const auto& entry : value.array_value) {
        if (entry.is_string()) {
            result.push_back(entry.string_value);
        }
    }
    return result;
}

std::vector<int> parse_int_array(const JsonValue& value)
{
    std::vector<int> result;
    if (!value.is_array()) {
        return result;
    }
    for (const auto& entry : value.array_value) {
        if (entry.is_number()) {
            result.push_back(static_cast<int>(entry.number_value));
        }
    }
    return result;
}

KspResource parse_resource(const JsonValue& value)
{
    KspResource resource;
    if (!value.is_object()) {
        return resource;
    }
    read_string(value, "name", &resource.name);
    read_number(value, "amount_kg", &resource.amount_kg);
    read_number(value, "capacity_kg", &resource.capacity_kg);
    return resource;
}

KspEngine parse_engine(const JsonValue& value)
{
    KspEngine engine;
    if (!value.is_object()) {
        return engine;
    }
    engine.present = true;
    read_number(value, "possible_thrust_vac_n", &engine.thrust_vac_n);
    read_number(value, "possible_thrust_sl_n", &engine.thrust_sl_n);
    read_number(value, "vacuum_isp_s", &engine.isp_vac_s);
    read_number(value, "sealevel_isp_s", &engine.isp_sl_s);
    read_number(value, "max_mass_flow_kgps", &engine.max_mass_flow_kgps);
    read_number(value, "min_throttle", &engine.min_throttle);
    return engine;
}

KspPart parse_part(const JsonValue& value)
{
    KspPart part;
    if (!value.is_object()) {
        return part;
    }
    read_int(value, "index", &part.index);
    read_int(value, "ksp_stage", &part.ksp_stage);
    read_int(value, "decoupled_in", &part.decoupled_in);
    read_int(value, "physical_stage", &part.physical_stage);
    read_string(value, "physical_stage_name", &part.physical_stage_name);
    read_string(value, "name", &part.name);
    read_string(value, "title", &part.title);
    read_string(value, "uid", &part.uid);
    read_string(value, "parent_uid", &part.parent_uid);
    read_number(value, "dry_mass_kg", &part.dry_mass_kg);
    read_number(value, "wet_mass_kg", &part.wet_mass_kg);
    if (!read_number(value, "mass_kg", &part.current_mass_kg)) {
        read_number(value, "current_mass_kg", &part.current_mass_kg);
    }
    read_bool(value, "has_engine", &part.has_engine);

    if (const JsonValue* modules = member(value, "module_names")) {
        part.module_names = parse_string_array(*modules);
    }
    if (const JsonValue* resources = member(value, "resources")) {
        if (resources->is_array()) {
            for (const auto& resource_value : resources->array_value) {
                part.resources.push_back(parse_resource(resource_value));
            }
        }
    }
    if (const JsonValue* engine = member(value, "engine")) {
        part.engine = parse_engine(*engine);
    } else if (part.has_engine) {
        part.engine.present = true;
    }
    part.has_engine = part.has_engine || part.engine.present;
    return part;
}

std::vector<KspPart> parse_parts(const JsonValue& root)
{
    std::vector<KspPart> parts;
    const JsonValue* parts_value = member(root, "parts");
    if (!parts_value || !parts_value->is_array()) {
        return parts;
    }
    parts.reserve(parts_value->array_value.size());
    for (const auto& part_value : parts_value->array_value) {
        parts.push_back(parse_part(part_value));
    }
    return parts;
}

KspHints parse_hints(const JsonValue& root)
{
    KspHints hints;
    const JsonValue* value = member(root, "staging_hints");
    if (!value || !value->is_object()) {
        return hints;
    }
    read_int(*value, "launch_engine_ksp_stage", &hints.launch_engine_stage);
    read_int(*value, "upper_engine_ksp_stage", &hints.upper_engine_stage);
    read_int(*value, "booster_separation_ksp_stage", &hints.booster_separation_stage);
    return hints;
}

void parse_launch_site(const JsonValue& root, KspVehicleSiteImport* imported)
{
    const JsonValue* site = member(root, "launch_site");
    if (!site || !site->is_object()) {
        return;
    }

    imported->has_launch_site =
        read_number(*site, "latitude_deg", &imported->launch_site.latitude_deg) |
        read_number(*site, "longitude_deg", &imported->launch_site.longitude_deg) |
        read_number(*site, "altitude_m", &imported->launch_site.altitude_m);

    imported->has_earth_radius_m = read_number(*site, "body_radius_m", &imported->earth_radius_m);
    imported->has_earth_mu_m3s2 = read_number(*site, "body_mu_m3ps2", &imported->earth_mu_m3s2);

    if (const JsonValue* angular_velocity = member(*site, "body_angular_velocity_radps")) {
        if (angular_velocity->is_object()) {
            double mag = 0.0;
            if (read_number(*angular_velocity, "mag", &mag) && mag > 0.0) {
                imported->earth_rotation_rad_per_s = mag;
                imported->has_earth_rotation_rad_per_s = true;
            }
        }
    }
    if (!imported->has_earth_rotation_rad_per_s) {
        double period_s = 0.0;
        if (read_number(*site, "body_rotation_period_s", &period_s) && period_s > 0.0) {
            imported->earth_rotation_rad_per_s = 2.0 * kPi / period_s;
            imported->has_earth_rotation_rad_per_s = true;
        }
    }
}

double read_object_number_or_zero(const JsonValue& value, const std::string& key)
{
    double number = 0.0;
    read_number(value, key, &number);
    return number;
}

StageAccum stage_accum_from_summary(const JsonValue& value, std::string default_name)
{
    StageAccum stage;
    stage.name = std::move(default_name);
    if (!value.is_object()) {
        return stage;
    }
    read_string(value, "name", &stage.name);
    read_number(value, "dry_mass_kg", &stage.dry_mass_kg);
    if (!read_number(value, "current_mass_kg", &stage.current_mass_kg)) {
        read_number(value, "wet_mass_kg", &stage.current_mass_kg);
    }
    read_number(value, "resource_initial_mass_kg", &stage.resource_initial_kg);
    read_number(value, "resource_capacity_mass_kg", &stage.resource_capacity_kg);
    read_number(value, "engine_thrust_vac_n", &stage.thrust_vac_n);
    read_number(value, "engine_thrust_sl_n", &stage.thrust_sl_n);
    const double isp_vac = read_object_number_or_zero(value, "engine_isp_vac_s");
    const double isp_sl = read_object_number_or_zero(value, "engine_isp_sl_s");
    stage.isp_vac_weighted = stage.thrust_vac_n * isp_vac;
    stage.isp_sl_weighted = stage.thrust_sl_n * isp_sl;
    read_number(value, "engine_max_mass_flow_kgps", &stage.max_mass_flow_kgps);
    return stage;
}

bool parse_stage_summaries(
    const JsonValue& root,
    StageAccum* booster,
    StageAccum* upper,
    std::string* error)
{
    const JsonValue* stages = member(root, "stages");
    if (!stages || !stages->is_array()) {
        return fail(error, "KSP vehicle/site JSON contains no parts or stages");
    }
    if (stages->array_value.empty()) {
        return fail(error, "KSP vehicle/site JSON stages array is empty");
    }
    *booster = stage_accum_from_summary(stages->array_value.front(), "booster");
    if (stages->array_value.size() >= 2) {
        *upper = stage_accum_from_summary(stages->array_value[1], "upper_stack");
    } else {
        upper->name = "upper_stack";
    }
    return true;
}

bool parse_explicit_payload(
    const JsonValue& root,
    StageAccum* payload,
    std::set<int>* payload_indices)
{
    const JsonValue* value = member(root, "payload");
    if (!value || !value->is_object()) {
        return false;
    }
    payload->name = "payload";
    read_number(*value, "dry_mass_kg", &payload->dry_mass_kg);
    read_number(*value, "wet_mass_kg", &payload->current_mass_kg);
    read_number(*value, "current_mass_kg", &payload->current_mass_kg);
    read_number(*value, "resource_initial_mass_kg", &payload->resource_initial_kg);
    read_number(*value, "resource_capacity_mass_kg", &payload->resource_capacity_kg);
    if (payload->current_mass_kg <= 0.0 && payload->dry_mass_kg > 0.0) {
        payload->current_mass_kg = payload->dry_mass_kg + payload->resource_initial_kg;
    }
    if (const JsonValue* indices = member(*value, "part_indices")) {
        for (const int index : parse_int_array(*indices)) {
            payload_indices->insert(index);
        }
    }
    return true;
}

void build_from_parts(
    const JsonValue& root,
    const std::vector<KspPart>& parts,
    StageAccum* booster,
    StageAccum* upper,
    StageAccum* payload)
{
    booster->name = "booster";
    upper->name = "upper_stack";
    payload->name = "payload";

    const KspHints hints = parse_hints(root);
    std::set<int> explicit_payload_indices;
    const bool has_explicit_payload = parse_explicit_payload(root, payload, &explicit_payload_indices);
    if (has_explicit_payload && payload->current_mass_kg <= 0.0) {
        payload->dry_mass_kg = 0.0;
        payload->resource_initial_kg = 0.0;
        payload->resource_capacity_kg = 0.0;
    }

    for (const auto& part : parts) {
        const PhysicalClass physical_class = classify_part(part, hints);
        if (physical_class == PhysicalClass::Support) {
            continue;
        }

        const bool explicit_payload_part =
            has_explicit_payload && explicit_payload_indices.find(part.index) != explicit_payload_indices.end();
        if (explicit_payload_part) {
            if (payload->current_mass_kg <= 0.0) {
                add_part_to_stage(part, payload);
            }
            continue;
        }

        if (!has_explicit_payload && is_payload_candidate(part, physical_class)) {
            add_part_to_stage(part, payload);
            continue;
        }

        if (physical_class == PhysicalClass::Booster) {
            add_part_to_stage(part, booster);
        } else {
            add_part_to_stage(part, upper);
        }
    }

    if (has_explicit_payload) {
        payload->dry_mass_kg = std::max(0.0, payload->current_mass_kg);
    } else {
        payload->dry_mass_kg = std::max(0.0, payload->current_mass_kg);
    }
    payload->resource_initial_kg = 0.0;
    payload->resource_capacity_kg = 0.0;
    payload->resource_names.clear();
    payload->thrust_vac_n = 0.0;
    payload->thrust_sl_n = 0.0;
    payload->isp_vac_weighted = 0.0;
    payload->isp_sl_weighted = 0.0;
}

std::string imported_vehicle_name(const JsonValue& root)
{
    if (const JsonValue* vehicle = member(root, "vehicle")) {
        if (vehicle->is_object()) {
            std::string name;
            if (read_string(*vehicle, "ship_name", &name) && !name.empty()) {
                return name;
            }
        }
    }
    return "KSP vehicle import";
}

std::string source_format(const JsonValue& root)
{
    if (const JsonValue* metadata = member(root, "metadata")) {
        if (metadata->is_object()) {
            std::string format;
            if (read_string(*metadata, "format", &format)) {
                return format;
            }
        }
    }
    return "";
}

post2::vehicle::StageConfig payload_stage_config(const StageAccum& payload)
{
    post2::vehicle::StageConfig stage;
    stage.name = "payload";
    stage.active = false;
    stage.attached = true;
    stage.dry_mass_kg = std::max(0.0, payload.dry_mass_kg);
    stage.engine.enabled = false;
    stage.engine.thrust_vac_n = 0.0;
    stage.engine.isp_vac_s = 0.0;
    stage.engine.feed_tanks.clear();
    stage.tanks.clear();
    return stage;
}

post2::vehicle::VehicleConfig vehicle_from_accums(
    const JsonValue& root,
    const post2::vehicle::AeroConfig& preserved_aero,
    const StageAccum& booster,
    const StageAccum& upper,
    const StageAccum& payload)
{
    post2::vehicle::VehicleConfig vehicle = post2::vehicle::default_vehicle_config();
    vehicle.name = imported_vehicle_name(root);
    vehicle.aero = preserved_aero;
    vehicle.tank_to_tank_connections.clear();
    vehicle.stages.clear();
    vehicle.stages.push_back(stage_config_from_accum(booster, true));
    vehicle.stages.push_back(stage_config_from_accum(upper, false));
    vehicle.stages.push_back(payload_stage_config(payload));

    double dry_mass = 0.0;
    for (const auto& stage : vehicle.stages) {
        dry_mass += std::max(0.0, stage.dry_mass_kg);
    }
    if (dry_mass <= 0.0) {
        if (const JsonValue* vehicle_object = member(root, "vehicle")) {
            if (vehicle_object->is_object()) {
                read_number(*vehicle_object, "dry_mass_kg", &dry_mass);
            }
        }
    }
    vehicle.dry_mass_kg = std::max(1.0, dry_mass);
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&vehicle);
    return vehicle;
}

} // namespace

bool ksp_vehicle_site_import_from_json(
    const std::string& text,
    const post2::vehicle::AeroConfig& preserved_aero,
    KspVehicleSiteImport* result,
    std::string* error)
{
    if (!result) {
        return fail(error, "KSP import result is null");
    }

    std::string json_text = text;
    if (json_text.size() >= 3 &&
        static_cast<unsigned char>(json_text[0]) == 0xEF &&
        static_cast<unsigned char>(json_text[1]) == 0xBB &&
        static_cast<unsigned char>(json_text[2]) == 0xBF) {
        json_text.erase(0, 3);
    }

    JsonValue parsed;
    if (!parse_json(json_text, &parsed, error)) {
        return false;
    }
    const JsonValue root = unwrap_kos_json(parsed);
    if (!root.is_object()) {
        return fail(error, "KSP vehicle/site JSON root must be an object");
    }

    KspVehicleSiteImport imported;
    imported.source_format = source_format(root);
    parse_launch_site(root, &imported);

    StageAccum booster;
    StageAccum upper;
    StageAccum payload;
    const std::vector<KspPart> parts = parse_parts(root);
    if (!parts.empty()) {
        build_from_parts(root, parts, &booster, &upper, &payload);
    } else if (!parse_stage_summaries(root, &booster, &upper, error)) {
        return false;
    } else {
        std::set<int> unused_payload_indices;
        parse_explicit_payload(root, &payload, &unused_payload_indices);
        payload.dry_mass_kg = std::max(0.0, payload.current_mass_kg);
    }
    if (payload.name.empty()) {
        payload.name = "payload";
    }

    imported.vehicle = vehicle_from_accums(root, preserved_aero, booster, upper, payload);
    if (!post2::vehicle::validate_vehicle_config(imported.vehicle, error)) {
        return false;
    }

    *result = std::move(imported);
    return true;
}

bool load_ksp_vehicle_site_import_file(
    const std::string& path,
    const post2::vehicle::AeroConfig& preserved_aero,
    KspVehicleSiteImport* result,
    std::string* error)
{
    std::ifstream input(path);
    if (!input) {
        return fail(error, "failed to open KSP vehicle/site JSON: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return ksp_vehicle_site_import_from_json(buffer.str(), preserved_aero, result, error);
}

void apply_ksp_vehicle_site_import(CaseConfig* config, const KspVehicleSiteImport& imported)
{
    if (!config) {
        return;
    }
    config->vehicle = imported.vehicle;
    if (imported.has_launch_site) {
        config->launch_site = imported.launch_site;
    }
    if (imported.has_earth_radius_m) {
        config->earth_radius_m = imported.earth_radius_m;
    }
    if (imported.has_earth_mu_m3s2) {
        config->earth_mu_m3s2 = imported.earth_mu_m3s2;
    }
    if (imported.has_earth_rotation_rad_per_s) {
        config->earth_rotation_rad_per_s = imported.earth_rotation_rad_per_s;
    }
}

void apply_ksp_vehicle_site_import(SimulationConfig* config, const KspVehicleSiteImport& imported)
{
    if (!config) {
        return;
    }
    config->vehicle = imported.vehicle;
    if (imported.has_launch_site) {
        config->launch_site = imported.launch_site;
    }
    if (imported.has_earth_radius_m) {
        config->earth_radius_m = imported.earth_radius_m;
    }
    if (imported.has_earth_mu_m3s2) {
        config->earth_mu_m3s2 = imported.earth_mu_m3s2;
    }
    if (imported.has_earth_rotation_rad_per_s) {
        config->earth_rotation_rad_per_s = imported.earth_rotation_rad_per_s;
    }
}

} // namespace post2::core
