#pragma once

#include <optional>
#include <string>
#include <vector>

namespace post2::vehicle {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct CartesianState6D {
    Vec3 position_m;
    Vec3 velocity_mps;
};

struct CartesianStateDerivative6D {
    Vec3 d_position_mps;
    Vec3 d_velocity_mps2;
};

struct TankRef {
    std::string stage_name;
    std::string tank_name;
};

bool operator==(const TankRef& lhs, const TankRef& rhs);
bool operator!=(const TankRef& lhs, const TankRef& rhs);

struct EngineConfig {
    bool enabled = false;
    double max_thrust_n = 0.0;
    double isp_s = 0.0;
    Vec3 direction_body = {1.0, 0.0, 0.0};
    // Priority-ordered list of tanks feeding this engine. The first feed_tank
    // with mass > eps takes the full draw; flow smoothly transfers to the
    // next as the first nears empty. Validation requires this to be
    // non-empty whenever the engine is enabled with positive thrust on an
    // active stage.
    std::vector<TankRef> feed_tanks;
};

struct TankConfig {
    std::string name = "main";
    std::string propellant = "generic";
    double capacity_kg = 0.0;
    double initial_kg = 0.0;
};

struct StageConfig {
    std::string name = "stage 1";
    bool active = true;
    bool attached = true;
    double dry_mass_kg = 0.0;
    EngineConfig engine;
    std::vector<TankConfig> tanks = {TankConfig{}};
};

struct T2TConnection {
    TankRef source;
    TankRef dest;
    double rate_kgps = 0.0;
    std::optional<double> start_time_s;
    std::optional<double> end_time_s;
};

struct AeroConfig {
    bool enabled = false;
    double reference_area_m2 = 10.0;
    double cd = 0.5;
    double cl = 0.0;
    std::string aero_table_path;
};

struct VehicleConfig {
    std::string name = "default";
    double dry_mass_kg = 1000.0;
    // Legacy single-stage fields. New configs should use stages; these remain
    // as the compatibility surface for old CLI/remote/config paths.
    EngineConfig engine;
    std::vector<TankConfig> tanks = {TankConfig{}};
    std::vector<StageConfig> stages;
    std::vector<T2TConnection> tank_to_tank_connections;
    AeroConfig aero;
};

struct VehicleState {
    CartesianState6D motion;
    double dry_mass_kg = 0.0;
    double propellant_mass_kg = 0.0;
    double total_mass_kg = 0.0;
};

struct EngineState {
    bool enabled = false;
    bool firing = false;
    double throttle = 0.0;
    double commanded_thrust_n = 0.0;
    double actual_thrust_n = 0.0;
    double isp_s = 0.0;
    double mass_flow_kgps = 0.0;
    Vec3 direction_body = {1.0, 0.0, 0.0};
};

struct TankState {
    std::string name = "main";
    std::string propellant = "generic";
    double capacity_kg = 0.0;
    double remaining_kg = 0.0;
};

struct HoldDownClampState {
    bool active = false;
    Vec3 planet_fixed_position_m;
};

struct StageRuntimeState {
    std::string name = "stage 1";
    bool active = true;
    bool attached = true;
    double dry_mass_kg = 0.0;
    EngineState engine;
    std::vector<TankState> tanks;
};

struct VehicleRuntimeState {
    double time_s = 0.0;
    VehicleState vehicle;
    EngineState engine;
    HoldDownClampState hold_down_clamp;
    std::vector<TankState> tanks;
    std::vector<StageRuntimeState> stages;
};

struct Vehicle {
    VehicleConfig config;
    VehicleRuntimeState runtime;
};

Vec3 operator+(const Vec3& lhs, const Vec3& rhs);
Vec3 operator-(const Vec3& lhs, const Vec3& rhs);
Vec3 operator*(const Vec3& value, double scale);
Vec3 operator*(double scale, const Vec3& value);
Vec3 operator/(const Vec3& value, double scale);

double dot(const Vec3& lhs, const Vec3& rhs);
double norm(const Vec3& value);

} // namespace post2::vehicle
