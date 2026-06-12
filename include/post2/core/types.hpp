#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "post2/core/state_log.hpp"
#include "post2/vehicle/vehicle.hpp"

namespace post2::core {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kEarthMuM3S2 = 3.986004418e14;
constexpr double kEarthRotationRadPerS = 7.2921159e-5;
constexpr double kDefaultAltitudeM = 200000.0;
constexpr double kDefaultInclinationDeg = 28.5;
constexpr double kDefaultDurationS = 5400.0;
constexpr double kDefaultStepS = 10.0;
constexpr double kDefaultLaunchLatitudeDeg = 28.5;
constexpr double kDefaultLaunchLongitudeDeg = 0.0;

using Vec3 = post2::vehicle::Vec3;
using State = post2::vehicle::CartesianState6D;
using StateDerivative = post2::vehicle::CartesianStateDerivative6D;
using Vehicle = post2::vehicle::Vehicle;
using TrajectoryPoint = LaunchVehicleStateLogEntry;

struct LaunchSiteConfig {
    double latitude_deg = kDefaultLaunchLatitudeDeg;
    double longitude_deg = kDefaultLaunchLongitudeDeg;
    double altitude_m = 0.0;
};

struct HoldDownClampConfig {
    bool enabled = false;
    double release_time_s = 0.0;
};

struct NormalForceConfig {
    bool enabled = true;
};

struct ForceModelSwitches {
    bool gravity = true;
    bool thrust = true;
    bool normal_force = true;
};

struct PhaseAction {
    double time_s = 0.0;
    std::string type = "set_engine_enabled";
    bool value = false;
    int stage_index = -1;
    std::string stage_name;
};

struct Poly2Config {
    double c0 = 0.0;
    double c1 = 0.0;
    double c2 = 0.0;
};

struct ThrottlePoint {
    double time_s = 0.0;
    double throttle = 0.0;
};

struct ThrottleModelConfig {
    std::string type = "poly";
    double c0 = 1.0;
    double c1 = 0.0;
    double c2 = 0.0;
    double target_t2w = 1.0;
    std::vector<ThrottlePoint> points;
};

struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct QuaternionPoint {
    double time_s = 0.0;
    Quaternion quat;
};

struct SteeringModelConfig;

struct SelectableSteeringSegment {
    double start_time_s = 0.0;
    std::shared_ptr<SteeringModelConfig> model;
};

struct SteeringModelConfig {
    std::string type = "generic_poly";
    Poly2Config roll_deg;
    Poly2Config pitch_deg;
    Poly2Config yaw_deg;
    Poly2Config azimuth_deg;
    Poly2Config elevation_deg;
    Vec3 fixed_direction_eci = {1.0, 0.0, 0.0};
    std::vector<QuaternionPoint> points;
    std::vector<SelectableSteeringSegment> segments;
};

struct PhaseConfig {
    std::string name = "default";
    double duration_s = kDefaultDurationS;
    bool optimize_enabled = true;
    bool inherit_initial_state = true;
    std::optional<State> initial_state_eci;
    bool hold_down_clamp_initial_active = false;
    std::string integrator = "ode";
    ForceModelSwitches force_models;
    ThrottleModelConfig throttle_model;
    SteeringModelConfig steering_model;
    std::vector<PhaseAction> actions;
};

struct OptimizationVariableConfig {
    std::string path;
    bool enabled = false;
    double min_value = 0.0;
    double max_value = 0.0;
};

struct OptimizationTargetConfig {
    std::string metric = "terminal_altitude_m";
    std::string mode = "equal";
    double value = 0.0;
    double min_value = 0.0;
    double max_value = 0.0;
    double weight = 1.0;
};

struct OptimizationObjectiveConfig {
    bool enabled = false;
    std::string metric = "terminal_altitude_m";
    std::string direction = "minimize";
    double weight = 1.0;
};

struct OptimizationConfig {
    std::string mode = "target";
    std::string optimizer = "fmincon";
    int max_iterations = 100;
    double tolerance = 1.0e-4;
    double initial_step_fraction = 0.25;
    std::vector<OptimizationVariableConfig> variables;
    std::vector<OptimizationTargetConfig> targets;
    OptimizationObjectiveConfig objective;
};

struct CaseConfig {
    std::string name = "default";
    post2::vehicle::VehicleConfig vehicle;
    LaunchSiteConfig launch_site;
    double earth_radius_m = kEarthRadiusM;
    double earth_mu_m3s2 = kEarthMuM3S2;
    double earth_rotation_rad_per_s = kEarthRotationRadPerS;
    double step_s = kDefaultStepS;
    std::vector<PhaseConfig> phases;
    OptimizationConfig optimization;
};

struct SimulationConfig {
    double earth_radius_m = kEarthRadiusM;
    double earth_mu_m3s2 = kEarthMuM3S2;
    double earth_rotation_rad_per_s = kEarthRotationRadPerS;
    double initial_altitude_m = kDefaultAltitudeM;
    double initial_speed_mps = 0.0;
    double inclination_deg = kDefaultInclinationDeg;
    double duration_s = kDefaultDurationS;
    double step_s = kDefaultStepS;
    LaunchSiteConfig launch_site;
    HoldDownClampConfig hold_down_clamp;
    NormalForceConfig normal_force;
    post2::vehicle::VehicleConfig vehicle;
};

struct SimulationResult {
    bool ok = false;
    std::string error;
    StateLog state_log;
};

struct OptimizationRunOptions {
    int max_iterations = -1;
    double tolerance = -1.0;
    double initial_step_fraction = -1.0;
    bool run_final_simulation = true;
};

struct OptimizationVariableChange {
    std::string path;
    double old_value = 0.0;
    double new_value = 0.0;
};

struct OptimizationMetricValue {
    std::string metric;
    double value = 0.0;
};

struct OptimizationResult {
    bool ok = false;
    std::string error;
    int iterations = 0;
    int evaluations = 0;
    double best_score = 0.0;
    std::vector<std::string> messages;
    std::vector<OptimizationVariableChange> variable_changes;
    std::vector<OptimizationMetricValue> final_metrics;
    SimulationResult final_simulation;
};

CaseConfig case_from_simulation_config(const SimulationConfig& config);
SimulationConfig simulation_config_from_case(const CaseConfig& config);

State make_default_leo_state(const SimulationConfig& config);
double circular_orbit_speed_mps(double mu_m3s2, double radius_m);

} // namespace post2::core
