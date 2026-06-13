#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "post2/core/state_log.hpp"
#include "post2/integrators/integrator.hpp"
#include "post2/vehicle/vehicle.hpp"

namespace post2::core {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kEarthMuM3S2 = 3.986004418e14;
constexpr double kEarthJ2 = 1.08262668e-3;
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

struct EpochUtc {
    int year = 2000;
    int month = 1;
    int day = 1;
    int hour = 12;
    int minute = 0;
    double second = 0.0;
};

struct HoldDownClampConfig {
    bool enabled = false;
    double release_time_s = 0.0;
};

struct NormalForceConfig {
    bool enabled = true;
};

struct GravityModelConfig {
    std::string type = "j2";
    double j2 = kEarthJ2;
    int degree = 0;
    int order = 0;
};

struct AtmosphereModelConfig {
    std::string type = "exponential";
};

struct AeroModelConfig {
    std::string type = "none";
};

struct ForceModelSwitches {
    bool gravity = true;
    bool thrust = true;
    bool normal_force = true;
    bool aerodynamic = false;
    bool third_body = false;
    bool solar_radiation_pressure = false;

    GravityModelConfig gravity_model;
    AtmosphereModelConfig atmosphere_model;
    AeroModelConfig aero_model;
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
    // "rk4" (fixed step), "dopri5" (adaptive 5/4), or legacy "ode" alias for rk4.
    std::string integrator = "rk4";
    post2::integrators::IntegratorTolerances tolerances;
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
    std::string scope = "terminal";
    int phase_index = -1;
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
    std::string qp_solver = "kkt-fallback";
    std::string fd_mode = "auto";
    int max_iterations = 100;
    double tolerance = 1.0e-4;
    double stationarity_tolerance = -1.0;
    double feasibility_tolerance = -1.0;
    double constraint_tolerance = -1.0;
    double initial_step_fraction = 0.25;
    bool parallel_fd = true;
    int max_restoration_iterations = 8;
    std::vector<OptimizationVariableConfig> variables;
    std::vector<OptimizationTargetConfig> targets;
    OptimizationObjectiveConfig objective;
};

struct CaseConfig {
    std::string name = "default";
    post2::vehicle::VehicleConfig vehicle;
    LaunchSiteConfig launch_site;
    EpochUtc epoch_utc;
    double earth_radius_m = kEarthRadiusM;
    double earth_mu_m3s2 = kEarthMuM3S2;
    double earth_j2 = kEarthJ2;
    double earth_rotation_rad_per_s = kEarthRotationRadPerS;
    // GMST at epoch (radians). Derived from epoch_utc during case load and
    // cached here so per-step transforms avoid recomputing JD/GMST.
    double earth_rotation_at_epoch_rad = 0.0;
    double step_s = kDefaultStepS;
    std::vector<PhaseConfig> phases;
    OptimizationConfig optimization;
};

struct SimulationConfig {
    EpochUtc epoch_utc;
    double earth_radius_m = kEarthRadiusM;
    double earth_mu_m3s2 = kEarthMuM3S2;
    double earth_j2 = kEarthJ2;
    double earth_rotation_rad_per_s = kEarthRotationRadPerS;
    double earth_rotation_at_epoch_rad = 0.0;
    GravityModelConfig gravity_model;
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
    bool found_feasible = false;
    std::string error;
    int iterations = 0;
    int evaluations = 0;
    double best_score = 0.0;
    double max_constraint_violation = 0.0;
    double l1_constraint_violation = 0.0;
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
