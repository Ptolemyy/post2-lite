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
    // "exponential" (default), "us_standard_1976", or "table".
    std::string type = "exponential";
    // CSV path when type == "table": altitude_m,density_kgpm3,pressure_pa,
    // temperature_k[,speed_of_sound_mps].
    std::string table_path;
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

// Shared between phase termination and case-level mission events. For phase
// termination the "time" type is phase-relative (preserves duration_s
// semantics). For mission events the "time" type is mission-absolute.
struct TriggerCondition {
    // "time" | "altitude_m" | "velocity_mps" | "total_mass_kg" | "propellant_mass_kg"
    std::string type = "time";
    // ">=" or "<="
    std::string comparison = ">=";
    double value = kDefaultDurationS;
};

// Case-level non-sequential event. Trigger evaluates on every integrator
// step boundary; the actions vector is applied atomically at the first
// rising edge (prev_g < 0, g_now >= 0). Subsequent rising edges re-fire.
struct EventConfig {
    std::string name = "event";
    bool enabled = true;
    TriggerCondition trigger;
    std::vector<PhaseAction> actions;
};

struct Poly2Config {
    double c0 = 0.0;
    double c1 = 0.0;
    double c2 = 0.0;
    // When true, at a phase transition the constant term c0 is overwritten so
    // this angle continues smoothly from the previous phase's final attitude.
    bool continuity = false;
};

struct SegmentedPolySegmentConfig {
    double start_time_s = 0.0;
    std::vector<double> coefficients;
};

struct SegmentedPolyConfig {
    int order = 1;
    bool continuity = true;
    std::vector<SegmentedPolySegmentConfig> segments;
};

struct SegmentedSteeringPolySegmentConfig {
    double start_time_s = 0.0;
    std::vector<double> azimuth_coefficients;
    std::vector<double> elevation_coefficients;
};

struct SegmentedSteeringPolyConfig {
    int order = 1;
    bool continuity = true;
    std::vector<SegmentedSteeringPolySegmentConfig> segments;
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
    // When true, at a phase transition the model is re-anchored so throttle
    // continues from the previous phase's final value: poly sets c0, the
    // interpolated model sets its first point, and the T/W model sets the
    // target ratio to the previous state's actual thrust-to-weight.
    bool continuity = false;
    std::vector<ThrottlePoint> points;
    SegmentedPolyConfig segmented_poly;
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

// Linear / bilinear tangent steering (the vacuum-optimal pitch law). The
// elevation angle follows tan(elevation) = a_value*dt + b_value, with
// a_value = a + a_dot*dt and b_value = b + b_dot*dt (KSPTOT LinearTangentModel),
// where dt = phase_time_s + t_offset_s. "linear_tangent" holds a_dot = b_dot = 0
// (tan linear in time); "bilinear_tangent" uses the full quadratic-in-tan form.
// Azimuth is still taken from azimuth_deg (a poly). Cannot represent a vertical
// (90 deg) elevation, so it belongs on the pitched-over upper-stage phases.
struct LinearTangentConfig {
    double a = 0.0;
    double a_dot = 0.0;
    double b = 0.0;
    double b_dot = 0.0;
    double t_offset_s = 0.0;
    // When true, b is re-anchored at a phase boundary to tan(previous final
    // elevation) so the pitch law continues smoothly (KSPTOT setBForContinuity).
    bool continuity = false;
};

// NASA-standard Unified Powered Flight Guidance (steering type "upfg"). The
// closed-loop backup to the open-loop polynomial steering: instead of a fixed
// attitude program it computes, from the live state, the thrust direction that
// flies the vehicle to the target orbit. Insertion is at the target periapsis
// (flight-path angle 0); periapsis == apoapsis gives a circular orbit. Use on
// vacuum/upper-stage phases only (it ignores drag and can command large
// pitch-overs in-atmosphere) and pair with a full-throttle phase (it assumes
// full vacuum thrust per remaining stage). See post2/core/upfg.hpp.
struct UpfgConfig {
    double periapsis_km = 200.0;
    double apoapsis_km = 200.0;
    double inclination_deg = kDefaultInclinationDeg;
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
    LinearTangentConfig tangent;
    UpfgConfig upfg;
    Vec3 fixed_direction_eci = {1.0, 0.0, 0.0};
    std::vector<QuaternionPoint> points;
    std::vector<SelectableSteeringSegment> segments;
    SegmentedSteeringPolyConfig segmented_poly;
};

struct PhaseConfig {
    std::string name = "default";
    // Polymorphic termination condition. For type == "time" the value is the
    // phase duration in seconds (phase-relative). For altitude/velocity/mass
    // types, the phase ends when the comparison first becomes true.
    TriggerCondition termination{"time", ">=", kDefaultDurationS};
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

struct OptimizationEnvelopeSearchConfig {
    bool enabled = false;
    int sample_count = 16;
    int seed = 1;
};

struct OptimizationContinuationConfig {
    bool enabled = false;
    // "variable": ramp an input variable (variable_path) to its feasibility
    // frontier. "objective": epsilon-constraint -- push the enabled objective
    // metric (as an upper/lower bound target) toward its optimum via warm-started
    // target-only solves. Use "objective" for output-metric objectives such as
    // minimize(max_q_pa) that have no single input lever.
    std::string mode = "variable";
    std::string variable_path;
    std::string direction = "increase";
    int steps = 8;
    bool multistart_enabled = false;
    int multistart_count = 2;
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
    std::vector<OptimizationObjectiveConfig> objectives;
    // Legacy single-objective surface. When objectives is non-empty, the
    // vector is authoritative; this member is kept for older case files and
    // CLI/UI paths that still set one objective.
    OptimizationObjectiveConfig objective;
    OptimizationEnvelopeSearchConfig envelope_search;
    OptimizationContinuationConfig continuation;
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
    // Non-sequential events. Evaluated each integrator step boundary; fire
    // on rising edges of the trigger condition.
    std::vector<EventConfig> events;
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
