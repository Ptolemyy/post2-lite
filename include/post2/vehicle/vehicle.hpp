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

struct RigidBodyConfig {
    // Scalar pitch-axis inertia used by the planar 2.5-DOF rigid-body model.
    // A value <= 0 is ignored by 3-DOF phases but rejected by 2.5-DOF phases.
    double moment_of_inertia_kgm2 = 0.0;
    double initial_attitude_rad = 0.0;
    double initial_angular_velocity_radps = 0.0;
    // Distance from the center of mass to the engine thrust line along the
    // negative body-X axis. Lateral thrust from engine.direction_body produces
    // pitch torque tau = r x F in the 2.5-DOF model.
    double engine_moment_arm_m = 0.0;
};

struct RigidBodyState {
    double attitude_rad = 0.0;
    double angular_velocity_radps = 0.0;
    double moment_of_inertia_kgm2 = 0.0;
};

struct RigidBodyDerivative {
    double attitude_radps = 0.0;
    double angular_acceleration_radps2 = 0.0;
};

struct TankRef {
    std::string stage_name;
    std::string tank_name;
};

bool operator==(const TankRef& lhs, const TankRef& rhs);
bool operator!=(const TankRef& lhs, const TankRef& rhs);

struct EngineThrottleCurvePoint {
    double throttle = 0.0;    // commanded throttle in [0, 1]
    double mdot_ratio = 0.0;  // actual mdot / vacuum mdot at commanded throttle
};

struct EngineConfig {
    bool enabled = false;

    // Per-engine vacuum performance. The cluster (engine_count engines)
    // aggregated values are F_cluster = engine_count * thrust_vac_n,
    // mdot_cluster = engine_count * (thrust_vac_n / (isp_vac_s * g0)).
    double thrust_vac_n = 0.0;
    double isp_vac_s = 0.0;

    // Optional sea-level reference. If thrust_sl_n > 0 it is used for the
    // case_config_io legacy alias migration and as a sanity check (we expect
    // thrust_sl_n ~= thrust_vac_n - p_sl * nozzle_exit_area_m2). Not used at
    // runtime: the physical model uses (thrust_vac_n, nozzle_exit_area_m2,
    // ambient_pressure_pa).
    double thrust_sl_n = 0.0;
    double isp_sl_s = 0.0;

    // Nozzle exit area for pressure-corrected thrust F(p) = F_vac - p * Ae.
    // When zero the engine behaves as a constant-thrust model (legacy).
    double nozzle_exit_area_m2 = 0.0;

    // Throttle bounds. Commanded throttle is clamped to
    // [min_throttle, max_throttle] before being mapped through throttle_curve.
    // Defaults of 0/1 preserve the legacy "any throttle in [0, 1]" behavior.
    double min_throttle = 0.0;
    double max_throttle = 1.0;

    // Optional non-linear throttle map. Empty -> mdot_ratio = throttle (linear).
    // Otherwise sorted by .throttle ascending and piecewise-linearly
    // interpolated; values outside the table clamp to the endpoints.
    std::vector<EngineThrottleCurvePoint> throttle_curve;

    // Transient thrust dynamics. After the engine is commanded on (off->on
    // edge) it stays dead for ignition_delay_s producing no thrust, then the
    // actual throttle spools up from zero toward the commanded throttle. The
    // spool is a first-order lag,
    //     d(theta_actual)/dt = rate * (theta_cmd - theta_actual),
    // with rate = spool_up_rate_per_s when accelerating (theta_cmd >=
    // theta_actual) and spool_down_rate_per_s when decelerating. Units are
    // throttle fraction per second; values derived from the KSP spool capture
    // (~3.5 /s, ignition_delay ~2.4 s reproduces the ~3.7 s cold-start build to
    // full thrust). A rate <= 0 means "instantaneous" (legacy, no transient).
    // The same model drives stage ignitions (first stage on the pad, upper
    // stage after separation) and any in-flight throttle change.
    double ignition_delay_s = 0.0;
    double spool_up_rate_per_s = 0.0;
    double spool_down_rate_per_s = 0.0;
    // Legacy buildup-duration field, kept for config compatibility. Superseded
    // by the spool-rate model above; not consulted at runtime.
    double thrust_buildup_s = 0.0;
    double shutdown_delay_s = 0.0;

    // Number of engines in this stage's cluster. Thrust and mass flow scale
    // linearly. Default 1 reproduces the pre-cluster (aggregated) behavior.
    int engine_count = 1;

    // Allowed numbers of engines that may be ignited together (a discrete
    // cluster-ignition constraint). E.g. Falcon 9 = {1, 3, 9}: the center
    // engine, the 3-engine entry/landing set, or all 9. Empty means "any count
    // from 1..engine_count is allowed" (no discrete restriction). Values are
    // clamped to [1, engine_count]; used by throttle models that pick how many
    // engines to light (e.g. the re-entry burn) to realise a commanded thrust.
    std::vector<int> ignition_count_options;

    // Gimbal envelope. Fields are persisted but not enforced at runtime
    // (enforcing the half-angle cone requires the vehicle body axis, which
    // depends on the attitude state that is out of scope here).
    double gimbal_max_rad = 0.0;
    double gimbal_rate_rad_s = 0.0;

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

// One aerodynamic table for a particular staging configuration. Each table
// describes the contiguous range of still-attached stage indices it represents:
//   * activate_at_min_attached_stage = the LOWEST still-attached stage index.
//   * max_attached_stage             = the HIGHEST still-attached stage index,
//     or -1 meaning "open to the top of the stack".
//
// Two families are generated (see generate_case_aero_tables):
//   * Upper-stack / full tables (max_attached_stage == -1): the configurations
//     the ascending vehicle flies through as lower stages separate. The full
//     stack is [0, top]; after stage k separates the stack is [k, top]. These
//     are selected whenever the topmost stage is still attached.
//   * Single-stage tables (max_attached_stage == activate_at_min_attached_stage):
//     one separated stage flying on its own (booster recovery / flyback), built
//     from that stage's own geometry with no fairing.
struct AeroStageTable {
    int activate_at_min_attached_stage = 0;
    std::string table_path;
    double reference_area_m2 = 0.0;
    // Geometry used to build this table (for display / regeneration).
    double ref_diameter_m = 0.0;
    double body_length_m = 0.0;
    double nose_length_m = 0.0;
    double base_diameter_m = 0.0;
    // Highest still-attached stage index this table represents; -1 == open to
    // the top of the stack. Appended before nose_radius_m to keep aggregate
    // initialisers that predate this field valid (they default it to -1).
    int max_attached_stage = -1;
    // Stagnation-point nose radius [m] for the heat-flux diagnostic in this
    // configuration. Zero means "auto" (derived from this table's ref_diameter_m).
    // Appended last so older positional aggregate initialisers stay valid.
    double nose_radius_m = 0.0;
};

// Deployed grid fins on a returning booster. count = number of fins (片数);
// area_per_fin_m2 = planform reference area of one fin (参考面积).
struct GridFinConfig {
    int count = 0;
    double area_per_fin_m2 = 0.0;

    double total_area_m2() const {
        return count > 0 ? count * area_per_fin_m2 : 0.0;
    }
};

struct AeroConfig {
    bool enabled = false;
    double reference_area_m2 = 10.0;
    double cd = 0.5;   // constant-CD fallback when no table is used
    double cl = 0.0;   // constant-CL fallback
    std::string aero_table_path;

    // When true, drag/lift use tabulated CD/CL(Mach, alpha). The active table is
    // chosen per staging configuration from stage_tables (falling back to
    // aero_table_path when stage_tables is empty), instead of the constant cd/cl.
    bool use_table = false;

    // Per-configuration tables, ascending by activate_at_min_attached_stage.
    std::vector<AeroStageTable> stage_tables;

    // Aerodynamic table for the first stage flying on its own (booster-only
    // geometry). Retained for tooling / back-compat; it now mirrors the
    // first-stage single-stage entry that also lives in stage_tables (with
    // max_attached_stage == 0), which is what the force model actually selects.
    // table_path empty means "not generated".
    AeroStageTable first_stage_table;

    // Geometry the offline generator used / will use to (re)build the level-0
    // (full-stack) table. Zero means "unset" (generator falls back to defaults
    // derived from the reference area). Lengths in metres.
    double ref_diameter_m = 0.0;
    double body_length_m = 0.0;
    double nose_length_m = 0.0;
    double base_diameter_m = 0.0;

    // Stagnation-point nose radius [m] for the Sutton-Graves aerodynamic
    // heat-flux diagnostic. Zero means "auto" (derived from ref_diameter_m).
    double nose_radius_m = 0.0;

    // Base-first descent aerodynamics (booster reentry). The semi-empirical
    // CD/CL tables only cover slender nose-first flight (alpha 0..20 deg); a
    // returning booster flies engine-first (alpha ~180 deg), a bluff body whose
    // drag the tables grossly under-predict. When descent_cd > 0 and the vehicle
    // is flying base-first (alpha > 90 deg), the drag model uses this bluff-body
    // coefficient (referenced to reference_area_m2) instead of the table value,
    // plus the deployed grid-fin drag below. Zero keeps the legacy table-only drag.
    double descent_cd = 0.0;

    // Deployed grid fins on the returning booster (lattice control surfaces).
    // count = number of fins (片数), area_per_fin_m2 = planform area of one fin
    // (参考面积). Their Mach-dependent drag/lift is computed by the post2::aero
    // grid-fin model: baked into the booster-only aero table at generation time,
    // and added to the base-first descent drag at runtime. count <= 0 disables.
    GridFinConfig grid_fins;
};

struct VehicleConfig {
    std::string name = "default";
    double dry_mass_kg = 1000.0;
    RigidBodyConfig rigid_body;
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
    RigidBodyState rigid_body;
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
    // Transient spool state. spool_throttle is the engine's actual (lagged)
    // throttle fraction; it relaxes toward the commanded throttle per the
    // first-order model in EngineConfig. ignition_time_s is the absolute time
    // the engine last transitioned off->on (negative when not ignited), used to
    // apply the ignition_delay dead-time before the spool begins.
    double spool_throttle = 0.0;
    double ignition_time_s = -1.0;
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
