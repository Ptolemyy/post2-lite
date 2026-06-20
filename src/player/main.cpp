// post2_player -- standalone C++ ascent-guidance player for KSP via kRPC.
//
// Reads a guidance-script CSV (post2::core::guidance_script_to_csv), then re-flies
// the ascent: it evaluates the per-phase steering polynomials itself and runs the
// real NASA UPFG (post2/core/upfg.hpp) for UPFG phases, commanding attitude,
// throttle and staging through the kRPC autopilot. Replaces the old kOS player.
//
//   post2_player <guidance.csv> [options]
//     --dry-run              fly an offline point-mass sim and print the command
//                            stream (no KSP/kRPC connection)
//     --address IP           kRPC server address (default 127.0.0.1)
//     --rpc-port N           kRPC rpc port (default 30000)
//     --stream-port N        kRPC stream port (default 30001)
//     --dt S                 control/integration step (default 0.2 live, 0.5 dry)
//     --no-launch            do not auto-activate the first stage at start (live)

#include "post2/player/guidance_script.hpp"

#include "post2/core/upfg.hpp"
#include "post2/vehicle/vehicle.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include <krpc.hpp>
#include <krpc/services/space_center.hpp>

namespace {

using post2::vehicle::Vec3;
using post2::vehicle::norm;
using post2::vehicle::dot;
using post2::player::GuidanceScript;
using post2::player::ScriptPhase;

constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;
// UPFG cutoff margin: stop this many seconds before UPFG's tgo reaches 0. UPFG
// drives the orbit to the target exactly at tgo=0, so this must be small (about
// one control step) or the burn ends early and periapsis comes up short.
constexpr double kBurnoutMarginS = 0.1;
// UPFG thrust-collapse abort: only give up on the upper-stage burn after thrust
// has been ~zero (thrust_ratio < kThrustEstablishedFloor) continuously for this
// long, so a brief dropout (ullage settling after the coast, a steering slew)
// does not falsely kill the burn on a single sample.
constexpr double kUpfgThrustLossDwellS = 3.0;
constexpr double kThrustEstablishedFloor = 0.05;
// Live staging timing: dwell after MECO (throttle 0) so the spent stage's thrust
// decays before the decoupler fires, then a short coast so it drifts clear
// before the next stage lights.
constexpr double kStageSettleS = 1.0;
constexpr double kStageSeparationCoastS = 0.8;
// After lighting the next stage, hold the separation attitude until its thrust
// has built up to this fraction of available (or the timeout), so the light
// upper stage does not tumble while the engine spools up and before its gimbal
// has authority. Only then is steering handed to the phase guidance.
constexpr double kStageThrustEstablishedFraction = 0.9;
constexpr double kStageThrustEstablishTimeoutS = 8.0;
// During the no-thrust separation/spool window there is no gimbal authority --
// only RCS / reaction wheels. If dynamic pressure is still above this, hold
// prograde so the stage flies at ~zero angle of attack and aero trims it (instead
// of holding a pitched steering attitude that aero would drive further into the
// turn); below it (effectively vacuum) hold the stage's actual attitude so RCS
// only has to null the separation rates rather than slew.
constexpr double kStageAeroHoldQPa = 100.0;
// Payload-fairing jettison: arm once dynamic pressure has risen through maxQ
// (q > kFairingArmQPa) during ascent, then jettison when it falls back to ~0
// (q < kFairingJettisonQPa) above the atmosphere, but only after the upper stage
// is active so the activation hits the fairing decoupler.
constexpr double kFairingArmQPa = 1000.0;
constexpr double kFairingJettisonQPa = 50.0;

Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3 normalized_or(const Vec3& v, const Vec3& fallback)
{
    const double n = norm(v);
    return n <= 1.0e-12 ? fallback : v / n;
}

// Local ENU -> ECI thrust direction. Mirrors
// direction_from_azimuth_elevation_deg in src/core/control_models.cpp so the
// player flies the exact attitude the host integrated.
Vec3 enu_direction(const Vec3& position_m, double azimuth_deg, double elevation_deg)
{
    const Vec3 up = normalized_or(position_m, {1.0, 0.0, 0.0});
    const Vec3 east = normalized_or(cross({0.0, 0.0, 1.0}, up), {0.0, 1.0, 0.0});
    const Vec3 north = normalized_or(cross(up, east), {0.0, 0.0, 1.0});
    const double az = azimuth_deg * kDegToRad;
    const double el = elevation_deg * kDegToRad;
    const double horizontal = std::cos(el);
    return normalized_or(
        north * (horizontal * std::cos(az)) + east * (horizontal * std::sin(az)) +
            up * std::sin(el),
        north);
}

double poly3(const double c[3], double t)
{
    return c[0] + c[1] * t + c[2] * t * t;
}

double eval_segmented(const ScriptPhase& phase, double phase_time_s, bool elevation)
{
    if (phase.segments.empty()) {
        return 0.0;
    }
    const post2::player::ScriptSegment* selected = &phase.segments.front();
    for (const auto& segment : phase.segments) {
        if (phase_time_s + 1.0e-9 >= segment.start_time_s) {
            selected = &segment;
        }
    }
    const double local_t = std::max(0.0, phase_time_s - selected->start_time_s);
    const auto& coeffs = elevation ? selected->elevation_coefficients : selected->azimuth_coefficients;
    double value = 0.0;
    double power = 1.0;
    for (double c : coeffs) {
        value += c * power;
        power *= local_t;
    }
    return value;
}

// Rebuild the UPFG remaining-stage stack from the script's STAGE records and the
// live total mass, starting at the active stage. Mirrors build_upfg_stages in
// control_models.cpp (active stage carries the full live mass; each later stage
// ignites at live_mass minus the dry+propellant of the stages jettisoned below).
std::vector<post2::core::UpfgStage> build_upfg_stages(
    const GuidanceScript& script,
    std::size_t active_stage,
    double live_mass_kg)
{
    std::vector<post2::core::UpfgStage> stages;
    double lower_mass = 0.0;
    for (std::size_t i = active_stage; i < script.stages.size(); ++i) {
        const auto& s = script.stages[i];
        const bool propulsive = s.thrust_n > 0.0 && s.mdot_kgps > 0.0;
        if (!propulsive) {
            continue;  // payload / inert: carried mass, not a burn
        }
        post2::core::UpfgStage stage;
        stage.mode = 1;
        stage.thrust_n = s.thrust_n;
        stage.mdot_kgps = s.mdot_kgps;
        stage.exhaust_velocity_mps = s.exhaust_velocity_mps;
        stage.mass_total_kg = std::max(0.0, live_mass_kg - lower_mass);
        stage.max_burn_time_s = s.propellant_kg / s.mdot_kgps;
        stages.push_back(stage);
        lower_mass += s.dry_mass_kg + s.propellant_kg;
    }
    return stages;
}

struct Command {
    Vec3 thrust_unit_eci = {0.0, 0.0, 1.0};
    double throttle = 1.0;
    double tgo_s = 0.0;
    bool upfg = false;
    bool upfg_ok = true;
};

Command compute_command(
    const GuidanceScript& script,
    const ScriptPhase& phase,
    double phase_time_s,
    const Vec3& position_m,
    const Vec3& velocity_mps,
    double mass_kg,
    std::size_t active_stage)
{
    Command cmd;
    const Vec3 prograde = normalized_or(velocity_mps, normalized_or(position_m, {0.0, 0.0, 1.0}));

    if (phase.is_upfg) {
        cmd.upfg = true;
        cmd.throttle = 1.0;  // UPFG assumes full vacuum thrust
        const double mu = script.meta.mu_m3s2;
        const auto stages = build_upfg_stages(script, active_stage, mass_kg);
        if (stages.empty()) {
            cmd.thrust_unit_eci = prograde;
            cmd.upfg_ok = false;
            return cmd;
        }
        const post2::core::UpfgTarget target = post2::core::make_upfg_orbit_target(
            phase.upfg_periapsis_km * 1000.0,
            phase.upfg_apoapsis_km * 1000.0,
            phase.upfg_inclination_deg,
            position_m,
            velocity_mps,
            mu,
            script.meta.radius_m);
        post2::core::UpfgVehicleState state;
        state.time_s = phase_time_s;
        state.mass_kg = mass_kg;
        state.position_m = position_m;
        state.velocity_mps = velocity_mps;
        const post2::core::UpfgResult result =
            post2::core::upfg_converge(stages, target, state, mu);
        cmd.upfg_ok = result.ok;
        cmd.tgo_s = result.tgo_s;
        cmd.thrust_unit_eci = result.ok ? normalized_or(result.thrust_unit_eci, prograde) : prograde;
        return cmd;
    }

    // Throttle: poly throttle is evaluated; non-poly (e.g. t2w) falls back to
    // full throttle (the host's closed throttle laws are not reconstructed here).
    if (phase.throttle_type == "poly") {
        const double c[3] = {phase.throttle_c0, phase.throttle_c1, phase.throttle_c2};
        cmd.throttle = std::min(1.0, std::max(0.0, poly3(c, phase_time_s)));
    } else {
        cmd.throttle = 1.0;
    }

    double azimuth_deg = 0.0;
    double elevation_deg = 0.0;
    if (!phase.segments.empty()) {
        azimuth_deg = eval_segmented(phase, phase_time_s, false);
        elevation_deg = eval_segmented(phase, phase_time_s, true);
    } else if (phase.has_tangent) {
        const double dt = phase_time_s + phase.tangent_t_offset_s;
        double tan_value;
        if (phase.tangent_bilinear) {
            tan_value = (phase.tangent_a + phase.tangent_a_dot * dt) * dt +
                (phase.tangent_b + phase.tangent_b_dot * dt);
        } else {
            tan_value = phase.tangent_a * dt + phase.tangent_b;
        }
        elevation_deg = std::atan(tan_value) * kRadToDeg;
        azimuth_deg = poly3(phase.azimuth, phase_time_s);
    } else {
        azimuth_deg = poly3(phase.azimuth, phase_time_s);
        elevation_deg = poly3(phase.elevation, phase_time_s);
    }
    cmd.thrust_unit_eci = enu_direction(position_m, azimuth_deg, elevation_deg);
    return cmd;
}

bool compare(double value, const std::string& comparison, double threshold)
{
    if (comparison == "<=") {
        return value <= threshold;
    }
    return value >= threshold;  // default ">="
}

// Whether the given phase's termination condition is satisfied. `metric_value`
// resolves the live metric for altitude/velocity/mass termination types.
bool phase_terminated(
    const ScriptPhase& phase,
    double phase_time_s,
    double altitude_m,
    double speed_mps,
    double mass_kg,
    double propellant_kg,
    double thrust_fraction)
{
    const std::string& type = phase.termination_type;
    double value = phase_time_s;
    if (type == "altitude_m") {
        value = altitude_m;
    } else if (type == "velocity_mps") {
        value = speed_mps;
    } else if (type == "total_mass_kg") {
        value = mass_kg;
    } else if (type == "propellant_mass_kg") {
        value = propellant_kg;
    } else if (type == "thrust_fraction") {
        // Ratio of current thrust to the commanded steady thrust. Drives the
        // pad-hold ignition phase: hold until thrust is established.
        value = thrust_fraction;
    }
    return compare(value, phase.termination_comparison, phase.termination_value);
}

bool action_separates_stage(const post2::player::ScriptAction& action)
{
    // A booster separation is the detach (set_stage_attached -> false). A phase
    // can also carry set_stage_active=false for the same stage; keying only on
    // the detach avoids counting one separation twice.
    return action.type == "set_stage_attached" && !action.value;
}

// ---- Frame remap: kRPC body non-rotating frame <-> post2 ECI ----------------
// post2 ECI uses z = spin axis; kRPC's celestial non-rotating frame uses y =
// north pole and is left-handed. Swapping y<->z aligns the spin axis AND flips
// handedness (LH->RH). This single swap is its own inverse. NOTE: this is the
// expected calibration point against a live KSP -- if the launch azimuth / orbit
// inclination comes out mirrored, flip a horizontal sign here.
Vec3 eci_from_krpc(const std::tuple<double, double, double>& t)
{
    return {std::get<0>(t), std::get<2>(t), std::get<1>(t)};
}
std::tuple<double, double, double> krpc_from_eci(const Vec3& v)
{
    return std::make_tuple(v.x, v.z, v.y);
}

struct Options {
    std::string csv_path;
    bool dry_run = false;
    std::string address = "127.0.0.1";
    unsigned int rpc_port = 30000;
    unsigned int stream_port = 30001;
    double dt_s = -1.0;
    bool auto_launch = true;
};

void print_usage()
{
    std::cerr << "usage: post2_player <guidance.csv> [--dry-run] [--address IP]"
                 " [--rpc-port N] [--stream-port N] [--dt S] [--no-launch]\n"
              << "example: post2_player scripts\\falcon9_min_maxq.guidance.csv --dry-run\n";
}

bool launched_in_private_console()
{
#ifdef _WIN32
    DWORD processes[2] = {};
    const DWORD count = GetConsoleProcessList(processes, 2);
    return count == 1;
#else
    return false;
#endif
}

int exit_player(int code)
{
    if (code != 0 && launched_in_private_console()) {
        std::cerr << "\nPress Enter to exit...";
        std::cerr.flush();
        std::cin.clear();
        std::cin.get();
    }
    return code;
}

std::string read_file(const std::string& path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return {};
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

// ---- Offline dry-run --------------------------------------------------------

int run_dry(const GuidanceScript& script, double dt_s)
{
    const double mu = script.meta.mu_m3s2;
    const double radius = script.meta.radius_m;
    if (!(mu > 0.0) || !(radius > 0.0)) {
        std::cerr << "dry-run needs mu and radius in META\n";
        return 1;
    }

    // Initial state at the launch site, in post2 ECI.
    const double lat = script.meta.launch_lat_deg * kDegToRad;
    const double lon = script.meta.launch_lon_deg * kDegToRad + script.meta.rotation_at_epoch_rad;
    const double r0 = radius + script.meta.launch_alt_m;
    Vec3 position = {r0 * std::cos(lat) * std::cos(lon),
                     r0 * std::cos(lat) * std::sin(lon),
                     r0 * std::sin(lat)};
    const double omega = script.meta.rotation_rad_s;
    Vec3 velocity = {-omega * position.y, omega * position.x, 0.0};

    std::vector<double> prop_remaining;
    double mass = 0.0;
    for (const auto& s : script.stages) {
        prop_remaining.push_back(s.propellant_kg);
        mass += s.dry_mass_kg + s.propellant_kg;
    }
    if (script.stages.empty()) {
        std::cerr << "dry-run needs STAGE records\n";
        return 1;
    }

    std::size_t active_stage = 0;
    std::size_t phase_index = 0;
    double t = 0.0;
    double phase_start = 0.0;
    double next_print = 0.0;
    std::vector<char> actions_done(script.phases.size(), 0);

    std::cout << "# dry-run: " << script.meta.name << "  stages=" << script.stages.size()
              << "  phases=" << script.phases.size() << '\n';
    std::cout << "#   t   phase  alt_km  speed  mass_kg  thr  steer        dir_eci\n";

    const double max_t = 2000.0;
    while (phase_index < script.phases.size() && t < max_t) {
        const ScriptPhase& phase = script.phases[phase_index];
        const double phase_time = t - phase_start;

        // Apply this phase's separation actions once on entry.
        if (!actions_done[phase_index]) {
            for (const auto& action : phase.actions) {
                if (action_separates_stage(action) && active_stage < script.stages.size()) {
                    mass -= script.stages[active_stage].dry_mass_kg + prop_remaining[active_stage];
                    prop_remaining[active_stage] = 0.0;
                    ++active_stage;
                }
            }
            actions_done[phase_index] = 1;
        }

        const Command cmd = compute_command(
            script, phase, phase_time, position, velocity, mass, active_stage);

        const double altitude = norm(position) - radius;
        const double speed = norm(velocity);
        double prop = 0.0;
        for (std::size_t i = active_stage; i < prop_remaining.size(); ++i) {
            prop += prop_remaining[i];
        }

        if (t >= next_print) {
            std::cout.setf(std::ios::fixed);
            std::cout.precision(1);
            std::cout << "  " << t << "   p" << phase_index << "(" << phase.name << ")  "
                      << altitude / 1000.0 << "  " << speed << "  ";
            std::cout.precision(0);
            std::cout << mass << "  ";
            std::cout.precision(2);
            std::cout << cmd.throttle << "  " << (cmd.upfg ? "UPFG" : "poly");
            if (cmd.upfg) {
                std::cout << " tgo=" << cmd.tgo_s;
            }
            std::cout.precision(3);
            std::cout << "  [" << cmd.thrust_unit_eci.x << ", " << cmd.thrust_unit_eci.y
                      << ", " << cmd.thrust_unit_eci.z << "]\n";
            std::cout.unsetf(std::ios::fixed);
            next_print = t + 5.0;
        }

        // Current-stage burn state. The offline model has no spool transient, so
        // thrust is "established" the instant the engine can burn -> thrust_fraction
        // is 1.0 while burning. This makes a thrust_fraction (pad-hold ignition)
        // termination pass through immediately in the dry-run.
        const auto& stage = script.stages[std::min(active_stage, script.stages.size() - 1)];
        const bool burning = active_stage < prop_remaining.size() &&
            prop_remaining[active_stage] > 0.0 && stage.thrust_n > 0.0;
        const double thrust_fraction = burning ? 1.0 : 0.0;

        // Advance phase?
        bool advance = phase_terminated(
            phase, phase_time, altitude, speed, mass, prop, thrust_fraction);
        if (phase.is_upfg && cmd.upfg && cmd.upfg_ok && cmd.tgo_s <= kBurnoutMarginS) {
            advance = true;
        }
        if (active_stage < prop_remaining.size() && prop_remaining[active_stage] <= 0.0 &&
            phase.termination_type != "time") {
            advance = true;  // burnout terminates non-time phases
        }
        if (advance) {
            ++phase_index;
            phase_start = t;
            continue;
        }

        // Integrate one step (point-mass gravity + current-stage thrust). While
        // the hold-down clamp holds the vehicle on the pad, the engine burns but
        // the vehicle does not move (motion is pinned until the clamp releases).
        Vec3 accel = position * (-mu / std::pow(norm(position), 3.0));
        if (burning) {
            const double thrust = cmd.throttle * stage.thrust_n;
            accel = accel + cmd.thrust_unit_eci * (thrust / std::max(mass, 1.0));
            const double dprop = std::min(prop_remaining[active_stage],
                                          cmd.throttle * stage.mdot_kgps * dt_s);
            prop_remaining[active_stage] -= dprop;
            mass -= dprop;
        }
        if (!phase.hold_down_clamp_initial_active) {
            velocity = velocity + accel * dt_s;
            position = position + velocity * dt_s;
        }
        t += dt_s;
    }

    // Final osculating orbit (periapsis/apoapsis) from r, v, mu.
    {
        const double r = norm(position);
        const double v = norm(velocity);
        const double energy = 0.5 * v * v - mu / r;
        const Vec3 h = {position.y * velocity.z - position.z * velocity.y,
                        position.z * velocity.x - position.x * velocity.z,
                        position.x * velocity.y - position.y * velocity.x};
        const double hmag = norm(h);
        const double a = (std::abs(energy) > 1e-12) ? -mu / (2.0 * energy) : 0.0;
        const double e = std::sqrt(std::max(0.0, 1.0 + 2.0 * energy * hmag * hmag / (mu * mu)));
        const double pe_km = (a * (1.0 - e) - radius) / 1000.0;
        const double ap_km = (a * (1.0 + e) - radius) / 1000.0;
        std::cout << "# done: t=" << t << "s  final alt="
                  << (r - radius) / 1000.0 << "km  speed=" << v
                  << "m/s  pe=" << pe_km << "km ap=" << ap_km << "km\n";
    }
    return 0;
}

// Remaining propellant of the active stage, inferred from the live vehicle mass:
// total mass minus the active stage's dry mass and the full (unburned) mass of
// every stage still stacked above it. Drives the propellant_mass_kg termination
// in live flight (KSP gives us mass, not a post2 per-stage propellant figure).
double active_stage_propellant_kg(
    const GuidanceScript& script, std::size_t active_stage, double live_mass_kg)
{
    if (active_stage >= script.stages.size()) {
        return 0.0;
    }
    double upper_mass = 0.0;
    for (std::size_t i = active_stage + 1; i < script.stages.size(); ++i) {
        upper_mass += script.stages[i].dry_mass_kg + script.stages[i].propellant_kg;
    }
    return std::max(0.0, live_mass_kg - script.stages[active_stage].dry_mass_kg - upper_mass);
}

// ---- Live kRPC flight --------------------------------------------------------

int run_live(const GuidanceScript& script, const Options& options)
{
    using SC = krpc::services::SpaceCenter;
    // High guidance rate: UPFG steers so the orbit completes at tgo=0, so a slow
    // loop + a coarse cutoff margin under-burns and drops periapsis. 20 Hz lets
    // the autopilot track the commanded direction and lets us cut close to tgo=0.
    const double dt_s = options.dt_s > 0.0 ? options.dt_s : 0.05;

    std::cout << "connecting to kRPC at " << options.address << ":" << options.rpc_port << " ...\n";
    krpc::Client conn = krpc::connect("post2_player", options.address, options.rpc_port,
                                      options.stream_port);
    SC sc(&conn);
    SC::Vessel vessel = sc.active_vessel();
    SC::CelestialBody body = vessel.orbit().body();
    SC::ReferenceFrame frame = body.non_rotating_reference_frame();
    const double mu = body.gravitational_parameter();
    const double radius = body.equatorial_radius();
    std::cout << "vessel '" << vessel.name() << "' orbiting '" << body.name()
              << "'  mu=" << mu << "  R=" << radius << "\n";

    SC::AutoPilot ap = vessel.auto_pilot();
    SC::Control ctrl = vessel.control();
    ap.set_reference_frame(frame);
    ap.engage();
    ctrl.set_rcs(true);  // RCS on for the whole flight (extra attitude authority)

    if (options.auto_launch) {
        std::cout << "launch: activating first stage\n";
        ctrl.set_throttle(1.0f);
        ctrl.activate_next_stage();
    }

    std::size_t active_stage = 0;
    std::size_t phase_index = 0;
    double phase_start = vessel.met();
    std::vector<char> actions_done(script.phases.size(), 0);
    double next_log = 0.0;
    // Last steering direction commanded (ECI). Held through a staging event so
    // attitude is maintained across separation and engine spool-up.
    Vec3 last_steer_eci = {0.0, 0.0, 1.0};
    bool fairing_armed = false;       // dynamic pressure has passed maxQ
    bool fairing_jettisoned = false;  // fairing already dropped
    // MET at which the UPFG upper-stage thrust first collapsed (< 0 = thrust is
    // present). Used to require a sustained loss before aborting the burn.
    double upfg_thrust_loss_since = -1.0;

    while (phase_index < script.phases.size()) {
        const ScriptPhase& phase = script.phases[phase_index];
        const double met = vessel.met();
        const double phase_time = met - phase_start;

        const Vec3 position = eci_from_krpc(vessel.position(frame));
        const Vec3 velocity = eci_from_krpc(vessel.velocity(frame));
        const double mass = vessel.mass();
        const double altitude = norm(position) - radius;
        const double speed = norm(velocity);
        // Thrust establishment fraction for the pad-hold ignition phase.
        const double available_thrust = vessel.available_thrust();
        const double thrust_ratio = available_thrust > 1.0
            ? vessel.thrust() / available_thrust : 0.0;

        // Apply this phase's staging once on entry. A stage transition in KSP
        // takes two activations: one fires the decoupler to jettison the spent
        // stage, a second ignites the next stage's engine. The script encodes
        // these as a set_stage_attached=false (separation) plus a
        // set_engine_enabled=true on the same phase, so we map a separating phase
        // that also lights an engine to two activate_next_stage() calls.
        if (!actions_done[phase_index]) {
            int separations = 0;
            bool ignites_engine = false;
            for (const auto& action : phase.actions) {
                if (action_separates_stage(action)) {
                    ++separations;
                    if (active_stage + 1 < script.stages.size()) {
                        ++active_stage;
                    }
                }
                if (action.type == "set_engine_enabled" && action.value) {
                    ignites_engine = true;
                }
            }
            if (separations > 0) {
                // MECO before separation: shut the spent stage down and let its
                // thrust decay first, otherwise the still-burning lower stage
                // chases the upper stage once the decoupler fires and collides
                // with it. Settle, fire the decoupler, coast a beat to open a
                // gap, then ignite the next stage. The separation attitude
                // (last_steer_eci) is held throughout so the light upper stage
                // does not tumble while it has no thrust / gimbal authority.
                // Hold target for the no-thrust window. Do NOT slew to the last
                // steering command: with no gimbal the weak RCS/wheels cannot hold
                // its pitch-up angle of attack, and in air aero drives the stage
                // further into the turn -- the "pitch-up, no control" failure. In
                // air hold prograde (angle of attack -> 0, aero trims it); in vacuum
                // hold the stage's current attitude (RCS only nulls the sep rates).
                // (prograde here is the non-rotating-frame velocity -- off from the
                // air-relative prograde by the body-rotation component, a small
                // direction error that still kills the dominant angle of attack.)
                const double hold_q_pa = vessel.flight(frame).dynamic_pressure();
                const Vec3 hold_eci = hold_q_pa > kStageAeroHoldQPa
                    ? normalized_or(velocity, last_steer_eci)
                    : normalized_or(eci_from_krpc(vessel.direction(frame)), last_steer_eci);
                const auto hold = krpc_from_eci(hold_eci);
                // RCS is already on for the whole flight; re-issuing the hold
                // target during the engine-off dwell keeps the autopilot locked
                // against the decoupler impulse while there is no gimbal authority.
                ap.set_target_direction(hold);
                std::cout << "MECO: throttle 0 before separation (RCS hold attitude)\n";
                ctrl.set_throttle(0.0f);
                // Re-issue the hold target in small steps (not one long sleep) so
                // the autopilot stays locked across the staging events.
                for (double held = 0.0; held < kStageSettleS; held += dt_s) {
                    ap.set_target_direction(hold);
                    std::this_thread::sleep_for(std::chrono::duration<double>(dt_s));
                }
                std::cout << "staging: jettison spent stage\n";
                ctrl.activate_next_stage();
                ap.set_target_direction(hold);
                if (ignites_engine) {
                    for (double held = 0.0; held < kStageSeparationCoastS; held += dt_s) {
                        ap.set_target_direction(hold);
                        std::this_thread::sleep_for(std::chrono::duration<double>(dt_s));
                    }
                    std::cout << "staging: ignite next stage\n";
                    ctrl.activate_next_stage();
                    // Hold the separation attitude and throttle up until the new
                    // stage's thrust is established, then let the phase guidance
                    // take over (it is safe to slew once the engine has authority).
                    const double t_ignite = vessel.met();
                    while (true) {
                        ap.set_target_direction(hold);
                        ctrl.set_throttle(1.0f);
                        const double avail = vessel.available_thrust();
                        const double ratio = avail > 1.0 ? vessel.thrust() / avail : 0.0;
                        if (ratio >= kStageThrustEstablishedFraction ||
                            vessel.met() - t_ignite > kStageThrustEstablishTimeoutS) {
                            std::cout << "next stage thrust established (ratio " << ratio
                                      << "); resuming guidance\n";
                            break;
                        }
                        std::this_thread::sleep_for(std::chrono::duration<double>(dt_s));
                    }
                    phase_start = vessel.met();  // re-anchor phase time past the hold
                }
            }
            actions_done[phase_index] = 1;
        }

        // Fairing jettison: once dynamic pressure has dropped back to ~0 after the
        // ascent (and the upper stage is active so the activation hits the fairing,
        // not the booster). Sheds the dead fairing mass so the upper stage / UPFG
        // insert the target orbit instead of carrying it through cutoff.
        if (!fairing_jettisoned) {  // skip the q RPC once the fairing is gone
            const double q_pa = vessel.flight(frame).dynamic_pressure();
            if (q_pa > kFairingArmQPa) {
                fairing_armed = true;
            }
            if (active_stage >= 1 && fairing_armed && q_pa < kFairingJettisonQPa) {
                std::cout << "dynamic pressure ~0 (" << q_pa
                          << " Pa); jettisoning fairing\n";
                ctrl.activate_next_stage();
                fairing_jettisoned = true;
            }
        }

        const Command cmd = compute_command(
            script, phase, phase_time, position, velocity, mass, active_stage);

        ap.set_target_direction(krpc_from_eci(cmd.thrust_unit_eci));
        ctrl.set_throttle(static_cast<float>(cmd.throttle));
        last_steer_eci = cmd.thrust_unit_eci;  // remembered for the next staging hold

        if (met >= next_log) {
            const double pe_km = vessel.orbit().periapsis_altitude() / 1000.0;
            const double ap_km = vessel.orbit().apoapsis_altitude() / 1000.0;
            std::cout << "t=" << met << " p" << phase_index << "(" << phase.name << ") alt="
                      << altitude / 1000.0 << "km v=" << speed << " m=" << mass
                      << " pe=" << pe_km << "km ap=" << ap_km << "km thr="
                      << cmd.throttle << (cmd.upfg ? " UPFG tgo=" : " poly");
            if (cmd.upfg) {
                std::cout << cmd.tgo_s;
            }
            std::cout << "\n";
            next_log = met + 2.0;
        }

        const double propellant = active_stage_propellant_kg(script, active_stage, mass);
        bool advance = phase_terminated(
            phase, phase_time, altitude, speed, mass, propellant, thrust_ratio);
        if (phase.is_upfg && cmd.upfg && cmd.upfg_ok && cmd.tgo_s <= kBurnoutMarginS) {
            advance = true;
        }
        // Burnout fallback: a non-time, non-ignition phase ends if the active
        // engine has quit (thrust collapsed) while we are still commanding it.
        // This stages the booster on flameout even if the live KSP mass does not
        // match the script's propellant figure exactly. Skips the pad-hold
        // ignition phase (thrust is legitimately ~0 during the dead-time).
        if (!advance && !phase.is_upfg && phase.termination_type != "time" &&
            phase.termination_type != "thrust_fraction" &&
            cmd.throttle > 0.1 && phase_time > 1.0 && thrust_ratio < 0.05) {
            std::cout << "active stage flameout detected (thrust collapsed)\n";
            advance = true;
        }
        // UPFG upper-stage thrust collapsed before reaching the target. Thrust
        // going to ~zero does NOT necessarily mean depletion -- it also happens
        // with propellant still aboard (ullage not settled after the coast, a
        // starved tank / missing crossfeed, or an engine that shut down). Require
        // the loss to persist (kUpfgThrustLossDwellS) so a brief dropout does not
        // kill the burn, then stop commanding a dead engine (which would otherwise
        // hang the guidance loop forever) and report the cause the propellant model
        // actually supports rather than always blaming depletion.
        if (phase.is_upfg && cmd.throttle > 0.1 && phase_time > 2.0 &&
            thrust_ratio < kThrustEstablishedFloor) {
            if (upfg_thrust_loss_since < 0.0) {
                upfg_thrust_loss_since = met;
            }
        } else {
            upfg_thrust_loss_since = -1.0;  // thrust present: reset the dwell
        }
        if (!advance && upfg_thrust_loss_since >= 0.0 &&
            met - upfg_thrust_loss_since > kUpfgThrustLossDwellS) {
            const double pe_km = vessel.orbit().periapsis_altitude() / 1000.0;
            const double ap_km = vessel.orbit().apoapsis_altitude() / 1000.0;
            const double full_prop = active_stage < script.stages.size()
                ? script.stages[active_stage].propellant_kg : 0.0;
            const bool fuel_remains = full_prop > 1.0 && propellant > 0.05 * full_prop;
            if (fuel_remains) {
                std::cout << "WARNING: upper-stage thrust collapsed with ~" << propellant
                          << " kg propellant still aboard before UPFG target (pe=" << pe_km
                          << "km ap=" << ap_km << "km) -- not depletion; check ullage"
                          << " settling, a starved tank / crossfeed, or an engine shutdown\n";
            } else {
                std::cout << "WARNING: upper-stage propellant depleted before UPFG target"
                          << " (pe=" << pe_km << "km ap=" << ap_km << "km) -- orbit is short\n";
            }
            advance = true;
        }
        if (advance) {
            std::cout << "phase " << phase_index << " complete at t=" << met << "\n";
            // Releasing a pad-hold ignition phase fires the launch clamp so the
            // vehicle lifts off only after thrust is established.
            const bool was_clamped = phase.hold_down_clamp_initial_active;
            const bool next_clamped = phase_index + 1 < script.phases.size() &&
                script.phases[phase_index + 1].hold_down_clamp_initial_active;
            ++phase_index;
            phase_start = met;
            if (was_clamped && !next_clamped) {
                std::cout << "thrust established; releasing launch clamp (staging)\n";
                ctrl.activate_next_stage();
            }
            continue;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(dt_s));
    }

    std::cout << "guidance complete; throttle 0, SAS on, autopilot released\n";
    ctrl.set_throttle(0.0f);
    // Engine shutdown at orbit insertion: hand attitude to SAS stability assist
    // so the coasting stage stays steady after the kRPC autopilot disengages.
    ap.disengage();
    ctrl.set_sas(true);
    return 0;
}

bool parse_options(int argc, char** argv, Options* options)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << "\n";
                return {};
            }
            return argv[++i];
        };
        if (arg == "--dry-run") {
            options->dry_run = true;
        } else if (arg == "--no-launch") {
            options->auto_launch = false;
        } else if (arg == "--address") {
            options->address = next("--address");
        } else if (arg == "--rpc-port") {
            options->rpc_port = static_cast<unsigned int>(std::stoul(next("--rpc-port")));
        } else if (arg == "--stream-port") {
            options->stream_port = static_cast<unsigned int>(std::stoul(next("--stream-port")));
        } else if (arg == "--dt") {
            options->dt_s = std::stod(next("--dt"));
        } else if (!arg.empty() && arg[0] == '-') {
            std::cerr << "unknown option: " << arg << "\n";
            return false;
        } else if (options->csv_path.empty()) {
            options->csv_path = arg;
        } else {
            std::cerr << "unexpected argument: " << arg << "\n";
            return false;
        }
    }
    return !options->csv_path.empty();
}

} // namespace

int run_player(int argc, char** argv)
{
    Options options;
    if (!parse_options(argc, argv, &options)) {
        print_usage();
        return 2;
    }

    const std::string text = read_file(options.csv_path);
    if (text.empty()) {
        std::cerr << "failed to read guidance script: " << options.csv_path << "\n";
        return 1;
    }

    GuidanceScript script;
    std::string error;
    if (!post2::player::parse_guidance_script(text, &script, &error)) {
        std::cerr << "parse error: " << error << "\n";
        return 1;
    }

    if (options.dry_run) {
        return run_dry(script, options.dt_s > 0.0 ? options.dt_s : 0.1);
    }
    try {
        return run_live(script, options);
    } catch (const std::exception& ex) {
        std::cerr << "live flight error: " << ex.what() << "\n";
        return 1;
    }
}

int main(int argc, char** argv)
{
    try {
        return exit_player(run_player(argc, argv));
    } catch (const std::exception& ex) {
        std::cerr << "fatal error: " << ex.what() << "\n";
        return exit_player(1);
    }
}
