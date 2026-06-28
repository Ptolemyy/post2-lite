// gridfin_test -- closed-loop aero-glide landing guidance (kRPC, C++).
//
// Architecture (two nested loops, see the headers for detail):
//   * OUTER (aero_glide_mpc.hpp): a sampling-based (CEM) nonlinear MPC over the
//     point-mass aero-glide dynamics, using the parent project's real CD/CL aero
//     table + a sampled density profile. It plans the angle-of-attack steering
//     that brings the UNPOWERED booster over the target lat/lon at the target
//     altitude with horizontal velocity -> 0 and nose -> vertical, and emits the
//     desired nose direction.
//   * INNER (attitude_pid.hpp): an own per-axis PID that points the nose at the
//     MPC's desired direction using only the four deployed grid fins
//     (Control.set_pitch/yaw/roll), damping roll and slewing gently to avoid the
//     pitch/yaw -> roll coupling this airframe has.
//
//   gridfin_test [options]
//     --address IP         kRPC server address           (default 127.0.0.1)
//     --rpc-port N         kRPC rpc port                 (default 30000)
//     --stream-port N      kRPC stream port              (default 30001)
//     --lat D --lon D      target latitude/longitude     (default LZ-1)
//     --alt M              target altitude (m, MSL)      (default 1000)
//     --dt S               inner control period          (default 0.05)
//     --mpc-period S       outer replan period           (default 1.0)
//     --aoa-max DEG        angle-of-attack cap           (default 10)
//     --mass KG            booster mass override         (default: read live)
//     --sref M2            aero reference area override  (default: from table)
//     --table FILE         aero table CSV (CD/CL vs M,a) (default cases/aero_table_stage1_only.csv)
//     --log FILE           CSV telemetry log             (default gridfin_log.csv)
//     --dry-run            offline sanity sim, no KSP/kRPC connection
//
// Keys (live):  1 = toggle grid fins   c = engage/disengage control
//               s = toggle SAS         q = quit

#include "aero_glide_mpc.hpp"
#include "fin_mpc.hpp"
#include "glide_math.hpp"
#include "post2/aero/aero_table.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#endif

#include <krpc.hpp>
#include <krpc/services/space_center.hpp>

namespace {

using gridfin::Vec3;
using gridfin::AeroGlideMpc;
using gridfin::MpcConfig;
using gridfin::MpcResult;
using gridfin::GlideEnv;
using gridfin::Atmosphere;
using gridfin::FinMpc;
using gridfin::FinMpcConfig;
using gridfin::FinCommand;
using gridfin::v_hat;
using gridfin::v_norm;
using gridfin::v_perp;
using gridfin::v_sub;
using gridfin::v_add;
using gridfin::v_dot;
using gridfin::v_scale;
using gridfin::clampd;
using gridfin::kDegToRad;
using gridfin::kRadToDeg;

struct Options {
    std::string address     = "127.0.0.1";
    unsigned    rpc_port    = 30000;
    unsigned    stream_port = 30001;
    double      lat         = 28.608367503204;
    double      lon         = -80.5997466838726;
    double      alt         = 1000.0;
    double      dt_s        = 0.05;
    double      mpc_period  = 1.0;
    double      aoa_max_deg = 10.0;
    double      max_defl_deg = 25.0;   // per-fin deployAngle command limit (deg)
    double      lift_sign   = -1.0;    // this booster's lateral force opposes the
                                       // nose tilt (confirmed live); +1 to override
    double      mass        = 0.0;     // 0 -> read live
    double      sref        = 0.0;     // 0 -> from table
    std::string table_path  = "aero_table_booster.csv";  // booster-only, fins deployed
    std::string log_path    = "gridfin_log.csv";
    bool        dry_run     = false;

    // ---- per-fin probe/test mode ----
    bool        probe       = false;   // --probe-fins: dump fin modules/fields
    std::string probe_field;           // --field NAME: localized field name to drive
    std::string probe_field_id;        // --field-id ID: internal (ASCII) field id to drive
    int         probe_fin   = 0;       // --fin N: which fin to drive (-1 = all)
    double      probe_value = 10.0;    // --value V: amplitude to set/sweep
    bool        probe_sweep = false;   // --sweep: oscillate the field +V/-V/0
    bool        probe_isolate = false; // --isolate: deploy + ignore pitch/yaw/roll
                                       //   on the driven fin so only deployAngle moves it
    bool        probe_measure = false; // --measure-speed: step deployAngle and poll
                                       //   readback + body rate at ~50 Hz (slew rate)
};

Options parse_args(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto nextd = [&](double& d) { if (i + 1 < argc) d = std::stod(argv[++i]); };
        auto nextu = [&](unsigned& u) { if (i + 1 < argc) u = static_cast<unsigned>(std::stoul(argv[++i])); };
        if      (a == "--address" && i + 1 < argc) o.address = argv[++i];
        else if (a == "--rpc-port")    nextu(o.rpc_port);
        else if (a == "--stream-port") nextu(o.stream_port);
        else if (a == "--lat")  nextd(o.lat);
        else if (a == "--lon")  nextd(o.lon);
        else if (a == "--alt")  nextd(o.alt);
        else if (a == "--dt")   nextd(o.dt_s);
        else if (a == "--mpc-period") nextd(o.mpc_period);
        else if (a == "--aoa-max")    nextd(o.aoa_max_deg);
        else if (a == "--max-defl")   nextd(o.max_defl_deg);
        else if (a == "--lift-sign")  nextd(o.lift_sign);
        else if (a == "--mass") nextd(o.mass);
        else if (a == "--sref") nextd(o.sref);
        else if (a == "--table" && i + 1 < argc) o.table_path = argv[++i];
        else if (a == "--log"   && i + 1 < argc) o.log_path = argv[++i];
        else if (a == "--dry-run") o.dry_run = true;
        else if (a == "--probe-fins") o.probe = true;
        else if (a == "--field" && i + 1 < argc) o.probe_field = argv[++i];
        else if (a == "--field-id" && i + 1 < argc) o.probe_field_id = argv[++i];
        else if (a == "--fin")   { if (i + 1 < argc) o.probe_fin = std::stoi(argv[++i]); }
        else if (a == "--value") nextd(o.probe_value);
        else if (a == "--sweep") o.probe_sweep = true;
        else if (a == "--isolate") o.probe_isolate = true;
        else if (a == "--measure-speed") o.probe_measure = true;
        else if (a == "--help" || a == "-h") std::cout <<
            "gridfin_test: aero-glide MPC + per-fin deployAngle MPC landing guidance (kRPC).\n"
            "  --address --rpc-port(30000) --stream-port(30001) --lat --lon --alt(1000)\n"
            "  --dt --mpc-period --aoa-max --max-defl(25) --mass --sref --table --log --dry-run\n"
            "  --probe-fins [--field-id deployAngle --fin N --value V --isolate --sweep|--measure-speed]\n";
    }
    return o;
}

int poll_key() {
#ifdef _WIN32
    if (_kbhit()) return _getch();
#endif
    return -1;
}

Vec3 to_vec(const std::tuple<double, double, double>& t) {
    return {std::get<0>(t), std::get<1>(t), std::get<2>(t)};
}
std::tuple<double, double, double> to_tup(const Vec3& v) {
    return std::make_tuple(v[0], v[1], v[2]);
}

// Proportional lift-steering fallback (the kOS predictor-corrector law): point
// the nose retrograde, tilted toward the horizontal direction that corrects the
// miss by an angle of attack proportional to (miss - velocity-lead), capped at
// the AoA limit. Used when the MPC plan is untrustworthy (did not reach the
// target altitude within its horizon). All vectors body-fixed.
Vec3 proportional_aim(const Vec3& pos, const Vec3& vel, const Vec3& target,
                      double max_aoa, double lift_sign) {
    const Vec3 up   = v_hat(pos);
    const Vec3 vhat = v_hat(vel);
    const Vec3 h_err = v_perp(v_sub(target, pos), up);   // horizontal miss toward target
    const Vec3 h_vel = v_perp(vel, up);                  // horizontal velocity
    const Vec3 steer = v_sub(h_err, v_scale(h_vel, 4.0));// PD: miss minus velocity lead
    const double mag = v_norm(steer);
    Vec3 nose = v_scale(vhat, -1.0);                     // retrograde by default
    if (mag > 1e-3) {
        const double aoa = std::min(5.0e-4 * mag, max_aoa);
        // Tilt toward the miss-correcting direction; if the real lift opposes the
        // nose tilt (lift_sign<0) tilt the OTHER way so the force points to target.
        Vec3 lift_dir = v_scale(v_hat(v_perp(steer, vhat)), lift_sign);
        if (v_norm(lift_dir) > 1e-6)
            nose = v_add(v_scale(vhat, -std::cos(aoa)), v_scale(lift_dir, std::sin(aoa)));
    }
    return v_hat(nose);
}

// Directory containing this executable (so the default table resolves no matter
// what the working directory is when launched).
std::string exe_dir() {
#ifdef _WIN32
    char buf[MAX_PATH] = {0};
    DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    std::string p(buf, n);
    const auto slash = p.find_last_of("\\/");
    return slash == std::string::npos ? std::string() : p.substr(0, slash);
#else
    return std::string();
#endif
}

// Load the aero table, or synthesise a crude analytic one (for --dry-run when
// the CSV is absent): CD ~ 0.8 + transonic bump, CL ~ 2.0*sin(a)*cos(a) per rad.
// The path is tried as given, then relative to the executable's own directory.
post2::aero::AeroTable load_or_synth_table(const std::string& path, bool allow_synth) {
    post2::aero::AeroTable t;
    std::string err;
    if (post2::aero::read_aero_table_csv(path, &t, &err) && !t.empty()) return t;
    const std::string dir = exe_dir();
    if (!dir.empty()) {
        const std::string alt = dir + "/" + path;
        std::string err2;
        if (post2::aero::read_aero_table_csv(alt, &t, &err2) && !t.empty()) {
            std::cout << "aero table: " << alt << "\n";
            return t;
        }
    }
    if (!allow_synth) {
        std::cout << "warning: could not read aero table '" << path << "': " << err << "\n";
        return t;
    }
    std::cout << "note: aero table '" << path << "' unavailable (" << err
              << ") -> using synthetic table\n";
    t.reference_area_m2 = 10.52;
    for (double m = 0.0; m <= 6.0; m += 0.5) t.mach.push_back(m);
    for (double a = 0.0; a <= 20.0; a += 2.0) t.alpha_deg.push_back(a);
    for (double m : t.mach) {
        const double wave = 0.5 * std::exp(-((m - 1.1) * (m - 1.1)) / 0.2);  // transonic bump
        for (double a : t.alpha_deg) {
            const double ar = a * kDegToRad;
            t.cd.push_back(0.8 + wave + 0.9 * std::sin(ar) * std::sin(ar));
            t.cl.push_back(2.2 * std::sin(ar) * std::cos(ar));
        }
    }
    return t;
}

// ===========================================================================
//                          offline sanity simulation
// ===========================================================================
int run_dry(const Options& opt) {
    std::cout << "=== gridfin_test --dry-run (offline sanity sim) ===\n\n";

    // ---- (0) inner per-fin MPC: roll suppression under an INVERTED roll sign
    //          and pitch/yaw -> roll coupling (no RCS, fins only) -------------
    {
        FinMpcConfig fc;
        FinMpc fin(fc);
        fin.set_layout(4, 0.0);
        // True plant: omega_dot[a] = b_true[a]*sign_true[a]*u[a] (+ roll leak).
        // Mixed signs (roll inverted, pitch/yaw normal) -> verifies the dither
        // identifies each axis sign INDEPENDENTLY (no global-sign assumption).
        const Vec3 b_true    = {1.8, 1.7, 6.0};   // roll very responsive (tiny MOI)
        const Vec3 sign_true = {1.0, 1.0, -1.0};  // ROLL WIRED BACKWARDS only
        const double Kleak   = 0.8;               // yaw command -> roll accel leak
        Vec3 omega = {0.0, 0.0, 0.0};
        const double dt = fc.dt;
        const Vec3 r_cmd = {0.10, 0.12, 0.0};     // steering demand drives yaw->roll
        double q = 0.0, roll_peak = 0.0;
        std::cout << "(0) inner per-fin MPC: inverted roll sign + yaw->roll leak (Kleak="
                  << Kleak << ")\n";
        for (int step = 0; step < 600; ++step) {
            q = std::min(40000.0, q + 200.0);
            fin.set_target_rate(r_cmd);
            FinCommand c = fin.update(omega, dt, q);
            const Vec3& u = c.axis;
            omega[0] += dt * (b_true[0] * sign_true[0] * u[0]);
            omega[1] += dt * (b_true[1] * sign_true[1] * u[1]);
            omega[2] += dt * (b_true[2] * sign_true[2] * u[2] + Kleak * b_true[1] * u[1]);
            if (step > 100) roll_peak = std::max(roll_peak, std::abs(omega[2]));
            if (step % 150 == 0 || step == 599) {
                const char* mn = fin.mode() == gridfin::FinMode::WaitQ ? "WaitQ" :
                                 fin.mode() == gridfin::FinMode::Dither ? "Dither" : "Active";
                std::printf("   t=%5.1f w=[% .3f % .3f % .3f] u_roll=% .2f g_roll=% .2f be_roll=% .2f mode=%-6s\n",
                            step * dt, omega[0], omega[1], omega[2], c.axis[2],
                            c.dbg_g[2], c.dbg_be[2], mn);
            }
        }
        const double perr = std::sqrt((omega[0]-r_cmd[0])*(omega[0]-r_cmd[0]) +
                                      (omega[1]-r_cmd[1])*(omega[1]-r_cmd[1]));
        std::printf("   -> signs id=[%+.0f %+.0f %+.0f] (true [%+.0f %+.0f %+.0f]); "
                    "peak|roll|=%.2f rad/s %s; pitch/yaw rate err=%.3f %s\n\n",
                    fin.roll_sign()[0], fin.roll_sign()[1], fin.roll_sign()[2],
                    sign_true[0], sign_true[1], sign_true[2], roll_peak,
                    roll_peak < 1.0 ? "[bounded OK]" : "[DIVERGING]",
                    perr, perr < 0.06 ? "[OK]" : "[high]");
    }

    post2::aero::AeroTable table = load_or_synth_table(opt.table_path, /*allow_synth=*/true);

    GlideEnv env;                       // Kerbin-like defaults (mu, r_body)
    env.table  = &table;
    env.mass   = opt.mass > 0 ? opt.mass : 25000.0;
    env.s_ref  = opt.sref > 0 ? opt.sref : (table.reference_area_m2 > 0 ? table.reference_area_m2 : 10.52);
    env.max_aoa= opt.aoa_max_deg * kDegToRad;
    env.lift_sign = opt.lift_sign;
    Atmosphere atmo;                    // exponential fallback (Earth-ish)
    atmo.rho0 = 1.225; atmo.scale_h = 7200.0; atmo.a_const = 300.0;
    env.atmo = &atmo;

    MpcConfig mc;
    AeroGlideMpc mpc(mc);

    // Booster starts high, descending, with a lateral offset from the target.
    // Radial = +x. Target is the +x point at the target altitude.
    const double R = env.r_body;
    const double target_alt = opt.alt;
    Vec3 target = {R + target_alt, 0.0, 0.0};
    Vec3 pos    = {R + 18000.0, 0.0, 1500.0};   // 18 km up, 1.5 km lateral (+z)
    Vec3 vel    = {-220.0, 0.0, 10.0};          // descending, slight lateral drift

    // First-order attitude follower: the real nose lags the commanded nose with
    // time constant tau (stands in for the inner PID + airframe).
    Vec3 nose = v_hat(v_scale(vel, -1.0));
    const double tau = 0.7, dt = opt.dt_s;
    double t = 0.0, next_plan = 0.0;
    MpcResult res;

    const Vec3 up0 = v_hat(pos);
    const double miss0 = v_norm(v_perp(v_sub(pos, target), up0));
    std::printf("start  alt=%6.0f m  horiz_miss=%6.0f m  horiz_speed=%5.1f m/s\n",
                v_norm(pos) - R, miss0, v_norm(v_perp(vel, up0)));

    for (int step = 0; step < 60000; ++step) {
        const double r = v_norm(pos);
        const double h = r - R;
        if (h <= target_alt) break;
        if (t >= 600.0) break;

        if (t >= next_plan) {
            next_plan = t + opt.mpc_period;
            res = mpc.plan(pos, vel, nose, Vec3{0, 0, 0}, target, env, target_alt);
        }
        // Inner loop stand-in: relax the actual nose toward the commanded aim.
        const Vec3 aim = res.aim_hat;
        for (int i = 0; i < 3; ++i) nose[i] += (dt / tau) * (aim[i] - nose[i]);
        nose = v_hat(nose);

        // Point-mass dynamics with the REALISED nose (so attitude lag matters).
        const double speed = v_norm(vel);
        const Vec3 vhat = speed > 1e-6 ? v_scale(vel, 1.0 / speed) : v_hat(v_scale(pos, -1.0));
        const double cos_a = clampd(-gridfin::v_dot(nose, vhat), -1.0, 1.0);
        const double alpha = std::acos(cos_a);                 // realised AoA
        const Vec3 lift_dir = v_hat(v_perp(nose, vhat));
        const double rho = atmo.density(h), q = 0.5 * rho * speed * speed;
        const double mach = speed / atmo.sound_speed(h);
        double cd = 0, cl = 0; table.lookup(mach, alpha * kRadToDeg, &cd, &cl);
        const Vec3 up = v_scale(pos, 1.0 / r);
        Vec3 acc = v_scale(up, -env.mu / (r * r));
        for (int i = 0; i < 3; ++i)
            acc[i] += (q * env.s_ref / env.mass) * (cl * env.lift_sign * lift_dir[i] - cd * vhat[i]);
        for (int i = 0; i < 3; ++i) { vel[i] += acc[i] * dt; pos[i] += vel[i] * dt; }
        t += dt;

        if (step % 400 == 0) {
            const double miss = v_norm(v_perp(v_sub(pos, target), up));
            std::printf("  t=%5.1f alt=%6.0f miss=%6.0f vh=%5.1f aoa=%4.1fdeg "
                        "pred(miss=%5.0f vh=%4.1f reach=%d)\n",
                        t, h, miss, v_norm(v_perp(vel, up)), alpha * kRadToDeg,
                        res.pred_pos_err_m, res.pred_vh_mps, res.reached ? 1 : 0);
        }
    }

    const double r = v_norm(pos), h = r - R;
    const Vec3 up = v_scale(pos, 1.0 / r);
    const double miss = v_norm(v_perp(v_sub(pos, target), up));
    const double vh = v_norm(v_perp(vel, up));
    std::printf("\nend    alt=%6.0f m  horiz_miss=%6.0f m  horiz_speed=%5.1f m/s  (t=%.0fs)\n",
                h, miss, vh, t);
    std::printf("  -> horiz miss %s (%.0f -> %.0f m);  horiz speed %s (%.1f m/s)\n",
                miss < miss0 ? "reduced [OK]" : "NOT reduced", miss0, miss,
                vh < 25.0 ? "low [OK]" : "high", vh);
    std::cout << "\n=== dry-run complete ===\n";
    return 0;
}

// ===========================================================================
//             per-fin probe: discover & test independent fin control
// ===========================================================================
// kRPC's ControlSurface exposes no deflection setter, but each grid fin's
// underlying PartModule may expose a settable deflection-angle field (the PAW
// row you edit after removing the fin from symmetry). This mode dumps every grid
// fin's modules/fields so we can find that field's exact name, and -- given
// --field NAME -- drives ONE fin's field to prove independent actuation works.
int run_probe(const Options& opt) {
    using SC = krpc::services::SpaceCenter;
    std::cout << "connecting to kRPC at " << opt.address << ":" << opt.rpc_port << " ...\n";
    krpc::Client conn = krpc::connect("gridfin_probe", opt.address, opt.rpc_port, opt.stream_port);
    SC sc(&conn);
    SC::Vessel vessel = sc.active_vessel();
    SC::Control ctrl = vessel.control();

    // Collect the grid-fin parts (via the control surfaces, which ARE the fins).
    std::vector<SC::Part> fin_parts;
    try {
        for (auto& cs : vessel.parts().control_surfaces()) {
            SC::Part p = cs.part();
            std::string nm = p.title();
            std::transform(nm.begin(), nm.end(), nm.begin(), ::tolower);
            if (nm.find("grid") != std::string::npos || nm.find("fin") != std::string::npos)
                fin_parts.push_back(p);
        }
    } catch (const std::exception& e) { std::cout << "enumerate error: " << e.what() << "\n"; }
    std::cout << "found " << fin_parts.size() << " grid-fin part(s)\n\n";

    // ---- dump each fin's modules, fields, events, actions ----
    for (size_t i = 0; i < fin_parts.size(); ++i) {
        std::cout << "==== FIN " << i << ": " << fin_parts[i].title() << " ====\n";
        try {
            for (auto& m : fin_parts[i].modules()) {
                std::cout << "  module: " << m.name() << "\n";
                // Print by internal ID (ASCII, language-independent -> use with
                // --field-id). The localized-name map is sorted differently, so we
                // do NOT try to pair them (that mismatch produced wrong labels).
                std::map<std::string, std::string> byid;
                try { byid = m.fields_by_id(); } catch (...) {}
                for (auto& kv : byid)
                    std::cout << "      field id[" << kv.first << "] = " << kv.second << "\n";
                for (auto& ev : m.events())   std::cout << "      event: " << ev << "\n";
                for (auto& ac : m.actions())  std::cout << "      action: " << ac << "\n";
            }
        } catch (const std::exception& e) { std::cout << "  module dump error: " << e.what() << "\n"; }
        std::cout << "\n";
    }

    const bool by_id = !opt.probe_field_id.empty();
    if (opt.probe_field.empty() && !by_id) {
        std::cout << "No --field/--field-id given: dump only. Re-run with e.g.\n"
                     "  --probe-fins --field-id deployAngle --fin 0 --value 15 --sweep\n"
                     "to drive ONE fin's field and confirm it moves independently in-game.\n"
                     "(Prefer --field-id with the ASCII id[...] above -- language-independent.)\n";
        return 0;
    }

    // ---- drive the chosen field on the selected fin(s) ----
    auto set_on = [&](int idx, float v) {
        if (idx < 0 || idx >= (int)fin_parts.size()) return;
        try {
            for (auto& m : fin_parts[idx].modules()) {
                if (by_id ? m.has_field_with_id(opt.probe_field_id)
                          : m.has_field(opt.probe_field)) {
                    if (by_id) m.set_field_float_by_id(opt.probe_field_id, v);
                    else       m.set_field_float(opt.probe_field, v);
                    const std::string rb = by_id ? m.get_field_by_id(opt.probe_field_id)
                                                 : m.get_field(opt.probe_field);
                    std::cout << "  fin " << idx << " [" << m.name() << "] <- " << v
                              << "  (readback " << rb << ")\n";
                    return;
                }
            }
            std::cout << "  fin " << idx << ": no module has that field\n";
        } catch (const std::exception& e) { std::cout << "  set error: " << e.what() << "\n"; }
    };

    try { ctrl.set_brakes(true); } catch (...) {}   // deploy so movement is visible
    const int target = opt.probe_fin;               // -1 = all fins

    // Make deployAngle actually move the fin: turn its "deploy" on; with
    // --isolate also disable the fin's pitch/yaw/roll control response so ONLY
    // deployAngle drives it (clean independent-actuation test).
    auto prep = [&](int idx) {
        if (idx < 0 || idx >= (int)fin_parts.size()) return;
        try {
            for (auto& m : fin_parts[idx].modules()) {
                if (!m.has_field_with_id("deployAngle")) continue;
                m.set_field_bool_by_id("deploy", true);
                if (opt.probe_isolate) {
                    m.set_field_bool_by_id("ignorePitch", true);
                    m.set_field_bool_by_id("ignoreYaw",   true);
                    m.set_field_bool_by_id("ignoreRoll",  true);
                }
                break;
            }
        } catch (const std::exception& e) { std::cout << "  prep error: " << e.what() << "\n"; }
    };
    if (target < 0) for (size_t i = 0; i < fin_parts.size(); ++i) prep((int)i);
    else prep(target);

    // ---- measure the actuator slew rate of one fin ----
    if (opt.probe_measure) {
        const int idx = target < 0 ? 0 : target;
        SC::CelestialBody body = vessel.orbit().body();
        SC::ReferenceFrame bcf = body.reference_frame();
        bool found = false;
        SC::Module mod = [&]{ for (auto& m : fin_parts[idx].modules())
            if (m.has_field_with_id("deployAngle")) { found = true; return m; }
            return fin_parts[idx].modules().front(); }();
        if (!found) { std::cout << "fin " << idx << " has no deployAngle field\n"; return 0; }

        const float V = static_cast<float>(opt.probe_value);
        try { mod.set_field_float_by_id("deployAngle", 0.0f); } catch (...) {}
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        auto norm = [](const std::tuple<double,double,double>& t){
            return std::sqrt(std::get<0>(t)*std::get<0>(t)+std::get<1>(t)*std::get<1>(t)+std::get<2>(t)*std::get<2>(t)); };

        std::cout << "step deployAngle 0 -> " << V << " deg on fin " << idx
                  << "; polling readback + |body rate| for 2 s ...\n";
        const auto t0 = std::chrono::steady_clock::now();
        try { mod.set_field_float_by_id("deployAngle", V); } catch (...) {}
        double prev_rb = 0.0, t_reach = -1.0;
        while (true) {
            const double now = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (now > 2.0) break;
            double rb = 0.0, w = 0.0;
            try { rb = std::stod(mod.get_field_by_id("deployAngle")); } catch (...) {}
            try { w = norm(vessel.angular_velocity(bcf)); } catch (...) {}
            if (t_reach < 0 && std::abs(rb - V) < 0.05 * std::abs(V) + 0.2) t_reach = now;
            std::printf("  t=%.3f  readback=%6.2f  |w|=%.4f rad/s\n", now, rb, w);
            prev_rb = rb;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        try { mod.set_field_float_by_id("deployAngle", 0.0f); } catch (...) {}
        if (t_reach >= 0)
            std::printf("-> readback reached commanded angle in ~%.0f ms"
                        " (%.0f deg/s if it ramped; instant => field is the SETPOINT,\n"
                        "   physical slew shows up in |w| rise instead)\n",
                        t_reach * 1000.0, t_reach > 1e-3 ? std::abs(V) / t_reach : 0.0);
        (void)prev_rb;
        return 0;
    }
    auto drive = [&](float v) {
        if (target < 0) { for (size_t i = 0; i < fin_parts.size(); ++i) set_on((int)i, v); }
        else set_on(target, v);
    };

    const float V = static_cast<float>(opt.probe_value);
    std::cout << "driving field '" << opt.probe_field << "' on fin "
              << (target < 0 ? -1 : target) << (opt.probe_sweep ? " (sweep)\n" : "\n");
    if (!opt.probe_sweep) {
        drive(V);
        std::cout << "held for 5 s -- watch the fin in-game ...\n";
        std::this_thread::sleep_for(std::chrono::seconds(5));
        drive(0.0f);
    } else {
        for (int c = 0; c < 3; ++c) {
            drive(+V);  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            drive(-V);  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            drive(0.0f);std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
    std::cout << "probe done.\n";
    return 0;
}

// ===========================================================================
//                               live control
// ===========================================================================
int run_live(const Options& opt) {
    using SC = krpc::services::SpaceCenter;
    std::cout << "connecting to kRPC at " << opt.address << ":" << opt.rpc_port
              << " (stream " << opt.stream_port << ") ...\n";
    krpc::Client conn = krpc::connect("gridfin_test", opt.address, opt.rpc_port, opt.stream_port);
    SC sc(&conn);
    SC::Vessel vessel = sc.active_vessel();
    SC::CelestialBody body = vessel.orbit().body();
    SC::ReferenceFrame bcf = body.reference_frame();    // rotates with the body
    SC::ReferenceFrame vrf = vessel.reference_frame();  // body-fixed vessel frame
    SC::Control ctrl = vessel.control();

    std::cout << "controlling '" << vessel.name() << "' around '" << body.name() << "'\n";
    std::cout << "target: lat=" << opt.lat << " lon=" << opt.lon
              << " alt=" << opt.alt << " m\n";

    // ---- aero table + environment -----------------------------------------
    post2::aero::AeroTable table = load_or_synth_table(opt.table_path, /*allow_synth=*/false);
    if (table.empty()) { std::cerr << "fatal: no usable aero table\n"; return 1; }

    GlideEnv env;
    env.mu     = body.gravitational_parameter();
    env.r_body = body.equatorial_radius();
    env.mass   = opt.mass > 0 ? opt.mass : static_cast<double>(vessel.mass());
    env.s_ref  = opt.sref > 0 ? opt.sref : (table.reference_area_m2 > 0 ? table.reference_area_m2 : 10.52);
    env.max_aoa= opt.aoa_max_deg * kDegToRad;
    env.lift_sign = opt.lift_sign;
    env.table  = &table;

    Atmosphere atmo;
    env.atmo = &atmo;
    // Sample a density column over the target so the MPC forward-sim sees the
    // real (orders-of-magnitude) density variation along the descent.
    try {
        const double top = std::max(2000.0, body.atmosphere_depth());
        for (double h = 0.0; h <= top + 1.0; h += top / 30.0) {
            const auto p = body.position_at_altitude(opt.lat, opt.lon, h, bcf);
            const double rho = body.atmospheric_density_at_position(p, bcf);
            atmo.h.push_back(h);
            atmo.rho.push_back(rho);
        }
        std::cout << "sampled density profile: " << atmo.h.size() << " levels, 0.."
                  << top << " m\n";
    } catch (const std::exception& e) {
        std::cout << "warning: density sampling failed (" << e.what()
                  << "); using exponential fallback\n";
        atmo.h.clear(); atmo.rho.clear();
    }
    std::cout << "env: mu=" << env.mu << " R=" << env.r_body << " mass=" << env.mass
              << " S_ref=" << env.s_ref << " aoa_max=" << opt.aoa_max_deg << "deg\n";

    // ---- grid fins: collect each fin's deployAngle module + azimuth --------
    // Each fin is actuated DIRECTLY via its ModuleControlSurface "deployAngle"
    // field (the per-fin angle, in degrees). We also read each fin's azimuth
    // around the roll axis (from its position in the vessel frame) so the inner
    // MPC's mixing matrix matches the real geometry.
    std::vector<SC::ControlSurface> grid_fins;
    std::vector<SC::Module> fin_mod;      // the module exposing deployAngle, per fin
    std::vector<double>     fin_phi;      // fin azimuth around the roll axis (rad)
    try {
        for (auto& cs : vessel.parts().control_surfaces()) {
            SC::Part part = cs.part();
            std::string nm = part.title();
            std::transform(nm.begin(), nm.end(), nm.begin(), ::tolower);
            if (nm.find("grid") == std::string::npos && nm.find("fin") == std::string::npos) continue;
            // deployAngle module
            bool has = false; SC::Module dm = cs.part().modules().front();
            for (auto& m : part.modules())
                if (m.has_field_with_id("deployAngle")) { dm = m; has = true; break; }
            if (!has) continue;
            // azimuth: position in vessel frame, projected on the transverse
            // plane (vessel +y = roll/nose axis; x,z are transverse).
            const auto p = part.position(vrf);
            const double phi = std::atan2(std::get<2>(p), std::get<0>(p));
            grid_fins.push_back(cs);
            fin_mod.push_back(dm);
            fin_phi.push_back(phi);
        }
    } catch (const std::exception& e) { std::cout << "fin enumerate error: " << e.what() << "\n"; }
    const int nfins = static_cast<int>(grid_fins.size());
    std::cout << "found " << nfins << " grid fin(s) with deployAngle; azimuths(deg):";
    for (double p : fin_phi) std::cout << ' ' << static_cast<int>(p * kRadToDeg);
    std::cout << "\n";

    bool fins_deployed = false;
    auto set_fins = [&](bool deploy) {
        // BRAKES is remapped to deploy/stow the fins; also extend each surface.
        try { ctrl.set_brakes(deploy); } catch (...) {}
        for (auto& f : grid_fins) { try { f.set_deployed(deploy); } catch (...) {} }
        fins_deployed = deploy;
        std::cout << (deploy ? "[1] grid fins DEPLOYED (brakes)\n" : "[1] grid fins STOWED (brakes)\n");
    };

    // Take over / hand back the fins. When controlling we set each fin to manual
    // deploy and DISABLE its pitch/yaw/roll control response, so the fin obeys
    // ONLY our deployAngle (KSP's 3-axis mixer / SAS no longer touch it). On
    // release we re-enable the normal control response.
    auto take_over_fins = [&](bool take) {
        for (auto& m : fin_mod) {
            try {
                m.set_field_bool_by_id("deploy", true);
                m.set_field_bool_by_id("ignorePitch", take);
                m.set_field_bool_by_id("ignoreYaw",   take);
                m.set_field_bool_by_id("ignoreRoll",  take);
                if (!take) m.set_field_float_by_id("deployAngle", 0.0f);
            } catch (...) {}
        }
    };

    // ---- controllers -------------------------------------------------------
    // OUTER: aero-glide MPC -> desired nose direction.
    // INNER: per-fin control-allocation MPC (fin_mpc). Pure grid fins, NO RCS:
    // it shares one per-fin deflection budget across pitch/yaw/roll, weights roll
    // far above the others (suppress roll), and we write the solved per-fin angle
    // straight to each fin's deployAngle. A desired-rate cascade points the nose.
    MpcConfig mc;
    AeroGlideMpc mpc(mc);
    FinMpcConfig fc;
    fc.dt = opt.dt_s;
    FinMpc fin(fc);
    if (nfins > 0) fin.set_layout_azimuths(fin_phi);
    else           fin.set_layout(4, 0.0);
    const float max_defl_deg = static_cast<float>(opt.max_defl_deg);

    // Attitude -> rate gains for the pointing cascade. Gentle on purpose: a large
    // pitch/yaw rate is exactly what pumps roll on this low-roll-MOI airframe.
    const double kp_att   = 1.2;     // desired rate per rad of pointing error
    const double max_rate = 0.20;    // rad/s steering-rate clamp

    std::ofstream log(opt.log_path);
    log << "t,ctrl,mode,alt_m,horiz_miss_m,horiz_speed_mps,q_pa,mach,aoa_cmd_deg,"
           "wp,wy,wr,cmd_pitch,cmd_yaw,cmd_roll,b_pitch,b_yaw,b_roll,sgn_roll,"
           "pred_miss_m,pred_vh_mps,reached,fins\n";
    log.flush();

    std::cout << "keys: 1=toggle fins (brakes)  c=control on/off  q=quit\n";
    bool control_on = false, arrived = false, converged_hold = false;
    try { ctrl.set_sas(true); } catch (...) {}

    const double dt = opt.dt_s;
    const auto t0 = std::chrono::steady_clock::now();
    double prev_t = 0.0, next_plan = 0.0, next_print = 0.0;
    MpcResult res;
    Vec3 aim_bcf = {0.0, 0.0, 0.0};
    bool running = true;

    // Engage/disengage the inner-loop fin controller (no RCS, no autopilot).
    // Engaging takes over the fins (manual deploy + ignore the 3-axis mixer);
    // disengaging hands them back to SAS.
    auto engage_control = [&](bool on) {
        control_on = on;
        fin.reset();
        take_over_fins(on);
        try { ctrl.set_sas(!on); } catch (...) {}   // SAS holds only when disengaged
    };

    std::cout << "entering control loop\n";
    while (running) {
      try {
        const double now = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const double real_dt = std::max(1e-3, now - prev_t);
        prev_t = now;

        int key = poll_key();
        if      (key == '1') set_fins(!fins_deployed);
        else if (key == 'c') engage_control(!control_on);
        else if (key == 'q' || key == 3) { running = false; break; }

        if (fins_deployed && !control_on) { engage_control(true);
            std::cout << "control engaged (auto, fins deployed)\n"; }

        // ---- telemetry ----
        Vec3 pos, vel, omega_w, nose_w, target;
        double q = 0.0, alt = 0.0, mach = 0.0, a_snd = 300.0;
        try {
            pos     = to_vec(vessel.position(bcf));
            vel     = to_vec(vessel.velocity(bcf));
            // Angular velocity read in the body-fixed frame (NOT the vessel frame,
            // which co-rotates and would read ~0), then rotated into vessel axes.
            omega_w = to_vec(vessel.angular_velocity(bcf));
            nose_w  = to_vec(vessel.direction(bcf));   // actual nose dir (6-DOF MPC)
            target  = to_vec(body.position_at_altitude(opt.lat, opt.lon, opt.alt, bcf));
            SC::Flight fl = vessel.flight(bcf);
            q     = fl.dynamic_pressure();
            alt   = fl.mean_altitude();
            mach  = fl.mach();
            a_snd = fl.speed_of_sound();
        } catch (const std::exception& e) {
            std::cout << "telemetry error: " << e.what() << " (vessel changed?)\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        if (a_snd > 1.0) atmo.a_const = a_snd;     // keep Mach model current

        // Body rate in [pitch,yaw,roll]: rotate omega into vessel axes (x=pitch,
        // y=roll(nose), z=yaw) -- transform_direction is a pure rotation so the
        // real spin magnitude is preserved.
        Vec3 omega_v = omega_w;
        try { omega_v = to_vec(sc.transform_direction(to_tup(omega_w), bcf, vrf)); } catch (...) {}
        const Vec3 omega = {omega_v[0], omega_v[2], omega_v[1]};

        const Vec3 up = v_hat(pos);
        const double horiz_miss  = v_norm(v_perp(v_sub(pos, target), up));
        const double horiz_speed = v_norm(v_perp(vel, up));

        // ---- outer MPC (replan at mpc_period) ----
        if (now >= next_plan) {
            next_plan = now + opt.mpc_period;
            // Refresh the 6-DOF attitude-dynamics authority from LIVE inertia +
            // available torque (anchored at the current q; the rollout scales it
            // ~ q/q_ref over the horizon). available_torque already includes the
            // q-dependent grid-fin aero authority.
            try {
                auto moi = vessel.moment_of_inertia();       // (pitch, roll, yaw)
                auto at  = vessel.available_torque();         // ((p,r,y)+, (p,r,y)-)
                const double Ip = std::max(1.0, std::get<0>(moi));
                const double Tp = std::abs(std::get<0>(std::get<0>(at)));
                if (Tp > 1.0 && q > 200.0) {
                    env.accel_max = clampd(Tp / Ip, 0.05, 5.0);
                    env.q_ref     = q;
                }
            } catch (...) {}
            res = mpc.plan(pos, vel, nose_w, omega_w, target, env, opt.alt);
            aim_bcf = res.aim_hat;
        }

        // TERMINAL HOLD: a booster cannot hover, so once it descends to the
        // target altitude the arrival is concluded -- hold the nose vertical.
        if (!arrived && alt <= opt.alt) {
            arrived = true;
            std::printf(">> ARRIVED at %.0f m: horiz miss=%.0f m, horiz speed=%.1f m/s"
                        " -- holding vertical\n", opt.alt, horiz_miss, horiz_speed);
        }
        // CONVERGED -> LOCK VERTICAL (borrowed from the kOS guidance): once the
        // booster is essentially over the target AND no longer moving sideways,
        // stop steering and hold vertical so we do not keep disturbing the
        // attitude chasing a sub-metre miss. Gate on BOTH position and horizontal
        // SPEED -- otherwise, when over the target but still moving fast laterally
        // (high-horizontal-velocity entry), locking vertical would stop nulling
        // the velocity and the booster drifts back out. Hysteresis to re-steer.
        if (!converged_hold && horiz_miss < 60.0 && horiz_speed < 5.0) converged_hold = true;
        else if (converged_hold && (horiz_miss > 200.0 || horiz_speed > 12.0)) converged_hold = false;

        // ---- aim selection ----
        const char* mode;
        Vec3 aim_use;
        if (arrived || converged_hold || res.holding) {
            aim_use = up;                                  // hold vertical
            mode = arrived ? "ARRIVED" : (res.holding ? "LOWSPD" : "CONVRG");
        } else if (res.reached) {
            aim_use = aim_bcf;                             // trust the MPC plan
            mode = "MPC";
        } else {
            aim_use = proportional_aim(pos, vel, target, env.max_aoa, env.lift_sign);  // fallback
            mode = "PROP";
        }

        // ---- inner loop: aim -> desired body rate -> per-fin MPC -> axes ----
        // Express the desired nose direction in the vessel frame (nose = +y);
        // the pointing error drives a desired pitch/yaw rate (roll target = 0).
        Vec3 aim_vessel;
        try { aim_vessel = to_vec(sc.transform_direction(to_tup(aim_use), bcf, vrf)); }
        catch (...) { aim_vessel = {0.0, 1.0, 0.0}; }
        aim_vessel = v_hat(aim_vessel);
        Vec3 target_rate = {
            clampd(kp_att * aim_vessel[2],  -max_rate, max_rate),   // pitch <- aim_z
            clampd(kp_att * (-aim_vessel[0]),-max_rate, max_rate),  // yaw   <- -aim_x
            0.0                                                     // roll  -> null
        };
        fin.set_target_rate(target_rate);
        FinCommand fcmd = fin.update(omega, real_dt, q);

        // Write the solved per-fin deflection straight to each fin's deployAngle
        // (normalised [-1,1] -> degrees). This is the independent per-fin control.
        const bool commanding = control_on && fins_deployed;
        if (commanding) {
            for (int i = 0; i < nfins && i < (int)fcmd.fin_deflection.size(); ++i) {
                try { fin_mod[i].set_field_float_by_id(
                          "deployAngle", static_cast<float>(fcmd.fin_deflection[i]) * max_defl_deg); }
                catch (...) {}
            }
        }

        // ---- log ----
        log << now << ',' << (commanding ? 1 : 0) << ',' << mode << ',' << alt
            << ',' << horiz_miss << ',' << horiz_speed << ',' << q << ',' << mach
            << ',' << res.aoa_rad * kRadToDeg
            << ',' << omega[0] << ',' << omega[1] << ',' << omega[2]
            << ',' << fcmd.axis[0] << ',' << fcmd.axis[1] << ',' << fcmd.axis[2]
            << ',' << fin.b_id()[0] << ',' << fin.b_id()[1] << ',' << fin.b_id()[2]
            << ',' << fin.roll_sign()[2]
            << ',' << res.pred_pos_err_m << ',' << res.pred_vh_mps
            << ',' << (res.reached ? 1 : 0) << ',' << (fins_deployed ? 1 : 0) << '\n';
        log.flush();

        if (now >= next_print) {
            next_print = now + 1.0;
            const char* fm = fin.mode() == gridfin::FinMode::WaitQ  ? "WaitQ"  :
                             fin.mode() == gridfin::FinMode::Dither ? "Dither" : "Active";
            std::printf("t=%6.1f %s %-7s alt=%7.0f miss=%7.0f vh=%5.1f q=%6.0f M=%4.2f "
                        "aoa=%4.1f w[p,y,r]=[% .2f % .2f % .2f] fin=%-6s "
                        "sgn=%+.0f pred(m=%5.0f vh=%4.1f att=%4.1f)\n",
                        now, commanding ? "CTRL" : "----", mode,
                        alt, horiz_miss, horiz_speed, q, mach, res.aoa_rad * kRadToDeg,
                        omega[0], omega[1], omega[2], fm,
                        fin.roll_sign()[2],
                        res.pred_pos_err_m, res.pred_vh_mps, res.pred_att_err_deg);
            if (!commanding)
                std::printf("   [NOT CONTROLLING] press '1' to deploy grid fins (brakes) "
                            "-> control engages and the MPC starts steering\n");
            if (fcmd.abort)
                std::printf("   [ABORT flag] |body rate| > limit -- roll-priority QP + "
                            "sign-ID recovering (b_roll=%.2f sign=%+.0f)\n",
                            fin.b_id()[2], fin.roll_sign()[2]);
        }

        const double spent = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() - now;
        if (dt - spent > 0.0)
            std::this_thread::sleep_for(std::chrono::duration<double>(dt - spent));
      } catch (const std::exception& e) {
        std::cout << "loop error (continuing): " << e.what() << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }

    take_over_fins(false);   // hand the fins back to normal control + zero angle
    try { ctrl.set_pitch(0); ctrl.set_yaw(0); ctrl.set_roll(0); ctrl.set_sas(true); } catch (...) {}
    std::cout << "exiting; log written to " << opt.log_path << "\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    // Unbuffered stdout so no telemetry/diagnostic line is lost if the process
    // dies abruptly (a buffered crash looks like "it exited printing nothing").
    setvbuf(stdout, nullptr, _IONBF, 0);
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);   // kRPC strings are UTF-8; render CJK correctly
#endif
    Options opt = parse_args(argc, argv);
    try {
        if (opt.probe)   return run_probe(opt);
        return opt.dry_run ? run_dry(opt) : run_live(opt);
    } catch (const std::exception& e) {
        std::cerr << "fatal: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "fatal: unknown (non-std) exception\n";
        return 1;
    }
}
