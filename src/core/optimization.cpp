#include "post2/core/optimization.hpp"

#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <future>
#include <limits>
#include <set>
#include <sstream>
#include <string_view>
#include <thread>

namespace post2::core {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

struct ResolvedPath {
    double* value = nullptr;
    int phase_index = -1;
};

bool fail(std::string* error, const std::string& message)
{
    if (error) {
        *error = message;
    }
    return false;
}

bool starts_with(std::string_view text, std::string_view prefix)
{
    return text.size() >= prefix.size() && text.substr(0, prefix.size()) == prefix;
}

bool consume_literal(std::string_view* text, std::string_view literal)
{
    if (!starts_with(*text, literal)) {
        return false;
    }
    text->remove_prefix(literal.size());
    return true;
}

bool consume_dot(std::string_view* text)
{
    return consume_literal(text, ".");
}

bool consume_identifier(std::string_view* text, std::string_view id)
{
    if (!consume_literal(text, id)) {
        return false;
    }
    return text->empty() || text->front() == '.' || text->front() == '[';
}

bool consume_indexed(std::string_view* text, std::string_view id, std::size_t* index)
{
    std::string_view working = *text;
    if (!consume_identifier(&working, id) || working.empty() || working.front() != '[') {
        return false;
    }
    working.remove_prefix(1);
    std::size_t parsed = 0;
    std::size_t digits = 0;
    while (!working.empty() && working.front() >= '0' && working.front() <= '9') {
        parsed = parsed * 10 + static_cast<std::size_t>(working.front() - '0');
        working.remove_prefix(1);
        ++digits;
    }
    if (digits == 0 || working.empty() || working.front() != ']') {
        return false;
    }
    working.remove_prefix(1);
    *text = working;
    *index = parsed;
    return true;
}

bool finish_value(std::string_view text, double* source, ResolvedPath* target)
{
    if (!text.empty()) {
        return false;
    }
    target->value = source;
    return true;
}

bool resolve_poly(Poly2Config* poly, std::string_view text, ResolvedPath* target)
{
    if (text == "c0") {
        return finish_value({}, &poly->c0, target);
    }
    if (text == "c1") {
        return finish_value({}, &poly->c1, target);
    }
    if (text == "c2") {
        return finish_value({}, &poly->c2, target);
    }
    return false;
}

bool resolve_vec3(Vec3* vec, std::string_view text, ResolvedPath* target)
{
    if (text == "x") {
        return finish_value({}, &vec->x, target);
    }
    if (text == "y") {
        return finish_value({}, &vec->y, target);
    }
    if (text == "z") {
        return finish_value({}, &vec->z, target);
    }
    return false;
}

bool resolve_quaternion(Quaternion* quat, std::string_view text, ResolvedPath* target)
{
    if (text == "w") {
        return finish_value({}, &quat->w, target);
    }
    if (text == "x") {
        return finish_value({}, &quat->x, target);
    }
    if (text == "y") {
        return finish_value({}, &quat->y, target);
    }
    if (text == "z") {
        return finish_value({}, &quat->z, target);
    }
    return false;
}

bool resolve_throttle(ThrottleModelConfig* throttle, std::string_view text, ResolvedPath* target)
{
    if (text == "c0") {
        return finish_value({}, &throttle->c0, target);
    }
    if (text == "c1") {
        return finish_value({}, &throttle->c1, target);
    }
    if (text == "c2") {
        return finish_value({}, &throttle->c2, target);
    }
    if (text == "target_t2w") {
        return finish_value({}, &throttle->target_t2w, target);
    }

    std::size_t index = 0;
    if (consume_indexed(&text, "points", &index)) {
        if (index >= throttle->points.size() || !consume_dot(&text)) {
            return false;
        }
        if (text == "time_s") {
            return finish_value({}, &throttle->points[index].time_s, target);
        }
        if (text == "throttle") {
            return finish_value({}, &throttle->points[index].throttle, target);
        }
    }
    return false;
}

bool resolve_steering(SteeringModelConfig* steering, std::string_view text, ResolvedPath* target)
{
    auto consume_poly_name = [&](std::string_view name, Poly2Config* poly) -> bool {
        std::string_view working = text;
        if (!consume_identifier(&working, name) || !consume_dot(&working)) {
            return false;
        }
        return resolve_poly(poly, working, target);
    };

    if (consume_poly_name("roll", &steering->roll_deg) ||
        consume_poly_name("roll_deg", &steering->roll_deg) ||
        consume_poly_name("pitch", &steering->pitch_deg) ||
        consume_poly_name("pitch_deg", &steering->pitch_deg) ||
        consume_poly_name("yaw", &steering->yaw_deg) ||
        consume_poly_name("yaw_deg", &steering->yaw_deg) ||
        consume_poly_name("azimuth", &steering->azimuth_deg) ||
        consume_poly_name("azimuth_deg", &steering->azimuth_deg) ||
        consume_poly_name("elevation", &steering->elevation_deg) ||
        consume_poly_name("elevation_deg", &steering->elevation_deg)) {
        return true;
    }

    std::string_view working = text;
    if (consume_identifier(&working, "fixed_direction_eci") && consume_dot(&working)) {
        return resolve_vec3(&steering->fixed_direction_eci, working, target);
    }

    std::size_t index = 0;
    working = text;
    if (consume_indexed(&working, "points", &index)) {
        if (index >= steering->points.size() || !consume_dot(&working)) {
            return false;
        }
        if (working == "time_s") {
            return finish_value({}, &steering->points[index].time_s, target);
        }
        if (consume_identifier(&working, "quat") && consume_dot(&working)) {
            return resolve_quaternion(&steering->points[index].quat, working, target);
        }
        return false;
    }

    working = text;
    if (consume_indexed(&working, "segments", &index)) {
        if (index >= steering->segments.size() || !consume_dot(&working)) {
            return false;
        }
        if (working == "start_time_s") {
            return finish_value({}, &steering->segments[index].start_time_s, target);
        }
        if (consume_identifier(&working, "model") && consume_dot(&working)) {
            if (!steering->segments[index].model) {
                return false;
            }
            return resolve_steering(steering->segments[index].model.get(), working, target);
        }
    }

    return false;
}

bool resolve_path(CaseConfig* config, const std::string& path, ResolvedPath* target, std::string* error)
{
    if (!config) {
        return fail(error, "case config is null");
    }

    std::string_view text(path);
    std::string_view working = text;
    if (consume_identifier(&working, "vehicle") && consume_dot(&working)) {
        if (working == "dry_mass_kg") {
            target->value = &config->vehicle.dry_mass_kg;
            return true;
        }
        std::size_t stage_index = 0;
        if (consume_indexed(&working, "stages", &stage_index) && consume_dot(&working)) {
            if (stage_index >= config->vehicle.stages.size()) {
                return fail(error, "vehicle stage index out of range in variable path: " + path);
            }
            if (working == "dry_mass_kg") {
                target->value = &config->vehicle.stages[stage_index].dry_mass_kg;
                return true;
            }
            return fail(error, "unsupported vehicle stage variable path: " + path);
        }
        return fail(error, "unsupported vehicle variable path: " + path);
    }

    std::size_t phase_index = 0;
    if (!consume_indexed(&text, "phases", &phase_index) || !consume_dot(&text)) {
        return fail(error, "variable path must start with phases[index]. or vehicle.");
    }
    if (phase_index >= config->phases.size()) {
        return fail(error, "phase index out of range in variable path: " + path);
    }

    PhaseConfig& phase = config->phases[phase_index];
    target->phase_index = static_cast<int>(phase_index);

    if (text == "duration_s") {
        target->value = &phase.duration_s;
        return true;
    }

    std::size_t action_index = 0;
    working = text;
    if (consume_indexed(&working, "actions", &action_index)) {
        if (action_index >= phase.actions.size() || !consume_dot(&working)) {
            return fail(error, "action index out of range in variable path: " + path);
        }
        if (working == "time_s") {
            target->value = &phase.actions[action_index].time_s;
            return true;
        }
        return fail(error, "unsupported action variable path: " + path);
    }

    working = text;
    if ((consume_identifier(&working, "throttle_model") || consume_identifier(&working, "throttle")) &&
        consume_dot(&working) &&
        resolve_throttle(&phase.throttle_model, working, target)) {
        return true;
    }

    working = text;
    if ((consume_identifier(&working, "steering_model") || consume_identifier(&working, "steering")) &&
        consume_dot(&working) &&
        resolve_steering(&phase.steering_model, working, target)) {
        return true;
    }

    return fail(error, "unsupported variable path: " + path);
}

bool parse_vehicle_stage_dry_mass_path(const std::string& path, std::size_t* stage_index)
{
    std::string_view text(path);
    if (!consume_identifier(&text, "vehicle") || !consume_dot(&text)) {
        return false;
    }
    if (!consume_indexed(&text, "stages", stage_index) || !consume_dot(&text)) {
        return false;
    }
    return text == "dry_mass_kg";
}

bool contains_payload_name(std::string name)
{
    std::transform(name.begin(), name.end(), name.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return name.find("payload") != std::string::npos;
}

bool metric_value(
    const std::vector<OptimizationMetricValue>& metrics,
    const std::string& metric,
    double* value)
{
    const auto it = std::find_if(metrics.begin(), metrics.end(), [&](const OptimizationMetricValue& candidate) {
        return candidate.metric == metric;
    });
    if (it == metrics.end()) {
        return false;
    }
    *value = it->value;
    return true;
}

bool parse_phase_metric(
    const std::string& metric,
    std::size_t* phase_index,
    std::string_view* base_metric)
{
    std::string_view text(metric);
    std::size_t parsed_index = 0;
    if (!consume_indexed(&text, "phases", &parsed_index) || !consume_dot(&text) || text.empty()) {
        return false;
    }
    if (phase_index) {
        *phase_index = parsed_index;
    }
    if (base_metric) {
        *base_metric = text;
    }
    return true;
}

std::string_view canonical_metric_name(const std::string& metric)
{
    std::string_view base;
    if (parse_phase_metric(metric, nullptr, &base)) {
        return base;
    }
    return metric;
}

const LaunchVehicleStateLogEntry* entry_for_metric(
    const StateLog& state_log,
    const std::string& metric)
{
    if (state_log.empty()) {
        return nullptr;
    }
    std::size_t phase_index = 0;
    if (!parse_phase_metric(metric, &phase_index, nullptr)) {
        return &state_log.back();
    }
    for (auto it = state_log.entries().rbegin(); it != state_log.entries().rend(); ++it) {
        if (it->phase_index == static_cast<int>(phase_index)) {
            return &(*it);
        }
    }
    return nullptr;
}

double inclination_deg_for_entry(const LaunchVehicleStateLogEntry& entry)
{
    const Vec3& r = entry.state.position_m;
    const Vec3& v = entry.state.velocity_mps;
    const Vec3 h{
        r.y * v.z - r.z * v.y,
        r.z * v.x - r.x * v.z,
        r.x * v.y - r.y * v.x,
    };
    const double h_norm = post2::vehicle::norm(h);
    if (h_norm <= 0.0) {
        return 0.0;
    }
    const double cosine = std::clamp(h.z / h_norm, -1.0, 1.0);
    return std::acos(cosine) * 180.0 / kPi;
}

double periapsis_altitude_m_for_entry(
    const LaunchVehicleStateLogEntry& entry,
    const CaseConfig& config)
{
    const Vec3& r = entry.state.position_m;
    const Vec3& v = entry.state.velocity_mps;
    const Vec3 h{
        r.y * v.z - r.z * v.y,
        r.z * v.x - r.x * v.z,
        r.x * v.y - r.y * v.x,
    };
    const double h_norm = post2::vehicle::norm(h);
    const double r_norm = post2::vehicle::norm(r);
    const double v_norm = post2::vehicle::norm(v);
    const double rv = post2::vehicle::dot(r, v);
    if (r_norm <= 0.0 || h_norm <= 0.0 || config.earth_mu_m3s2 <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const Vec3 eccentricity_vector =
        ((v_norm * v_norm - config.earth_mu_m3s2 / r_norm) * r -
            rv * v) /
        config.earth_mu_m3s2;
    const double eccentricity = post2::vehicle::norm(eccentricity_vector);
    const double p = h_norm * h_norm / config.earth_mu_m3s2;
    if (1.0 + eccentricity <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return p / (1.0 + eccentricity) - config.earth_radius_m;
}

bool trajectory_metric_value(
    const StateLog& state_log,
    const CaseConfig& config,
    const std::string& metric,
    double* value)
{
    const std::string_view base_metric = canonical_metric_name(metric);
    if (base_metric == "payload_mass_kg") {
        *value = post2::vehicle::payload_stage_dry_mass_kg(config.vehicle);
        return true;
    }
    const LaunchVehicleStateLogEntry* entry = entry_for_metric(state_log, metric);
    if (!entry) {
        return false;
    }
    if (base_metric == "terminal_altitude_m") {
        *value = entry->altitude_m;
        return true;
    }
    if (base_metric == "terminal_speed_mps") {
        *value = entry->speed_mps;
        return true;
    }
    if (base_metric == "inclination_deg") {
        *value = inclination_deg_for_entry(*entry);
        return true;
    }
    if (base_metric == "periapsis_altitude_m") {
        *value = periapsis_altitude_m_for_entry(*entry, config);
        return true;
    }
    return false;
}

double default_metric_scale(const std::string& metric)
{
    const std::string_view base_metric = canonical_metric_name(metric);
    if (base_metric == "terminal_altitude_m" || base_metric == "periapsis_altitude_m") {
        return 1000.0;
    }
    if (base_metric == "terminal_speed_mps") {
        return 100.0;
    }
    if (base_metric == "inclination_deg") {
        return 1.0;
    }
    if (base_metric == "payload_mass_kg") {
        return 1000.0;
    }
    return 1.0;
}

double metric_scale(const std::string& metric, double reference)
{
    return std::max(default_metric_scale(metric), std::abs(reference));
}

void append_limited_message(std::vector<std::string>* messages, const std::string& message)
{
    if (!messages) {
        return;
    }
    if (std::find(messages->begin(), messages->end(), message) != messages->end()) {
        return;
    }
    if (messages->size() < 30) {
        messages->push_back(message);
    }
}

double clamp(double value, double min_value, double max_value)
{
    return std::min(std::max(value, min_value), max_value);
}

// ---- fmincon: variable scaling to z in [-1, 1] -------------------------
//
// Map x in [lb, ub] to z in [-1, 1]:   z = 2 * (x - lb) / (ub - lb) - 1
// Inverse:                              x = lb + (z + 1) * (ub - lb) / 2
//
// All BFGS arithmetic happens in z-space; conversion back to x only happens
// at the boundary when writing variables before each simulation. This makes
// the initial Hessian H = I meaningful across heterogeneous units
// (kg vs sec vs deg) and gives a uniform finite-difference step.

double to_z_scalar(double x, double lb, double ub)
{
    if (ub <= lb) {
        return 0.0;
    }
    return 2.0 * (x - lb) / (ub - lb) - 1.0;
}

double to_x_scalar(double z, double lb, double ub)
{
    if (ub <= lb) {
        return lb;
    }
    return lb + (z + 1.0) * 0.5 * (ub - lb);
}

// ---- fmincon: constraint extraction ------------------------------------

struct EqConstraint {
    std::string metric;
    double target = 0.0;
    double scale = 1.0;
    double weight = 1.0;     // folded as sqrt(weight) into the constraint
};

struct IneqConstraint {
    std::string metric;
    double bound = 0.0;
    double scale = 1.0;
    double weight = 1.0;
    bool upper = false;      // true: metric <= bound (c = (metric-bound)/scale)
                             // false: metric >= bound (c = (bound-metric)/scale)
};

struct ConstraintSet {
    std::vector<EqConstraint> eq;
    std::vector<IneqConstraint> ineq;
};

ConstraintSet build_constraints(const OptimizationConfig& opt)
{
    ConstraintSet cs;
    for (const auto& t : opt.targets) {
        if (t.weight <= 0.0) {
            continue;
        }
        if (t.mode == "equal") {
            cs.eq.push_back({t.metric, t.value, metric_scale(t.metric, t.value), t.weight});
        } else if (t.mode == "range") {
            cs.ineq.push_back({t.metric, t.min_value, metric_scale(t.metric, t.min_value), t.weight, false});
            cs.ineq.push_back({t.metric, t.max_value, metric_scale(t.metric, t.max_value), t.weight, true});
        }
    }
    return cs;
}

// ---- fmincon: point evaluation -----------------------------------------

struct VariableSpec {
    std::string path;
    double x0 = 0.0;
    double lb = 0.0;
    double ub = 0.0;
    int phase_index = -1;
};

struct PointEval {
    bool ok = false;
    std::string error;
    SimulationResult simulation;
    std::vector<OptimizationMetricValue> metrics;
    double objective_f = 0.0;            // sign-adjusted objective (minimization)
    std::vector<double> c_eq;            // unweighted residuals
    std::vector<double> c_ineq;          // unweighted residuals (<= 0 feasible)
    double max_violation = 0.0;          // max(|c_eq|, max(0, c_ineq))
};

double objective_value(
    const OptimizationConfig& opt,
    const StateLog& state_log,
    const CaseConfig& config)
{
    if (opt.mode != "optimize" || !opt.objective.enabled) {
        return 0.0;
    }
    double v = 0.0;
    if (!trajectory_metric_value(state_log, config, opt.objective.metric, &v)) {
        return 0.0;
    }
    const double scaled = v / default_metric_scale(opt.objective.metric);
    if (opt.objective.direction == "maximize") {
        return -opt.objective.weight * scaled;   // minimize negative
    }
    return opt.objective.weight * scaled;
}

PointEval evaluate_at(
    const std::vector<double>& z,                    // in [-1, 1] per variable
    const std::vector<VariableSpec>& vars,
    CaseConfig working,
    const ConstraintSet& cs,
    ITrajectoryService& service,
    int* sim_count,
    std::vector<std::string>* messages)
{
    PointEval pe;
    pe.c_eq.assign(cs.eq.size(), 0.0);
    pe.c_ineq.assign(cs.ineq.size(), 0.0);
    for (std::size_t i = 0; i < vars.size(); ++i) {
        const double x_i = to_x_scalar(z[i], vars[i].lb, vars[i].ub);
        std::string err;
        if (!write_optimization_variable(&working, vars[i].path, x_i, &err)) {
            pe.error = err;
            append_limited_message(messages, err);
            return pe;
        }
    }
    ++(*sim_count);
    pe.simulation = service.simulate(working);
    if (!pe.simulation.ok) {
        pe.error = pe.simulation.error;
        append_limited_message(messages, "simulation failed: " + pe.error);
        return pe;
    }
    if (pe.simulation.state_log.empty()) {
        pe.error = "simulation produced an empty StateLog";
        append_limited_message(messages, pe.error);
        return pe;
    }
    pe.metrics = evaluate_trajectory_metrics(pe.simulation.state_log, working);
    pe.objective_f = objective_value(working.optimization, pe.simulation.state_log, working);
    double max_v = 0.0;
    for (std::size_t i = 0; i < cs.eq.size(); ++i) {
        double m = 0.0;
        if (!trajectory_metric_value(pe.simulation.state_log, working, cs.eq[i].metric, &m)) {
            pe.error = "unsupported metric: " + cs.eq[i].metric;
            append_limited_message(messages, pe.error);
            return pe;
        }
        pe.c_eq[i] = (m - cs.eq[i].target) / cs.eq[i].scale;
        max_v = std::max(max_v, std::abs(pe.c_eq[i]));
    }
    for (std::size_t i = 0; i < cs.ineq.size(); ++i) {
        double m = 0.0;
        if (!trajectory_metric_value(pe.simulation.state_log, working, cs.ineq[i].metric, &m)) {
            pe.error = "unsupported metric: " + cs.ineq[i].metric;
            append_limited_message(messages, pe.error);
            return pe;
        }
        pe.c_ineq[i] = cs.ineq[i].upper
            ? (m - cs.ineq[i].bound) / cs.ineq[i].scale
            : (cs.ineq[i].bound - m) / cs.ineq[i].scale;
        max_v = std::max(max_v, std::max(0.0, pe.c_ineq[i]));
    }
    pe.max_violation = max_v;
    pe.ok = std::isfinite(pe.objective_f);
    return pe;
}

// ---- Augmented Lagrangian value -----------------------------------------

double augmented_lagrangian(
    const PointEval& pe,
    const ConstraintSet& cs,
    double mu,
    const std::vector<double>& lambda_eq,
    const std::vector<double>& lambda_ineq)
{
    double L = pe.objective_f;
    for (std::size_t i = 0; i < cs.eq.size(); ++i) {
        const double w = cs.eq[i].weight;
        const double c = pe.c_eq[i];
        L += lambda_eq[i] * c + 0.5 * mu * w * c * c;
    }
    for (std::size_t i = 0; i < cs.ineq.size(); ++i) {
        const double w = cs.ineq[i].weight;
        const double c = pe.c_ineq[i];
        const double s = lambda_ineq[i] + mu * w * c;
        if (s > 0.0) {
            L += (s * s - lambda_ineq[i] * lambda_ineq[i]) / (2.0 * mu * w);
        } else {
            L += -lambda_ineq[i] * lambda_ineq[i] / (2.0 * mu * w);
        }
    }
    return L;
}

// ---- Forward-difference gradient ----------------------------------------

// Parallelized: each FD probe is an independent simulation, dispatched via
// std::async (windowed to hardware_concurrency in flight at once). Requires
// ITrajectoryService::simulate to be reentrant - LocalTrajectoryService is
// (stateless), RemoteTrajectoryService is not (single socket) and the
// caller is responsible for ensuring the service supports concurrent calls.
bool gradient_forward(
    const std::vector<double>& z,                    // in [-1, 1] per variable
    const std::vector<VariableSpec>& vars,
    const CaseConfig& base,
    const ConstraintSet& cs,
    double mu,
    const std::vector<double>& lambda_eq,
    const std::vector<double>& lambda_ineq,
    const PointEval& pe0,
    double L0,
    double fd_step_fraction,
    ITrajectoryService& service,
    int* sim_count,
    std::vector<std::string>* messages,
    std::vector<double>* grad)
{
    const std::size_t n = z.size();
    grad->assign(n, 0.0);
    // In z-space every variable has range 2 (from -1 to +1), so the step
    // is uniform across variables, so no per-unit scaling is needed.
    const double h_raw = std::max(fd_step_fraction * 2.0, 1.0e-6);

    // Phase 1: pre-compute perturbed z vectors and effective step sizes.
    struct Probe {
        std::vector<double> z_perturbed;
        double eff_h = 0.0;
        bool needs_eval = false;
    };
    std::vector<Probe> probes(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double sign = (z[i] + h_raw <= 1.0) ? 1.0 : -1.0;
        const double h = sign * h_raw;
        probes[i].z_perturbed = z;
        probes[i].z_perturbed[i] = clamp(z[i] + h, -1.0, 1.0);
        probes[i].eff_h = probes[i].z_perturbed[i] - z[i];
        probes[i].needs_eval = std::abs(probes[i].eff_h) >= 1.0e-18;
    }

    // Phase 2: dispatch independent probes in parallel. Each thread captures
    // its own copies of inputs and returns its local sim_count / messages /
    // PointEval; the main thread merges after join.
    struct ProbeResult {
        PointEval pe;
        int local_sim_count = 0;
        std::vector<std::string> local_messages;
    };

    auto run_probe = [&vars, &base, &cs, &service](std::vector<double> z_p) -> ProbeResult {
        ProbeResult r;
        r.pe = evaluate_at(z_p, vars, base, cs, service,
            &r.local_sim_count, &r.local_messages);
        return r;
    };

    const unsigned hw = std::thread::hardware_concurrency();
    const std::size_t max_inflight = (hw == 0) ? 1u : static_cast<std::size_t>(hw);

    std::vector<std::future<ProbeResult>> futures(n);
    // Windowed dispatch: keep up to max_inflight in flight at once.
    std::size_t next = 0;
    std::vector<std::size_t> inflight;
    inflight.reserve(max_inflight);
    while (next < n || !inflight.empty()) {
        while (inflight.size() < max_inflight && next < n) {
            if (probes[next].needs_eval) {
                futures[next] = std::async(std::launch::async, run_probe, probes[next].z_perturbed);
                inflight.push_back(next);
            }
            ++next;
        }
        if (inflight.empty()) {
            break;
        }
        const std::size_t i = inflight.front();
        inflight.erase(inflight.begin());
        ProbeResult r = futures[i].get();
        *sim_count += r.local_sim_count;
        for (const auto& m : r.local_messages) {
            append_limited_message(messages, m);
        }
        if (!r.pe.ok) {
            (*grad)[i] = 0.0;
            continue;
        }
        const double L = augmented_lagrangian(r.pe, cs, mu, lambda_eq, lambda_ineq);
        (*grad)[i] = (L - L0) / probes[i].eff_h;
    }
    (void)pe0;
    return true;
}

// ---- BFGS inverse-Hessian update ----------------------------------------

void bfgs_update(
    std::vector<std::vector<double>>* H,
    const std::vector<double>& s,
    const std::vector<double>& y)
{
    const std::size_t n = s.size();
    double sy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        sy += s[i] * y[i];
    }
    if (sy <= 1.0e-10) {
        return;     // skip update to keep H positive-definite
    }
    const double rho = 1.0 / sy;
    // Hy = H * y
    std::vector<double> Hy(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            Hy[i] += (*H)[i][j] * y[j];
        }
    }
    // yHy = y^T H y
    double yHy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        yHy += y[i] * Hy[i];
    }
    // H <- H + (sy + yHy)/sy^2 * s s^T - (s Hy^T + Hy s^T)/sy
    const double coef_ss = (sy + yHy) * rho * rho;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            (*H)[i][j] += coef_ss * s[i] * s[j]
                - rho * (s[i] * Hy[j] + Hy[i] * s[j]);
        }
    }
}

// ---- Projected line search ----------------------------------------------

struct LineSearchResult {
    bool ok = false;
    double alpha = 0.0;
    PointEval pe;
    double L = 0.0;
};

LineSearchResult line_search(
    const std::vector<double>& z,                    // in [-1, 1]
    const std::vector<double>& d,
    const std::vector<VariableSpec>& vars,
    const CaseConfig& base,
    const ConstraintSet& cs,
    double mu,
    const std::vector<double>& lambda_eq,
    const std::vector<double>& lambda_ineq,
    double L0,
    const std::vector<double>& g0,
    ITrajectoryService& service,
    int* sim_count,
    std::vector<std::string>* messages)
{
    LineSearchResult res;
    constexpr double c1 = 1.0e-4;
    constexpr double rho = 0.5;
    constexpr int max_backtracks = 20;
    double dg = 0.0;
    for (std::size_t i = 0; i < z.size(); ++i) {
        dg += g0[i] * d[i];
    }
    double alpha = 1.0;
    for (int k = 0; k < max_backtracks; ++k) {
        std::vector<double> z_new = z;
        for (std::size_t i = 0; i < z.size(); ++i) {
            z_new[i] = clamp(z[i] + alpha * d[i], -1.0, 1.0);
        }
        PointEval pe = evaluate_at(z_new, vars, base, cs, service, sim_count, messages);
        if (pe.ok) {
            const double L = augmented_lagrangian(pe, cs, mu, lambda_eq, lambda_ineq);
            if (L <= L0 + c1 * alpha * dg) {
                res.ok = true;
                res.alpha = alpha;
                res.pe = std::move(pe);
                res.L = L;
                return res;
            }
        }
        alpha *= rho;
    }
    return res;
}

// ---- BFGS inner solver --------------------------------------------------

struct BfgsResult {
    PointEval pe;
    double L = 0.0;
    std::vector<double> z;              // optimum in z-space [-1, 1]
    bool converged_grad = false;
    int iterations = 0;
};

// BFGS inner solver operating entirely in z-space (each variable on [-1, 1]).
// Bounds are uniform across components which makes H = I a reasonable starting
// Hessian regardless of the original variable units (kg vs sec vs deg).
BfgsResult bfgs_inner(
    std::vector<double> z,                          // in [-1, 1]
    const std::vector<VariableSpec>& vars,
    const CaseConfig& base,
    const ConstraintSet& cs,
    double mu,
    const std::vector<double>& lambda_eq,
    const std::vector<double>& lambda_ineq,
    int max_iter,
    double tol_grad,
    double fd_step_fraction,
    ITrajectoryService& service,
    int* sim_count,
    std::vector<std::string>* messages)
{
    BfgsResult res;
    const std::size_t n = z.size();
    std::vector<std::vector<double>> H(n, std::vector<double>(n, 0.0));
    for (std::size_t i = 0; i < n; ++i) {
        H[i][i] = 1.0;
    }
    PointEval pe = evaluate_at(z, vars, base, cs, service, sim_count, messages);
    if (!pe.ok) {
        res.pe = std::move(pe);
        res.z = std::move(z);
        return res;
    }
    double L = augmented_lagrangian(pe, cs, mu, lambda_eq, lambda_ineq);
    std::vector<double> g;
    gradient_forward(z, vars, base, cs, mu, lambda_eq, lambda_ineq, pe, L,
        fd_step_fraction, service, sim_count, messages, &g);

    for (int it = 0; it < max_iter; ++it) {
        res.iterations = it + 1;
        // Active-bound mask: zero gradient components that point outward of [-1, +1].
        std::vector<double> g_masked = g;
        for (std::size_t i = 0; i < n; ++i) {
            if ((z[i] <= -1.0 + 1.0e-12 && g[i] > 0.0) ||
                (z[i] >=  1.0 - 1.0e-12 && g[i] < 0.0)) {
                g_masked[i] = 0.0;
            }
        }
        double g_inf = 0.0;
        for (double v : g_masked) g_inf = std::max(g_inf, std::abs(v));
        if (g_inf < tol_grad) {
            res.converged_grad = true;
            break;
        }
        // Search direction d = -H * g_masked
        std::vector<double> d(n, 0.0);
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < n; ++j) {
                d[i] -= H[i][j] * g_masked[j];
            }
        }
        // Zero outward components.
        for (std::size_t i = 0; i < n; ++i) {
            if ((z[i] <= -1.0 + 1.0e-12 && d[i] < 0.0) ||
                (z[i] >=  1.0 - 1.0e-12 && d[i] > 0.0)) {
                d[i] = 0.0;
            }
        }
        double dnorm = 0.0;
        for (double v : d) dnorm = std::max(dnorm, std::abs(v));
        if (dnorm < 1.0e-15) {
            res.converged_grad = true;
            break;
        }
        auto ls = line_search(z, d, vars, base, cs, mu, lambda_eq, lambda_ineq,
            L, g, service, sim_count, messages);
        if (!ls.ok) {
            // Line search failure: reset Hessian and exit (caller can restart).
            for (std::size_t i = 0; i < n; ++i) {
                for (std::size_t j = 0; j < n; ++j) {
                    H[i][j] = (i == j) ? 1.0 : 0.0;
                }
            }
            break;
        }
        // Build s and y.
        std::vector<double> z_new(n, 0.0);
        for (std::size_t i = 0; i < n; ++i) {
            z_new[i] = clamp(z[i] + ls.alpha * d[i], -1.0, 1.0);
        }
        std::vector<double> g_new;
        gradient_forward(z_new, vars, base, cs, mu, lambda_eq, lambda_ineq, ls.pe, ls.L,
            fd_step_fraction, service, sim_count, messages, &g_new);
        std::vector<double> s(n, 0.0);
        std::vector<double> y(n, 0.0);
        for (std::size_t i = 0; i < n; ++i) {
            s[i] = z_new[i] - z[i];
            y[i] = g_new[i] - g[i];
        }
        bfgs_update(&H, s, y);
        z = std::move(z_new);
        g = std::move(g_new);
        pe = std::move(ls.pe);
        L = ls.L;
    }
    res.pe = std::move(pe);
    res.L = L;
    res.z = std::move(z);
    return res;
}

struct LocalOptimResult {
    bool started = false;
    bool found_feasible = false;
    std::string error;
    int iterations = 0;
    PointEval report_pe;
    std::vector<double> report_z;
};

LocalOptimResult run_local_augmented_lagrangian(
    const std::vector<double>& start_z,
    const std::vector<VariableSpec>& vars,
    const CaseConfig& working,
    const ConstraintSet& cs,
    const OptimizationConfig& optimization,
    int iteration_budget,
    double tol,
    double fd_step_fraction,
    ITrajectoryService& service,
    int* sim_count,
    std::vector<std::string>* messages)
{
    LocalOptimResult out;
    out.started = true;

    int bfgs_budget = std::max(1, iteration_budget);
    const int initial_budget = bfgs_budget;
    double mu = 10.0;
    constexpr double mu_max = 1.0e8;
    std::vector<double> lambda_eq(cs.eq.size(), 0.0);
    std::vector<double> lambda_ineq(cs.ineq.size(), 0.0);
    double prev_violation = std::numeric_limits<double>::infinity();
    constexpr int max_outer = 20;

    PointEval best_feasible_pe;
    std::vector<double> best_feasible_z;

    auto consider_feasible = [&](const PointEval& pe, const std::vector<double>& z_at) {
        if (!pe.ok || pe.max_violation > tol) {
            return;
        }
        if (!out.found_feasible || pe.objective_f < best_feasible_pe.objective_f) {
            best_feasible_pe = pe;
            best_feasible_z = z_at;
            out.found_feasible = true;
        }
    };

    PointEval least_infeasible_pe;
    std::vector<double> least_infeasible_z;
    bool found_least_infeasible = false;

    auto consider_least_infeasible = [&](const PointEval& pe, const std::vector<double>& z_at) {
        if (!pe.ok) {
            return;
        }
        if (!found_least_infeasible || pe.max_violation < least_infeasible_pe.max_violation) {
            least_infeasible_pe = pe;
            least_infeasible_z = z_at;
            found_least_infeasible = true;
        }
    };

    std::vector<double> z = start_z;
    PointEval init_pe = evaluate_at(z, vars, working, cs, service, sim_count, messages);
    if (!init_pe.ok) {
        out.error = "initial simulation failed: " + init_pe.error;
        return out;
    }
    PointEval best_pe = init_pe;
    consider_feasible(init_pe, z);
    consider_least_infeasible(init_pe, z);

    for (int outer = 0; outer < max_outer && bfgs_budget > 0; ++outer) {
        const int per_outer_iters =
            std::max(3, std::min(bfgs_budget, initial_budget / 4 + 5));
        BfgsResult inner = bfgs_inner(z, vars, working, cs, mu, lambda_eq, lambda_ineq,
            /*max_iter=*/per_outer_iters,
            /*tol_grad=*/std::max(1.0e-6, tol),
            fd_step_fraction,
            service, sim_count, messages);
        bfgs_budget -= inner.iterations;
        out.iterations += inner.iterations;
        if (!inner.pe.ok) {
            append_limited_message(messages, "inner BFGS failed: " + inner.pe.error);
            break;
        }

        z = inner.z;
        best_pe = inner.pe;
        consider_feasible(inner.pe, z);
        consider_least_infeasible(inner.pe, z);

        // KKT-style convergence: feasibility and inner-loop stationarity.
        if (inner.converged_grad && best_pe.max_violation < tol) {
            break;
        }

        // Powell-Hestenes-Rockafellar updates.
        if (best_pe.max_violation < 0.25 * prev_violation) {
            for (std::size_t i = 0; i < cs.eq.size(); ++i) {
                lambda_eq[i] += mu * cs.eq[i].weight * best_pe.c_eq[i];
            }
            for (std::size_t i = 0; i < cs.ineq.size(); ++i) {
                lambda_ineq[i] = std::max(0.0,
                    lambda_ineq[i] + mu * cs.ineq[i].weight * best_pe.c_ineq[i]);
            }
        } else if (mu < mu_max) {
            mu *= 10.0;
        }
        prev_violation = best_pe.max_violation;
    }

    if (out.found_feasible) {
        out.report_pe = best_feasible_pe;
        out.report_z = best_feasible_z;
    } else if (found_least_infeasible) {
        out.report_pe = least_infeasible_pe;
        out.report_z = least_infeasible_z;
    } else {
        out.report_pe = best_pe;
        out.report_z = z;
    }
    return out;
}

// ---- SQP: dense Gaussian elimination with partial pivoting --------------
//
// Solves A x = b in-place (A is mutated). Returns false if the matrix is
// numerically singular (small pivot relative to per-row scale). Sized for
// the KKT system (a few dozen rows/cols at most), so dense factorization
// is fine and we don't need anything fancier.

bool solve_linear_system(
    std::vector<std::vector<double>> A,
    std::vector<double> b,
    std::vector<double>* x)
{
    const std::size_t n = b.size();
    if (n == 0 || A.size() != n) {
        return false;
    }
    for (const auto& row : A) {
        if (row.size() != n) {
            return false;
        }
    }
    // Row scaling for pivot threshold.
    std::vector<double> row_scale(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            row_scale[i] = std::max(row_scale[i], std::abs(A[i][j]));
        }
        if (row_scale[i] == 0.0) {
            row_scale[i] = 1.0;
        }
    }
    for (std::size_t k = 0; k < n; ++k) {
        // Scaled partial pivot.
        std::size_t pivot = k;
        double best = std::abs(A[k][k]) / row_scale[k];
        for (std::size_t i = k + 1; i < n; ++i) {
            const double cand = std::abs(A[i][k]) / row_scale[i];
            if (cand > best) {
                best = cand;
                pivot = i;
            }
        }
        if (best < 1.0e-14) {
            return false;
        }
        if (pivot != k) {
            std::swap(A[k], A[pivot]);
            std::swap(b[k], b[pivot]);
            std::swap(row_scale[k], row_scale[pivot]);
        }
        const double inv = 1.0 / A[k][k];
        for (std::size_t i = k + 1; i < n; ++i) {
            const double factor = A[i][k] * inv;
            if (factor == 0.0) {
                continue;
            }
            for (std::size_t j = k + 1; j < n; ++j) {
                A[i][j] -= factor * A[k][j];
            }
            b[i] -= factor * b[k];
            A[i][k] = 0.0;
        }
    }
    x->assign(n, 0.0);
    for (std::size_t k = n; k > 0; --k) {
        const std::size_t i = k - 1;
        double sum = b[i];
        for (std::size_t j = i + 1; j < n; ++j) {
            sum -= A[i][j] * (*x)[j];
        }
        (*x)[i] = sum / A[i][i];
    }
    return true;
}

// ---- SQP: finite-difference jacobians -----------------------------------
//
// Central differences when both perturbed points stay strictly inside the
// box [-1, +1]; falls back to one-sided forward (or backward) differences
// near boundaries. If a perturbed simulation fails, we degrade gracefully:
// switch to the surviving one-sided value, or zero out the column if both
// sides failed. Probes run sequentially because not every ITrajectoryService
// is reentrant (RemoteTrajectoryService holds a single socket).

struct SqpJacobians {
    std::vector<double> grad_f;                       // size n
    std::vector<std::vector<double>> J_eq;            // m_eq x n
    std::vector<std::vector<double>> J_ineq;          // m_ineq x n
    bool ok = false;
};

// Parallelized: each FD probe (plus or minus) for each variable is an
// independent simulation and dispatched via std::async, windowed to
// hardware_concurrency in flight at once. Same reentrancy contract as
// fmincon's gradient_forward: LocalTrajectoryService is stateless and
// safe to call concurrently, but RemoteTrajectoryService is not (single
// socket) - the caller is responsible for using a service that supports
// concurrent simulate().
SqpJacobians compute_fd_jacobians(
    const std::vector<double>& z,
    const std::vector<VariableSpec>& vars,
    const CaseConfig& base,
    const ConstraintSet& cs,
    const PointEval& pe0,
    double fd_step_fraction,
    ITrajectoryService& service,
    int* sim_count,
    std::vector<std::string>* messages)
{
    const std::size_t n = z.size();
    SqpJacobians j;
    j.grad_f.assign(n, 0.0);
    j.J_eq.assign(cs.eq.size(), std::vector<double>(n, 0.0));
    j.J_ineq.assign(cs.ineq.size(), std::vector<double>(n, 0.0));
    const double h_base = std::max(fd_step_fraction * 2.0, 1.0e-4);

    // Phase 1: pre-compute every probe we might want. Each variable can
    // contribute up to two probes (one per side); the assembly step then
    // picks central / forward / backward based on what came back ok.
    struct VarProbes {
        double h_plus = 0.0;
        double h_minus = 0.0;
        std::size_t plus_idx = static_cast<std::size_t>(-1);
        std::size_t minus_idx = static_cast<std::size_t>(-1);
    };
    std::vector<VarProbes> var_probes(n);
    std::vector<std::vector<double>> probe_z;
    probe_z.reserve(2 * n);
    for (std::size_t i = 0; i < n; ++i) {
        const double zi = z[i];
        const double h_plus = std::min(h_base, 1.0 - zi);
        const double h_minus = std::min(h_base, zi + 1.0);
        var_probes[i].h_plus = h_plus;
        var_probes[i].h_minus = h_minus;
        if (h_plus > 1.0e-9) {
            std::vector<double> zp = z;
            zp[i] = zi + h_plus;
            var_probes[i].plus_idx = probe_z.size();
            probe_z.push_back(std::move(zp));
        }
        if (h_minus > 1.0e-9) {
            std::vector<double> zm = z;
            zm[i] = zi - h_minus;
            var_probes[i].minus_idx = probe_z.size();
            probe_z.push_back(std::move(zm));
        }
    }

    // Phase 2: dispatch probes in parallel windows.
    struct ProbeResult {
        PointEval pe;
        int local_sim_count = 0;
        std::vector<std::string> local_messages;
    };
    auto run_probe = [&vars, &base, &cs, &service](std::vector<double> zp) -> ProbeResult {
        ProbeResult r;
        r.pe = evaluate_at(zp, vars, base, cs, service,
            &r.local_sim_count, &r.local_messages);
        return r;
    };

    const unsigned hw = std::thread::hardware_concurrency();
    const std::size_t max_inflight = (hw == 0) ? 1u : static_cast<std::size_t>(hw);
    const std::size_t total = probe_z.size();
    std::vector<std::future<ProbeResult>> futures(total);
    std::vector<ProbeResult> results(total);

    std::size_t next = 0;
    std::vector<std::size_t> inflight;
    inflight.reserve(max_inflight);
    while (next < total || !inflight.empty()) {
        while (inflight.size() < max_inflight && next < total) {
            futures[next] = std::async(std::launch::async, run_probe, probe_z[next]);
            inflight.push_back(next);
            ++next;
        }
        if (inflight.empty()) {
            break;
        }
        const std::size_t idx = inflight.front();
        inflight.erase(inflight.begin());
        results[idx] = futures[idx].get();
        *sim_count += results[idx].local_sim_count;
        for (const auto& m : results[idx].local_messages) {
            append_limited_message(messages, m);
        }
    }

    // Phase 3: assemble derivatives.
    for (std::size_t i = 0; i < n; ++i) {
        const auto& vp = var_probes[i];
        const bool have_plus = vp.plus_idx != static_cast<std::size_t>(-1)
            && results[vp.plus_idx].pe.ok;
        const bool have_minus = vp.minus_idx != static_cast<std::size_t>(-1)
            && results[vp.minus_idx].pe.ok;

        // Prefer symmetric central diff (O(h^2)); otherwise fall through
        // to single-sided forward/backward (O(h)).
        if (have_plus && have_minus &&
            std::abs(vp.h_plus - vp.h_minus) < 1.0e-12) {
            const PointEval& pp = results[vp.plus_idx].pe;
            const PointEval& pm = results[vp.minus_idx].pe;
            const double inv = 1.0 / (vp.h_plus + vp.h_minus);
            j.grad_f[i] = (pp.objective_f - pm.objective_f) * inv;
            for (std::size_t k = 0; k < cs.eq.size(); ++k) {
                j.J_eq[k][i] = (pp.c_eq[k] - pm.c_eq[k]) * inv;
            }
            for (std::size_t k = 0; k < cs.ineq.size(); ++k) {
                j.J_ineq[k][i] = (pp.c_ineq[k] - pm.c_ineq[k]) * inv;
            }
        } else if (have_plus) {
            const PointEval& pp = results[vp.plus_idx].pe;
            const double inv = 1.0 / vp.h_plus;
            j.grad_f[i] = (pp.objective_f - pe0.objective_f) * inv;
            for (std::size_t k = 0; k < cs.eq.size(); ++k) {
                j.J_eq[k][i] = (pp.c_eq[k] - pe0.c_eq[k]) * inv;
            }
            for (std::size_t k = 0; k < cs.ineq.size(); ++k) {
                j.J_ineq[k][i] = (pp.c_ineq[k] - pe0.c_ineq[k]) * inv;
            }
        } else if (have_minus) {
            const PointEval& pm = results[vp.minus_idx].pe;
            const double inv = 1.0 / vp.h_minus;
            j.grad_f[i] = (pe0.objective_f - pm.objective_f) * inv;
            for (std::size_t k = 0; k < cs.eq.size(); ++k) {
                j.J_eq[k][i] = (pe0.c_eq[k] - pm.c_eq[k]) * inv;
            }
            for (std::size_t k = 0; k < cs.ineq.size(); ++k) {
                j.J_ineq[k][i] = (pe0.c_ineq[k] - pm.c_ineq[k]) * inv;
            }
        } else {
            append_limited_message(messages,
                "sqp: finite difference both sides failed for variable index "
                    + std::to_string(i));
        }
    }
    j.ok = true;
    return j;
}

// ---- SQP: KKT step ------------------------------------------------------
//
// Solve the equality-constrained QP step:
//   min   grad_f^T dz + 0.5 dz^T (B + reg*I) dz
//   s.t.  J_act * dz + c_act = 0
//
// via the augmented KKT system
//   [B + reg*I, J^T] [dz    ]   [-grad_f]
//   [J,        0   ] [lambda] = [-c_act ]
//
// Returns lambda with the same row order as J_active. If the system is
// singular at all regularizations we try, returns false and the caller
// should fall back to a steepest-descent step.

bool solve_kkt_step(
    const std::vector<std::vector<double>>& B,
    const std::vector<double>& grad_f,
    const std::vector<std::vector<double>>& J_active,
    const std::vector<double>& c_active,
    std::vector<double>* dz,
    std::vector<double>* lambda)
{
    const std::size_t n = grad_f.size();
    const std::size_t m = c_active.size();
    if (B.size() != n) {
        return false;
    }
    dz->assign(n, 0.0);
    lambda->assign(m, 0.0);
    double reg_p = 1.0e-8;
    // Dual regularization keeps |lambda| bounded when J*J^T is near-singular
    // (which happens near feasibility when constraint Jacobians share row
    // structure). Without it, KKT returns lambdas of thousands which then
    // pump rho up and freeze the line search. The dual reg trades exact
    // constraint linearization for bounded multipliers - the constraint
    // residual after the step shrinks by (1 - reg_d/(eigenvalue)) instead
    // of going to zero, but multiplier magnitudes stay O(1) which is what
    // we need for the merit function to behave.
    double reg_d = 1.0e-4;
    for (int attempt = 0; attempt < 8; ++attempt) {
        const std::size_t N = n + m;
        std::vector<std::vector<double>> A(N, std::vector<double>(N, 0.0));
        std::vector<double> b(N, 0.0);
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t k = 0; k < n; ++k) {
                A[i][k] = B[i][k];
            }
            A[i][i] += reg_p;
        }
        for (std::size_t r = 0; r < m; ++r) {
            for (std::size_t c = 0; c < n; ++c) {
                A[n + r][c] = J_active[r][c];
                A[c][n + r] = J_active[r][c];
            }
            A[n + r][n + r] = -reg_d;
        }
        for (std::size_t i = 0; i < n; ++i) {
            b[i] = -grad_f[i];
        }
        for (std::size_t r = 0; r < m; ++r) {
            b[n + r] = -c_active[r];
        }
        std::vector<double> sol;
        if (solve_linear_system(A, b, &sol)) {
            for (std::size_t i = 0; i < n; ++i) {
                (*dz)[i] = sol[i];
            }
            for (std::size_t r = 0; r < m; ++r) {
                (*lambda)[r] = sol[n + r];
            }
            return true;
        }
        reg_p *= 100.0;
        reg_d *= 10.0;
    }
    return false;
}

// ---- SQP: damped BFGS update on Lagrangian Hessian B --------------------

void sqp_bfgs_update(
    std::vector<std::vector<double>>* B,
    const std::vector<double>& s,
    const std::vector<double>& y)
{
    const std::size_t n = s.size();
    if (n == 0) {
        return;
    }
    // Bs = B * s
    std::vector<double> Bs(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            Bs[i] += (*B)[i][j] * s[j];
        }
    }
    double sBs = 0.0;
    double sy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        sBs += s[i] * Bs[i];
        sy += s[i] * y[i];
    }
    if (sBs <= 0.0) {
        return;  // numerical: skip update
    }
    // Powell's damped BFGS: keep B positive definite even when sy < 0.2 sBs.
    double theta = 1.0;
    if (sy < 0.2 * sBs) {
        if (sBs - sy <= 0.0) {
            return;
        }
        theta = 0.8 * sBs / (sBs - sy);
    }
    std::vector<double> r(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        r[i] = theta * y[i] + (1.0 - theta) * Bs[i];
    }
    double sr = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        sr += s[i] * r[i];
    }
    if (sr <= 1.0e-12 || sBs <= 1.0e-12) {
        return;
    }
    // B_new = B - (B s)(B s)^T / sBs + r r^T / sr
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t k = 0; k < n; ++k) {
            (*B)[i][k] += -(Bs[i] * Bs[k]) / sBs + (r[i] * r[k]) / sr;
        }
    }
}

// ---- SQP: merit function (L1 exact penalty) -----------------------------

double sqp_merit(
    const PointEval& pe,
    double rho_eq,
    double rho_ineq)
{
    double m = pe.objective_f;
    for (double c : pe.c_eq) {
        m += rho_eq * std::abs(c);
    }
    for (double c : pe.c_ineq) {
        m += rho_ineq * std::max(0.0, c);
    }
    return m;
}

double sqp_violation_l1(const PointEval& pe)
{
    double v = 0.0;
    for (double c : pe.c_eq) {
        v += std::abs(c);
    }
    for (double c : pe.c_ineq) {
        v += std::max(0.0, c);
    }
    return v;
}

// ---- SQP: main NLP solver -----------------------------------------------
//
// Algorithm: bound-constrained SQP with L1 exact-penalty merit line search
// and damped-BFGS Lagrangian Hessian update. Each major iteration:
//   1. Finite-difference grad_f, J_eq, J_ineq at z.
//   2. Build the active set: all equalities + inequalities with
//      c_ineq > -active_margin (currently violated or barely satisfied).
//   3. Solve the equality-constrained KKT step:
//        [B + reg*I, J^T] [dz    ]   [-grad_f]
//        [J,        0   ] [lambda] = [-c_act ]
//      where B is the current Hessian approximation. Fall back to
//      steepest-descent on the merit penalty if KKT is singular.
//   4. Fraction-to-boundary cap for the [-1, +1] box on z.
//   5. Backtracking line search on the L1 merit. The penalty parameters
//      rho_eq / rho_ineq are grown when the line search rejects the step
//      or when |lambda| exceeds rho (so merit stays an exact descent
//      direction).
//   6. Damped-BFGS update of B from (s, y) where y is the change in the
//      Lagrangian gradient with the latest multipliers.
//
// Bookkeeping tracks both the best feasible point (by objective) and the
// least-infeasible point as a fallback when no feasible iterate is found.

LocalOptimResult run_sqp_nlp(
    const std::vector<double>& start_z,
    const std::vector<VariableSpec>& vars,
    const CaseConfig& base,
    const ConstraintSet& cs,
    const OptimizationConfig& /*optimization*/,
    int iteration_budget,
    double tol,
    double fd_step_fraction,
    ITrajectoryService& service,
    int* sim_count,
    std::vector<std::string>* messages)
{
    LocalOptimResult out;
    out.started = true;

    const std::size_t n = start_z.size();
    if (n == 0) {
        out.error = "sqp: no variables";
        return out;
    }

    std::vector<double> z = start_z;
    for (auto& v : z) {
        v = clamp(v, -1.0, 1.0);
    }

    PointEval pe = evaluate_at(z, vars, base, cs, service, sim_count, messages);
    if (!pe.ok) {
        out.error = "sqp: initial simulation failed: " + pe.error;
        out.report_pe = pe;
        out.report_z = z;
        return out;
    }

    PointEval best_feasible_pe;
    std::vector<double> best_feasible_z;
    PointEval least_infeasible_pe = pe;
    std::vector<double> least_infeasible_z = z;
    bool have_least_infeasible = true;

    auto consider = [&](const PointEval& candidate, const std::vector<double>& z_at) {
        if (!candidate.ok) {
            return;
        }
        if (candidate.max_violation <= tol) {
            if (!out.found_feasible ||
                candidate.objective_f < best_feasible_pe.objective_f) {
                best_feasible_pe = candidate;
                best_feasible_z = z_at;
                out.found_feasible = true;
            }
        }
        if (!have_least_infeasible ||
            candidate.max_violation < least_infeasible_pe.max_violation) {
            least_infeasible_pe = candidate;
            least_infeasible_z = z_at;
            have_least_infeasible = true;
        }
    };
    consider(pe, z);

    // BFGS Hessian on the Lagrangian, in z-space. Identity is a reasonable
    // start because every variable shares the box [-1, +1].
    std::vector<std::vector<double>> B(n, std::vector<double>(n, 0.0));
    for (std::size_t i = 0; i < n; ++i) {
        B[i][i] = 1.0;
    }

    // Full-size multipliers; inactive entries stay at zero.
    std::vector<double> lambda_eq(cs.eq.size(), 0.0);
    std::vector<double> lambda_ineq(cs.ineq.size(), 0.0);

    // L1 penalty parameters. We grow these whenever the step is rejected or
    // |lambda| exceeds the current rho.
    double rho_eq = 10.0;
    double rho_ineq = 10.0;
    constexpr double rho_max = 1.0e6;
    // Active margin small enough that opposite halves of a range target
    // (e.g., periapsis lower + upper bounds, which produce linearly
    // dependent Jacobian rows) don't both land in the active set together;
    // otherwise the KKT matrix is structurally singular.
    const double active_margin = std::max(1.0e-3, tol);

    // For BFGS we need grad_L at the previous iterate with the same lambda
    // used at the previous step. Cache it across iterations.
    std::vector<double> grad_L_prev;
    std::vector<double> z_prev;
    SqpJacobians jac_prev;
    bool have_prev = false;

    int active_eq_count = static_cast<int>(cs.eq.size());
    int active_ineq_count = 0;
    int kkt_singular_events = 0;
    int line_search_failures = 0;

    int it = 0;
    for (; it < iteration_budget; ++it) {
        out.iterations = it + 1;

        SqpJacobians jac = compute_fd_jacobians(z, vars, base, cs, pe,
            fd_step_fraction, service, sim_count, messages);
        if (!jac.ok) {
            append_limited_message(messages, "sqp: FD jacobian failed");
            break;
        }

        // BFGS update from the previous step (skip on iteration 0). Uses
        // multipliers from the previous iteration; the gradient difference
        // y is computed at the new and previous z under the same lambda.
        if (have_prev && z_prev.size() == n) {
            std::vector<double> s(n, 0.0);
            for (std::size_t i = 0; i < n; ++i) {
                s[i] = z[i] - z_prev[i];
            }
            std::vector<double> grad_L_cur(n, 0.0);
            for (std::size_t i = 0; i < n; ++i) {
                grad_L_cur[i] = jac.grad_f[i];
                for (std::size_t k = 0; k < cs.eq.size(); ++k) {
                    grad_L_cur[i] += lambda_eq[k] * jac.J_eq[k][i];
                }
                for (std::size_t k = 0; k < cs.ineq.size(); ++k) {
                    grad_L_cur[i] += lambda_ineq[k] * jac.J_ineq[k][i];
                }
            }
            std::vector<double> y(n, 0.0);
            for (std::size_t i = 0; i < n; ++i) {
                y[i] = grad_L_cur[i] - grad_L_prev[i];
            }
            sqp_bfgs_update(&B, s, y);
        }

        // Build the active set.
        std::vector<std::vector<double>> J_active;
        std::vector<double> c_active;
        std::vector<int> ineq_map;
        J_active.reserve(cs.eq.size() + cs.ineq.size());
        c_active.reserve(cs.eq.size() + cs.ineq.size());
        for (std::size_t k = 0; k < cs.eq.size(); ++k) {
            J_active.push_back(jac.J_eq[k]);
            c_active.push_back(pe.c_eq[k]);
        }
        for (std::size_t k = 0; k < cs.ineq.size(); ++k) {
            if (pe.c_ineq[k] > -active_margin || lambda_ineq[k] > 0.0) {
                J_active.push_back(jac.J_ineq[k]);
                c_active.push_back(pe.c_ineq[k]);
                ineq_map.push_back(static_cast<int>(k));
            }
        }
        active_eq_count = static_cast<int>(cs.eq.size());
        active_ineq_count = static_cast<int>(ineq_map.size());

        // Solve KKT, with fallback to steepest descent on the L1 merit.
        std::vector<double> dz;
        std::vector<double> lambda_active;
        const bool kkt_ok = solve_kkt_step(B, jac.grad_f, J_active, c_active,
            &dz, &lambda_active);
        if (!kkt_ok) {
            ++kkt_singular_events;
            dz.assign(n, 0.0);
            for (std::size_t i = 0; i < n; ++i) {
                double g = jac.grad_f[i];
                for (std::size_t k = 0; k < cs.eq.size(); ++k) {
                    const double sign = pe.c_eq[k] >= 0.0 ? 1.0 : -1.0;
                    g += rho_eq * sign * jac.J_eq[k][i];
                }
                for (std::size_t k = 0; k < cs.ineq.size(); ++k) {
                    if (pe.c_ineq[k] > 0.0) {
                        g += rho_ineq * jac.J_ineq[k][i];
                    }
                }
                dz[i] = -g;
            }
            lambda_active.assign(c_active.size(), 0.0);
        }

        // Update full-size multipliers from this step.
        for (std::size_t k = 0; k < cs.eq.size(); ++k) {
            lambda_eq[k] = lambda_active.empty() ? 0.0 : lambda_active[k];
        }
        for (auto& v : lambda_ineq) {
            v = 0.0;
        }
        for (std::size_t k = 0; k < ineq_map.size(); ++k) {
            const double l = lambda_active[cs.eq.size() + k];
            lambda_ineq[ineq_map[k]] = std::max(0.0, l);
        }

        // Penalty parameters: clip lambda contribution to keep rho moderate.
        // Larger rho gives constraints stricter weight (good for driving
        // feasibility) but freezes the line search if it exceeds the
        // f-vs-c trade-off the user actually wants. The clamp here keeps
        // rho near 1/tol which gives the merit a sensible scale for
        // typical scaled c values of order tol.
        double lambda_eq_max = 0.0;
        double lambda_ineq_max = 0.0;
        for (double v : lambda_eq) {
            lambda_eq_max = std::max(lambda_eq_max, std::abs(v));
        }
        for (double v : lambda_ineq) {
            lambda_ineq_max = std::max(lambda_ineq_max, v);
        }
        const double rho_target = 1.0 / std::max(tol, 1.0e-3);
        rho_eq = std::min(rho_max,
            std::max(rho_eq, 1.5 * std::min(lambda_eq_max, rho_target)));
        rho_ineq = std::min(rho_max,
            std::max(rho_ineq, 1.5 * std::min(lambda_ineq_max, rho_target)));

        // Zero outward dz components at active bounds.
        for (std::size_t i = 0; i < n; ++i) {
            if (z[i] >= 1.0 - 1.0e-12 && dz[i] > 0.0) dz[i] = 0.0;
            if (z[i] <= -1.0 + 1.0e-12 && dz[i] < 0.0) dz[i] = 0.0;
        }

        // Fraction to boundary on the [-1, +1] box.
        double alpha_bound = 1.0;
        for (std::size_t i = 0; i < n; ++i) {
            if (dz[i] > 1.0e-18) {
                alpha_bound = std::min(alpha_bound, (1.0 - z[i]) / dz[i]);
            } else if (dz[i] < -1.0e-18) {
                alpha_bound = std::min(alpha_bound, (-1.0 - z[i]) / dz[i]);
            }
        }
        alpha_bound = std::max(0.0, alpha_bound);

        double dz_norm = 0.0;
        for (double v : dz) {
            dz_norm = std::max(dz_norm, std::abs(v));
        }
        double grad_f_inf = 0.0;
        for (double v : jac.grad_f) {
            grad_f_inf = std::max(grad_f_inf, std::abs(v));
        }

        // First-order SQP can stall at a "linearization-stationary" point
        // that isn't a true optimum: when grad_f lies (approximately) in the
        // row space of J_active, the projected step -P_null*grad_f is tiny
        // even though a finite step along the curved feasible manifold
        // would still improve f. Detect this (dz tiny in the objective
        // gradient direction + feasible) and augment dz with a normalized
        // steepest-descent objective pull. The constraints will re-violate
        // and the merit line search will renegotiate via rho growth. The
        // augmented step is also a large enough s to give the BFGS update
        // meaningful curvature data for subsequent iterations.
        constexpr double kStallStep = 0.15;
        double grad_dot_dz = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            grad_dot_dz += jac.grad_f[i] * dz[i];
        }
        const double obj_descent = (grad_f_inf > 0.0)
            ? (-grad_dot_dz / grad_f_inf) : 0.0;
        // Stuck at linearization-stationarity: feasible AND the KKT step
        // makes negligible objective progress (the alignment between dz and
        // -grad_f, scaled by grad magnitude, is small). Switch to a pure
        // steepest-descent step on the objective: the constraints will
        // re-violate but the merit line search + outer iterations will
        // renegotiate. This is what lets SQP traverse curved feasible
        // manifolds where the linear projection is degenerate.
        bool augment_active = false;
        if (pe.max_violation <= tol * 5.0 &&
            obj_descent < kStallStep * 0.5 &&
            grad_f_inf > tol * 5.0) {
            augment_active = true;
            rho_eq = std::min(rho_eq, 10.0);
            rho_ineq = std::min(rho_ineq, 10.0);
            // Replace with pure objective descent normalized to the stall
            // step. Cap z-magnitude so we never step further than
            // kStallStep in any one variable.
            for (std::size_t i = 0; i < n; ++i) {
                dz[i] = -(jac.grad_f[i] / grad_f_inf) * kStallStep;
            }
            for (std::size_t i = 0; i < n; ++i) {
                if (z[i] >= 1.0 - 1.0e-12 && dz[i] > 0.0) dz[i] = 0.0;
                if (z[i] <= -1.0 + 1.0e-12 && dz[i] < 0.0) dz[i] = 0.0;
            }
            alpha_bound = 1.0;
            for (std::size_t i = 0; i < n; ++i) {
                if (dz[i] > 1.0e-18) {
                    alpha_bound = std::min(alpha_bound, (1.0 - z[i]) / dz[i]);
                } else if (dz[i] < -1.0e-18) {
                    alpha_bound = std::min(alpha_bound, (-1.0 - z[i]) / dz[i]);
                }
            }
            alpha_bound = std::max(0.0, alpha_bound);
            dz_norm = 0.0;
            for (double v : dz) {
                dz_norm = std::max(dz_norm, std::abs(v));
            }
        }

        // True KKT stationarity: tiny step at feasible point AND no
        // objective gradient to chase. Real convergence.
        if (pe.max_violation <= tol &&
            dz_norm < tol &&
            grad_f_inf < tol * 5.0) {
            append_limited_message(messages,
                "sqp: converged - stationarity at feasible point");
            break;
        }

        if (dz_norm < 1.0e-12 || alpha_bound <= 0.0) {
            // No useful direction. If infeasible, ramp rho; otherwise give up.
            if (rho_eq < rho_max || rho_ineq < rho_max) {
                rho_eq = std::min(rho_max, rho_eq * 10.0);
                rho_ineq = std::min(rho_max, rho_ineq * 10.0);
                continue;
            }
            append_limited_message(messages,
                "sqp: stuck - no descent direction");
            break;
        }

        // Inner rho-retry loop: a line search rejection only means the
        // current rho was too small to make the merit a descent direction
        // for this (dz, alpha_bound). Grow rho and retry without recomputing
        // the (expensive) FD jacobian. Capped to avoid spinning.
        bool accepted = false;
        PointEval pe_new;
        std::vector<double> z_new(n, 0.0);
        double alpha_final = 0.0;
        const double viol_cap = std::max(0.5, 5.0 * pe.max_violation);
        for (int rho_retry = 0; rho_retry < 6 && !accepted; ++rho_retry) {
            const double phi0 = sqp_merit(pe, rho_eq, rho_ineq);
            const double viol0 = pe.max_violation;
            double alpha = std::min(1.0, alpha_bound);
            for (int ls = 0; ls < 20; ++ls) {
                for (std::size_t i = 0; i < n; ++i) {
                    z_new[i] = clamp(z[i] + alpha * dz[i], -1.0, 1.0);
                }
                pe_new = evaluate_at(z_new, vars, base, cs, service,
                    sim_count, messages);
                if (pe_new.ok) {
                    const double phi_new = sqp_merit(pe_new, rho_eq, rho_ineq);
                    if (phi_new < phi0 - 1.0e-6 * alpha *
                            std::max(1.0, std::abs(phi0))) {
                        accepted = true;
                        alpha_final = alpha;
                        break;
                    }
                    // Constraint-progress fallback: an aggressive reduction
                    // in violation buys us the step even if merit didn't
                    // pass Armijo, as long as the objective didn't regress
                    // more than the violation we eliminated.
                    if (pe_new.max_violation < 0.5 * viol0 &&
                        pe_new.objective_f < pe.objective_f + 2.0 * viol0) {
                        accepted = true;
                        alpha_final = alpha;
                        break;
                    }
                    // Stall-escape acceptance: when we replaced dz with a
                    // pure-objective step, accept any meaningful objective
                    // decrease that keeps violation under a loose cap. This
                    // lets SQP traverse curved manifolds where merit is
                    // overweighted by stiff penalty terms.
                    if (augment_active &&
                        pe_new.objective_f < pe.objective_f - 0.01 &&
                        pe_new.max_violation < viol_cap) {
                        accepted = true;
                        alpha_final = alpha;
                        break;
                    }
                }
                alpha *= 0.5;
                if (alpha < 1.0e-4) {
                    break;
                }
            }
            if (!accepted && (rho_eq < rho_max || rho_ineq < rho_max)) {
                rho_eq = std::min(rho_max, rho_eq * 10.0);
                rho_ineq = std::min(rho_max, rho_ineq * 10.0);
                continue;
            }
            break;
        }

        if (!accepted) {
            ++line_search_failures;
            // Feasibility-restoration phase: ignore objective and pull
            // straight toward c = 0 along a minimum-norm step that hits
            // the linearized constraint manifold. Solve
            //   min ||dz_fr||^2 s.t. J_active * dz_fr + c_active = 0
            // via dz_fr = -J^T * (J*J^T + reg*I)^-1 * c_active.
            // Accept if viol strictly decreases.
            if (!c_active.empty() && pe.max_violation > tol) {
                const std::size_t m = c_active.size();
                std::vector<std::vector<double>> JJT(m,
                    std::vector<double>(m, 0.0));
                for (std::size_t i = 0; i < m; ++i) {
                    for (std::size_t k = 0; k < m; ++k) {
                        double sum = 0.0;
                        for (std::size_t j = 0; j < n; ++j) {
                            sum += J_active[i][j] * J_active[k][j];
                        }
                        JJT[i][k] = sum;
                    }
                    JJT[i][i] += 1.0e-4;
                }
                std::vector<double> rhs(m, 0.0);
                for (std::size_t i = 0; i < m; ++i) {
                    rhs[i] = -c_active[i];
                }
                std::vector<double> y;
                if (solve_linear_system(JJT, rhs, &y)) {
                    std::vector<double> dz_fr(n, 0.0);
                    for (std::size_t j = 0; j < n; ++j) {
                        double sum = 0.0;
                        for (std::size_t i = 0; i < m; ++i) {
                            sum += J_active[i][j] * y[i];
                        }
                        dz_fr[j] = sum;
                    }
                    for (std::size_t i = 0; i < n; ++i) {
                        if (z[i] >= 1.0 - 1.0e-12 && dz_fr[i] > 0.0) dz_fr[i] = 0.0;
                        if (z[i] <= -1.0 + 1.0e-12 && dz_fr[i] < 0.0) dz_fr[i] = 0.0;
                    }
                    double alpha_fr_bound = 1.0;
                    for (std::size_t i = 0; i < n; ++i) {
                        if (dz_fr[i] > 1.0e-18) {
                            alpha_fr_bound = std::min(alpha_fr_bound,
                                (1.0 - z[i]) / dz_fr[i]);
                        } else if (dz_fr[i] < -1.0e-18) {
                            alpha_fr_bound = std::min(alpha_fr_bound,
                                (-1.0 - z[i]) / dz_fr[i]);
                        }
                    }
                    alpha_fr_bound = std::max(0.0, alpha_fr_bound);
                    double alpha_fr = std::min(1.0, alpha_fr_bound);
                    PointEval pe_fr;
                    std::vector<double> z_fr(n, 0.0);
                    bool fr_accepted = false;
                    for (int ls = 0; ls < 16; ++ls) {
                        for (std::size_t i = 0; i < n; ++i) {
                            z_fr[i] = clamp(z[i] + alpha_fr * dz_fr[i], -1.0, 1.0);
                        }
                        pe_fr = evaluate_at(z_fr, vars, base, cs, service,
                            sim_count, messages);
                        // Accept any genuine viol decrease, however small.
                        // At tiny alpha the linearized prediction matches
                        // nonlinear well, so a strict-less-than accept is
                        // safe and lets the algorithm grind down stiff
                        // constraint residuals one tiny correction at a time.
                        if (pe_fr.ok && pe_fr.max_violation <
                                pe.max_violation - 1.0e-9) {
                            fr_accepted = true;
                            break;
                        }
                        alpha_fr *= 0.5;
                        if (alpha_fr < 1.0e-4) {
                            break;
                        }
                    }
                    if (fr_accepted) {
                        z_prev = z;
                        z = z_fr;
                        pe = pe_fr;
                        consider(pe, z);
                        have_prev = false;
                        continue;
                    }
                }
            }
            // Forced merit-penalty steepest-descent step. Both LS and FR
            // failed; rather than exiting, take a small fixed-magnitude
            // step in the direction that reduces the L1 merit penalty.
            // This breaks stalls where rho is over-weighted and the LS /
            // FR convergence tests are conservative.
            std::vector<double> dz_sd(n, 0.0);
            for (std::size_t i = 0; i < n; ++i) {
                double g = jac.grad_f[i];
                for (std::size_t k = 0; k < cs.eq.size(); ++k) {
                    const double s = pe.c_eq[k] >= 0.0 ? 1.0 : -1.0;
                    g += rho_eq * s * jac.J_eq[k][i];
                }
                for (std::size_t k = 0; k < cs.ineq.size(); ++k) {
                    if (pe.c_ineq[k] > 0.0) {
                        g += rho_ineq * jac.J_ineq[k][i];
                    }
                }
                dz_sd[i] = -g;
            }
            for (std::size_t i = 0; i < n; ++i) {
                if (z[i] >= 1.0 - 1.0e-12 && dz_sd[i] > 0.0) dz_sd[i] = 0.0;
                if (z[i] <= -1.0 + 1.0e-12 && dz_sd[i] < 0.0) dz_sd[i] = 0.0;
            }
            double dz_sd_inf = 0.0;
            for (double v : dz_sd) {
                dz_sd_inf = std::max(dz_sd_inf, std::abs(v));
            }
            if (dz_sd_inf < 1.0e-12) {
                append_limited_message(messages,
                    "sqp: line search failed - exiting (no descent)");
                break;
            }
            const double sd_step = 0.03;
            for (std::size_t i = 0; i < n; ++i) {
                dz_sd[i] *= sd_step / dz_sd_inf;
            }
            std::vector<double> z_sd(n, 0.0);
            for (std::size_t i = 0; i < n; ++i) {
                z_sd[i] = clamp(z[i] + dz_sd[i], -1.0, 1.0);
            }
            PointEval pe_sd = evaluate_at(z_sd, vars, base, cs, service,
                sim_count, messages);
            if (!pe_sd.ok) {
                append_limited_message(messages,
                    "sqp: line search failed - exiting (sd sim failed)");
                break;
            }
            // Reset B to keep the Hessian approximation honest after the
            // forced (non-Newton-like) step.
            for (std::size_t i = 0; i < n; ++i) {
                for (std::size_t k = 0; k < n; ++k) {
                    B[i][k] = (i == k) ? 1.0 : 0.0;
                }
            }
            have_prev = false;
            z_prev = z;
            z = z_sd;
            pe = pe_sd;
            consider(pe, z);
            continue;
        }

        // Save the previous Lagrangian gradient for the next BFGS update.
        std::vector<double> grad_L_at_z(n, 0.0);
        for (std::size_t i = 0; i < n; ++i) {
            grad_L_at_z[i] = jac.grad_f[i];
            for (std::size_t k = 0; k < cs.eq.size(); ++k) {
                grad_L_at_z[i] += lambda_eq[k] * jac.J_eq[k][i];
            }
            for (std::size_t k = 0; k < cs.ineq.size(); ++k) {
                grad_L_at_z[i] += lambda_ineq[k] * jac.J_ineq[k][i];
            }
        }
        z_prev = z;
        grad_L_prev = grad_L_at_z;
        jac_prev = jac;
        have_prev = true;

        z = z_new;
        pe = pe_new;
        consider(pe, z);
        (void)alpha_final;
    }

    if (out.found_feasible) {
        out.report_pe = best_feasible_pe;
        out.report_z = best_feasible_z;
    } else if (have_least_infeasible) {
        out.report_pe = least_infeasible_pe;
        out.report_z = least_infeasible_z;
    } else {
        out.report_pe = pe;
        out.report_z = z;
    }

    {
        std::ostringstream m;
        m << "sqp: iters=" << out.iterations
          << " active_eq=" << active_eq_count
          << " active_ineq=" << active_ineq_count
          << " kkt_singular=" << kkt_singular_events
          << " line_search_fails=" << line_search_failures
          << " final_violation=" << out.report_pe.max_violation
          << " feasible=" << (out.found_feasible ? "true" : "false");
        append_limited_message(messages, m.str());
    }
    return out;
}

// ---- Augmented Lagrangian outer loop ------------------------------------

bool validate_optimization_config(const OptimizationConfig& optimization, std::string* error)
{
    if (optimization.optimizer != "fmincon" && optimization.optimizer != "sqp") {
        return fail(error, "unsupported optimizer: " + optimization.optimizer);
    }
    if (optimization.mode != "target" && optimization.mode != "optimize") {
        return fail(error, "unsupported optimization mode: " + optimization.mode);
    }
    if (optimization.targets.empty() && !optimization.objective.enabled) {
        return fail(error, "optimization requires at least one target or an enabled objective");
    }
    if (optimization.mode == "target" && optimization.targets.empty()) {
        return fail(error, "target mode requires at least one target");
    }
    return true;
}

} // namespace

std::vector<OptimizationMetricValue> evaluate_trajectory_metrics(
    const StateLog& state_log,
    const CaseConfig& config)
{
    if (state_log.empty()) {
        return {
            {"terminal_altitude_m", std::numeric_limits<double>::quiet_NaN()},
            {"terminal_speed_mps", std::numeric_limits<double>::quiet_NaN()},
            {"inclination_deg", std::numeric_limits<double>::quiet_NaN()},
            {"periapsis_altitude_m", std::numeric_limits<double>::quiet_NaN()},
            {"payload_mass_kg", post2::vehicle::payload_stage_dry_mass_kg(config.vehicle)},
        };
    }

    const auto& last = state_log.back();
    const Vec3& r = last.state.position_m;
    const Vec3& v = last.state.velocity_mps;
    const Vec3 h{
        r.y * v.z - r.z * v.y,
        r.z * v.x - r.x * v.z,
        r.x * v.y - r.y * v.x,
    };
    const double h_norm = post2::vehicle::norm(h);
    double inclination_deg = 0.0;
    if (h_norm > 0.0) {
        const double cosine = std::clamp(h.z / h_norm, -1.0, 1.0);
        inclination_deg = std::acos(cosine) * 180.0 / kPi;
    }

    const double r_norm = post2::vehicle::norm(r);
    const double v_norm = post2::vehicle::norm(v);
    const double rv = post2::vehicle::dot(r, v);
    double periapsis_altitude_m = std::numeric_limits<double>::quiet_NaN();
    if (r_norm > 0.0 && h_norm > 0.0 && config.earth_mu_m3s2 > 0.0) {
        const Vec3 eccentricity_vector =
            ((v_norm * v_norm - config.earth_mu_m3s2 / r_norm) * r -
                rv * v) /
            config.earth_mu_m3s2;
        const double eccentricity = post2::vehicle::norm(eccentricity_vector);
        const double p = h_norm * h_norm / config.earth_mu_m3s2;
        if (1.0 + eccentricity > 0.0) {
            periapsis_altitude_m = p / (1.0 + eccentricity) - config.earth_radius_m;
        }
    }

    return {
        {"terminal_altitude_m", last.altitude_m},
        {"terminal_speed_mps", last.speed_mps},
        {"inclination_deg", inclination_deg},
        {"periapsis_altitude_m", periapsis_altitude_m},
        {"payload_mass_kg", post2::vehicle::payload_stage_dry_mass_kg(config.vehicle)},
    };
}

bool read_optimization_variable(
    const CaseConfig& config,
    const std::string& path,
    double* value,
    std::string* error)
{
    CaseConfig copy = config;
    ResolvedPath resolved;
    if (!resolve_path(&copy, path, &resolved, error)) {
        return false;
    }
    *value = *resolved.value;
    return true;
}

bool write_optimization_variable(
    CaseConfig* config,
    const std::string& path,
    double value,
    std::string* error)
{
    ResolvedPath resolved;
    if (!resolve_path(config, path, &resolved, error)) {
        return false;
    }
    *resolved.value = value;
    return true;
}

OptimizationResult optimize_case(
    CaseConfig* config,
    ITrajectoryService& service,
    OptimizationRunOptions options)
{
    OptimizationResult result;
    if (!config) {
        result.error = "case config is null";
        return result;
    }

    OptimizationConfig optimization = config->optimization;
    if (options.max_iterations > 0) {
        optimization.max_iterations = options.max_iterations;
    }
    if (options.tolerance > 0.0) {
        optimization.tolerance = options.tolerance;
    }
    if (options.initial_step_fraction > 0.0) {
        optimization.initial_step_fraction = options.initial_step_fraction;
    }
    config->optimization = optimization;

    if (!validate_optimization_config(optimization, &result.error)) {
        return result;
    }

    CaseConfig working = *config;

    // Build VariableSpec list (only enabled variables in optimize_enabled phases).
    // The optimizer operates entirely on scaled z in [-1, +1] per variable;
    // real x is only reconstructed when writing variables before each sim.
    std::vector<VariableSpec> vars;
    std::vector<double> z;          // scaled state
    std::vector<double> x0;         // original (real) values, for variable_changes report
    std::set<int> used_phases;
    std::set<std::string> used_paths;

    for (const auto& variable : optimization.variables) {
        if (!variable.enabled) {
            continue;
        }
        if (variable.min_value > variable.max_value) {
            result.error = "variable min is greater than max: " + variable.path;
            return result;
        }
        ResolvedPath resolved;
        if (!resolve_path(&working, variable.path, &resolved, &result.error)) {
            return result;
        }
        if (resolved.phase_index >= 0 &&
            static_cast<std::size_t>(resolved.phase_index) < working.phases.size() &&
            !working.phases[static_cast<std::size_t>(resolved.phase_index)].optimize_enabled) {
            continue;
        }
        const double old_value = *resolved.value;
        const double clamped = clamp(old_value, variable.min_value, variable.max_value);
        *resolved.value = clamped;
        vars.push_back({variable.path, old_value, variable.min_value, variable.max_value,
            resolved.phase_index});
        z.push_back(to_z_scalar(clamped, variable.min_value, variable.max_value));
        x0.push_back(old_value);
        if (resolved.phase_index >= 0) {
            used_phases.insert(resolved.phase_index);
        }
        used_paths.insert(variable.path);
    }

    if (vars.empty()) {
        result.error = "optimization has no enabled variables in enabled phases";
        return result;
    }

    // Hard constraints from targets.
    const ConstraintSet cs = build_constraints(optimization);

    // Budget: max_iterations -> total BFGS iterations across all AL outer
    // cycles. Each BFGS iteration runs one finite-difference gradient
    // (N+1 simulations) and one projected line search (1-20 simulations).
    const int total_iteration_budget = std::max(1, optimization.max_iterations);
    const double tol = optimization.tolerance > 0.0 ? optimization.tolerance : 1.0e-4;
    const double fd_step_fraction = optimization.initial_step_fraction > 0.0
        ? std::min(0.1, optimization.initial_step_fraction * 0.25)
        : 0.01;

    // Detect whether the payload-stage dry mass is one of our variables.
    // SQP keeps payload as an ordinary NLP decision variable. The index is
    // only used by the fmincon fallback below, where payload continuation
    // remains available for that older optimizer.
    const bool maximize_payload =
        optimization.mode == "optimize" &&
        optimization.objective.enabled &&
        optimization.objective.metric == "payload_mass_kg" &&
        optimization.objective.direction == "maximize";
    int payload_var_index = -1;
    if (maximize_payload && !working.vehicle.stages.empty()) {
        std::size_t payload_stage_index = working.vehicle.stages.size() - 1;
        for (std::size_t i = 0; i < working.vehicle.stages.size(); ++i) {
            if (contains_payload_name(working.vehicle.stages[i].name)) {
                payload_stage_index = i;
                break;
            }
        }
        for (std::size_t i = 0; i < vars.size(); ++i) {
            std::size_t stage_index = 0;
            if (parse_vehicle_stage_dry_mass_path(vars[i].path, &stage_index) &&
                stage_index == payload_stage_index) {
                payload_var_index = static_cast<int>(i);
                break;
            }
        }
    }

    LocalOptimResult best_run;
    bool have_report = false;
    auto consider_run = [&](LocalOptimResult run) {
        if (!run.report_pe.ok || run.report_z.empty()) {
            return;
        }
        const bool better =
            !have_report ||
            (run.found_feasible && !best_run.found_feasible) ||
            (run.found_feasible == best_run.found_feasible &&
                ((run.found_feasible && run.report_pe.objective_f < best_run.report_pe.objective_f) ||
                 (!run.found_feasible && run.report_pe.max_violation < best_run.report_pe.max_violation)));
        if (better) {
            best_run = std::move(run);
            have_report = true;
        }
    };

    const bool use_sqp = optimization.optimizer == "sqp";

    if (use_sqp) {
        // ---- SQP NLP path -------------------------------------------------
        // Solve the full NLP directly. Payload (if present) is a normal
        // decision variable rather than the homotopy parameter of fmincon's
        // continuation; the SQP merit phase handles equality / inequality
        // targets simultaneously with the bound constraints.
        append_limited_message(&result.messages, "sqp: single start");
        LocalOptimResult run = run_sqp_nlp(z, vars, working, cs,
            optimization, total_iteration_budget, tol, fd_step_fraction,
            service, &result.evaluations, &result.messages);
        result.iterations += run.iterations;
        if (!run.error.empty()) {
            append_limited_message(&result.messages, run.error);
        }
        consider_run(std::move(run));
    } else if (payload_var_index >= 0 && vars.size() > 1) {
        // ---- Continuation sweep on payload --------------------------------
        const VariableSpec& payload_var = vars[static_cast<std::size_t>(payload_var_index)];

        // Build inner vars by stripping the payload variable.
        std::vector<VariableSpec> inner_vars;
        std::vector<double> inner_z;
        inner_vars.reserve(vars.size() - 1);
        inner_z.reserve(vars.size() - 1);
        for (std::size_t i = 0; i < vars.size(); ++i) {
            if (static_cast<int>(i) == payload_var_index) {
                continue;
            }
            inner_vars.push_back(vars[i]);
            inner_z.push_back(z[i]);
        }

        // Sweep schedule. Reserve some budget for the upward sweep and
        // bisection refinement; the initial feasibility search may also
        // need a couple of attempts when starting from x0.
        //
        // Each inner solve must run long enough that AL+BFGS can drive a
        // ~N-variable constraint problem to feasibility from a warm-start.
        // The total budget caps the number of attempts; per-step budget
        // is sized for convergence quality, not divided evenly across
        // attempts (because the warm-started later steps converge much
        // faster than the first cold step).
        constexpr int sweep_steps = 8;
        constexpr int bisect_iters = 6;
        const int per_step_iters = std::max(40, total_iteration_budget / 4);
        const double payload_range = payload_var.ub - payload_var.lb;

        // Interleave inner z back with payload's z coordinate to produce a
        // full-vars vector matching the original vars order.
        auto build_full_z = [&](const std::vector<double>& inner_z_solved,
                                double payload_x) -> std::vector<double> {
            std::vector<double> full(vars.size());
            std::size_t k = 0;
            for (std::size_t i = 0; i < vars.size(); ++i) {
                if (static_cast<int>(i) == payload_var_index) {
                    full[i] = to_z_scalar(payload_x, vars[i].lb, vars[i].ub);
                } else {
                    full[i] = inner_z_solved[k++];
                }
            }
            return full;
        };

        // Helper: run an inner AL+BFGS solve with payload pinned to x_payload.
        // sweep_working is reused (mutated only at the payload path) so the
        // rest of the case config stays stable across steps.
        CaseConfig sweep_working = working;
        auto solve_at_payload = [&](double x_payload,
                                    const std::vector<double>& warm_inner_z) -> LocalOptimResult {
            write_optimization_variable(&sweep_working, payload_var.path, x_payload, nullptr);
            LocalOptimResult r = run_local_augmented_lagrangian(
                warm_inner_z, inner_vars, sweep_working, cs,
                optimization, per_step_iters, tol, fd_step_fraction,
                service, &result.evaluations, &result.messages);
            result.iterations += r.iterations;
            if (!r.error.empty()) {
                append_limited_message(&result.messages, r.error);
            }
            return r;
        };

        auto run_continuation_from = [&](double payload_start,
                                         const std::vector<double>& start_inner_z) {
            // Step 1: find a feasible starting payload. Try the requested
            // start first; if it does not yield feasibility, sweep down
            // toward the lower bound.
            double payload_cur = clamp(payload_start, payload_var.lb, payload_var.ub);

            std::ostringstream msg;
            msg << "continuation: trying start payload " << payload_cur << " kg";
            append_limited_message(&result.messages, msg.str());

            LocalOptimResult init = solve_at_payload(payload_cur, start_inner_z);
            bool have_feasible_start = init.found_feasible;
            std::vector<double> inner_z_best = init.report_z.empty() ? start_inner_z : init.report_z;
            PointEval pe_best = init.report_pe;
            double payload_best = payload_cur;

            if (!have_feasible_start) {
                // Scan downward in (payload_cur - lb) / sweep_steps increments.
                for (int i = 1; i <= sweep_steps && !have_feasible_start; ++i) {
                    const double down = payload_cur -
                        static_cast<double>(i) * (payload_cur - payload_var.lb)
                            / static_cast<double>(sweep_steps);
                    const double try_payload = std::max(payload_var.lb, down);
                    LocalOptimResult r = solve_at_payload(try_payload, inner_z_best);
                    if (r.found_feasible) {
                        payload_best = try_payload;
                        inner_z_best = r.report_z;
                        pe_best = r.report_pe;
                        have_feasible_start = true;
                        std::ostringstream found_msg;
                        found_msg << "continuation: feasibility found at payload = "
                            << try_payload << " kg";
                        append_limited_message(&result.messages, found_msg.str());
                    }
                }
            } else {
                std::ostringstream feasible_msg;
                feasible_msg << "continuation: start payload " << payload_cur << " kg is feasible";
                append_limited_message(&result.messages, feasible_msg.str());
            }

            if (have_feasible_start) {
                // Adaptive-step continuation. Start with range/sweep_steps. Grow
                // the step (x1.5) after each feasible advance to accelerate
                // climbing; shrink it (x0.5) after each infeasible attempt to
                // hone in on the boundary. This combines the upward sweep and
                // the bisection refinement into a single self-tuning loop, so
                // it doesn't matter whether feasibility breaks on the 1st or
                // 8th sweep step.
                const double step_floor = std::max(1.0, payload_range * 0.001);  // 0.1% of range
                double step = std::max(step_floor,
                    payload_range / static_cast<double>(sweep_steps));
                const int max_attempts = sweep_steps + bisect_iters + 8;
                // Total iteration cap (soft): stop the continuation loop once
                // the cumulative inner BFGS iterations exceed ~3x the user's
                // requested max_iterations. Without this, an adaptive loop
                // with per_step = max(40, budget/4) could run for many more
                // total iters than the user expected.
                const int iter_cap = total_iteration_budget * 3;
                const int start_iterations = result.iterations;
                for (int attempt = 0; attempt < max_attempts && step > step_floor; ++attempt) {
                    if (payload_best >= payload_var.ub - step_floor) {
                        break;
                    }
                    if (result.iterations - start_iterations > iter_cap) {
                        append_limited_message(&result.messages,
                            "continuation: iteration budget exhausted, stopping sweep");
                        break;
                    }
                    const double try_payload = std::min(payload_var.ub, payload_best + step);
                    if (try_payload <= payload_best + step_floor * 0.25) {
                        break;
                    }
                    LocalOptimResult r = solve_at_payload(try_payload, inner_z_best);
                    if (r.found_feasible) {
                        payload_best = try_payload;
                        inner_z_best = r.report_z;
                        pe_best = r.report_pe;
                        std::ostringstream feasible_msg;
                        feasible_msg << "continuation: payload " << try_payload
                            << " kg feasible (step grew to " << (step * 1.5) << ")";
                        append_limited_message(&result.messages, feasible_msg.str());
                        step = std::min(step * 1.5, payload_var.ub - payload_best);
                    } else {
                        std::ostringstream infeasible_msg;
                        infeasible_msg << "continuation: payload " << try_payload
                            << " kg infeasible (step halved to " << (step * 0.5) << ")";
                        append_limited_message(&result.messages, infeasible_msg.str());
                        step *= 0.5;
                    }
                }
                {
                    std::ostringstream settled_msg;
                    settled_msg << "continuation: settled at payload = " << payload_best << " kg";
                    append_limited_message(&result.messages, settled_msg.str());
                }

                LocalOptimResult continuation_run;
                continuation_run.found_feasible = true;
                continuation_run.report_pe = pe_best;
                continuation_run.report_z = build_full_z(inner_z_best, payload_best);
                continuation_run.iterations = result.iterations;
                consider_run(std::move(continuation_run));
            } else {
                // No feasible payload found anywhere in the swept range; carry
                // the least-violation result we have.
                LocalOptimResult continuation_run;
                continuation_run.found_feasible = false;
                continuation_run.report_pe = init.report_pe;
                continuation_run.report_z = build_full_z(inner_z_best, payload_cur);
                continuation_run.iterations = result.iterations;
                consider_run(std::move(continuation_run));
            }
        };

        std::vector<double> continuation_starts;
        auto add_continuation_start = [&](double x_start) {
            x_start = clamp(x_start, payload_var.lb, payload_var.ub);
            const double duplicate_tolerance = std::max(1.0, payload_range * 1.0e-4);
            for (const double existing : continuation_starts) {
                if (std::abs(existing - x_start) <= duplicate_tolerance) {
                    return;
                }
            }
            continuation_starts.push_back(x_start);
        };

        const double payload_cur = clamp(x0[static_cast<std::size_t>(payload_var_index)],
            payload_var.lb, payload_var.ub);
        add_continuation_start(payload_cur);
        if (payload_range > 1.0) {
            // A high-payload continuation can land on a better steering branch
            // than a sweep that starts from the original low payload.
            add_continuation_start(payload_var.lb + 0.75 * payload_range);
        }

        for (const double start_payload : continuation_starts) {
            run_continuation_from(start_payload, inner_z);
        }

        // Direct multistart remains useful as a fallback: continuation can
        // follow a conservative local branch, while a high-payload direct
        // seed may converge to a better feasible basin.
        std::vector<std::vector<double>> direct_starts;
        auto add_direct_start = [&](std::vector<double> candidate) {
            for (const auto& existing : direct_starts) {
                double max_diff = 0.0;
                for (std::size_t i = 0; i < candidate.size(); ++i) {
                    max_diff = std::max(max_diff, std::abs(candidate[i] - existing[i]));
                }
                if (max_diff < 1.0e-9) {
                    return;
                }
            }
            direct_starts.push_back(std::move(candidate));
        };
        auto add_payload_direct_seed = [&](double x_seed) {
            std::vector<double> seeded = z;
            x_seed = clamp(x_seed, payload_var.lb, payload_var.ub);
            seeded[static_cast<std::size_t>(payload_var_index)] =
                to_z_scalar(x_seed, payload_var.lb, payload_var.ub);
            add_direct_start(std::move(seeded));
        };

        add_direct_start(z);
        add_payload_direct_seed(payload_var.lb + 0.65 * payload_range);
        add_payload_direct_seed(payload_var.lb + 0.75 * payload_range);
        if (payload_range > 1000.0) {
            add_payload_direct_seed(payload_var.ub - 1000.0);
        }
        add_payload_direct_seed(payload_var.ub);
        {
            std::ostringstream msg;
            msg << "direct payload multistart: " << direct_starts.size() << " starts";
            append_limited_message(&result.messages, msg.str());
        }
        for (const auto& start : direct_starts) {
            LocalOptimResult run = run_local_augmented_lagrangian(start, vars, working, cs,
                optimization, total_iteration_budget, tol, fd_step_fraction,
                service, &result.evaluations, &result.messages);
            result.iterations += run.iterations;
            if (!run.error.empty()) {
                append_limited_message(&result.messages, run.error);
            }
            consider_run(std::move(run));
        }
    } else {
        // ---- No payload continuation: single-start direct fmincon ---------
        LocalOptimResult run = run_local_augmented_lagrangian(z, vars, working, cs,
            optimization, total_iteration_budget, tol, fd_step_fraction,
            service, &result.evaluations, &result.messages);
        result.iterations += run.iterations;
        if (!run.error.empty()) {
            append_limited_message(&result.messages, run.error);
        }
        consider_run(std::move(run));
    }

    if (!have_report) {
        result.error = "optimization failed before producing a valid candidate";
        return result;
    }

    const PointEval& report_pe = best_run.report_pe;
    const std::vector<double>& report_z = best_run.report_z;
    const bool found_feasible = best_run.found_feasible;

    // Write back the reported variable values (converted from z to x).
    for (std::size_t i = 0; i < vars.size(); ++i) {
        const double x_i = to_x_scalar(report_z[i], vars[i].lb, vars[i].ub);
        write_optimization_variable(&working, vars[i].path, x_i, nullptr);
    }

    *config = working;
    config->optimization = optimization;
    for (auto& variable : config->optimization.variables) {
        if (used_paths.count(variable.path) > 0) {
            variable.enabled = false;
        }
    }
    for (const int phase_index : used_phases) {
        if (phase_index >= 0 && static_cast<std::size_t>(phase_index) < config->phases.size()) {
            config->phases[static_cast<std::size_t>(phase_index)].optimize_enabled = false;
        }
    }

    for (std::size_t i = 0; i < vars.size(); ++i) {
        const double x_i = to_x_scalar(report_z[i], vars[i].lb, vars[i].ub);
        result.variable_changes.push_back({vars[i].path, x0[i], x_i});
    }

    // best_score reflects objective + max constraint violation.
    result.best_score = report_pe.objective_f + report_pe.max_violation;

    if (options.run_final_simulation) {
        result.final_simulation = service.simulate(*config);
        ++result.evaluations;
        if (!result.final_simulation.ok) {
            result.error = "final simulation failed: " + result.final_simulation.error;
            return result;
        }
        result.final_metrics = evaluate_trajectory_metrics(result.final_simulation.state_log, *config);
    } else {
        result.final_simulation = report_pe.simulation;
        result.final_metrics = report_pe.metrics;
    }

    auto add_requested_metric = [&](const std::string& metric) {
        const auto existing = std::find_if(
            result.final_metrics.begin(),
            result.final_metrics.end(),
            [&](const OptimizationMetricValue& candidate) {
                return candidate.metric == metric;
            });
        if (existing != result.final_metrics.end()) {
            return;
        }
        double value = 0.0;
        if (!trajectory_metric_value(result.final_simulation.state_log, *config, metric, &value)) {
            return;
        }
        const auto payload_it = std::find_if(
            result.final_metrics.begin(),
            result.final_metrics.end(),
            [](const OptimizationMetricValue& candidate) {
                return candidate.metric == "payload_mass_kg";
            });
        result.final_metrics.insert(payload_it, {metric, value});
    };
    for (const auto& target : optimization.targets) {
        add_requested_metric(target.metric);
    }
    if (optimization.objective.enabled) {
        add_requested_metric(optimization.objective.metric);
    }

    // Strict feasibility: ok=true requires a feasible point found.
    if (!found_feasible) {
        std::ostringstream msg;
        msg << "infeasible: best max_violation = " << report_pe.max_violation
            << " > tolerance = " << tol;
        result.error = msg.str();
        append_limited_message(&result.messages, result.error);
        result.ok = false;
        return result;
    }
    result.ok = true;
    return result;
}

} // namespace post2::core
