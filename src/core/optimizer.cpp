#include "post2/core/optimization.hpp"

#include "post2/core/nlp_evaluator.hpp"
#include "post2/core/optimizer.hpp"
#include "post2/core/qp_solver.hpp"
#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string_view>
#include <utility>

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
        std::size_t tank_index = 0;
        if (consume_indexed(&working, "tanks", &tank_index) && consume_dot(&working)) {
            if (tank_index >= config->vehicle.tanks.size()) {
                return fail(error, "vehicle tank index out of range in variable path: " + path);
            }
            if (working == "capacity_kg") {
                target->value = &config->vehicle.tanks[tank_index].capacity_kg;
                return true;
            }
            if (working == "initial_kg") {
                target->value = &config->vehicle.tanks[tank_index].initial_kg;
                return true;
            }
            return fail(error, "unsupported vehicle tank variable path: " + path);
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
            if (consume_indexed(&working, "tanks", &tank_index) && consume_dot(&working)) {
                auto& tanks = config->vehicle.stages[stage_index].tanks;
                if (tank_index >= tanks.size()) {
                    return fail(error, "vehicle stage tank index out of range in variable path: " + path);
                }
                if (working == "capacity_kg") {
                    target->value = &tanks[tank_index].capacity_kg;
                    return true;
                }
                if (working == "initial_kg") {
                    target->value = &tanks[tank_index].initial_kg;
                    return true;
                }
                return fail(error, "unsupported vehicle stage tank variable path: " + path);
            }
            return fail(error, "unsupported vehicle stage variable path: " + path);
        }
        return fail(error, "unsupported vehicle variable path: " + path);
    }

    working = text;
    if (consume_identifier(&working, "launch_site") && consume_dot(&working)) {
        if (working == "latitude_deg") {
            target->value = &config->launch_site.latitude_deg;
            return true;
        }
        if (working == "longitude_deg") {
            target->value = &config->launch_site.longitude_deg;
            return true;
        }
        if (working == "altitude_m") {
            target->value = &config->launch_site.altitude_m;
            return true;
        }
        return fail(error, "unsupported launch_site variable path: " + path);
    }

    std::size_t phase_index = 0;
    if (!consume_indexed(&text, "phases", &phase_index) || !consume_dot(&text)) {
        return fail(error, "variable path must start with phases[index], vehicle, or launch_site.");
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

ConstraintSet constraint_set_from_problem(const NlpProblem& problem)
{
    ConstraintSet cs;
    for (const auto& constraint : problem.constraints) {
        if (constraint.weight <= 0.0) {
            continue;
        }
        switch (constraint.type) {
        case NlpConstraintType::Equality:
            cs.eq.push_back({constraint.metric, constraint.target, constraint.scale, constraint.weight});
            break;
        case NlpConstraintType::LowerBound:
        case NlpConstraintType::Path:
            cs.ineq.push_back({constraint.metric, constraint.min_value, constraint.scale,
                constraint.weight, false});
            break;
        case NlpConstraintType::UpperBound:
            cs.ineq.push_back({constraint.metric, constraint.max_value, constraint.scale,
                constraint.weight, true});
            break;
        case NlpConstraintType::Range:
            cs.ineq.push_back({constraint.metric, constraint.min_value, constraint.scale,
                constraint.weight, false});
            cs.ineq.push_back({constraint.metric, constraint.max_value, constraint.scale,
                constraint.weight, true});
            break;
        }
    }
    return cs;
}

// ---- fmincon: point evaluation -----------------------------------------

struct PointEval {
    bool ok = false;
    std::string error;
    SimulationResult simulation;
    std::vector<OptimizationMetricValue> metrics;
    double objective_f = 0.0;            // sign-adjusted objective (minimization)
    std::vector<double> c_eq;            // unweighted residuals
    std::vector<double> c_ineq;          // unweighted residuals (<= 0 feasible)
    double max_violation = 0.0;          // max(|c_eq|, max(0, c_ineq))
    NlpEvalResult nlp_eval;
};

PointEval point_eval_from_nlp_eval(NlpEvalResult eval)
{
    PointEval pe;
    pe.ok = eval.ok;
    pe.error = eval.error;
    pe.simulation = eval.simulation;
    pe.metrics = eval.metrics;
    pe.objective_f = eval.objective;
    pe.c_eq = eval.equality_residuals;
    pe.c_ineq = eval.inequality_residuals;
    pe.max_violation = eval.max_violation;
    pe.nlp_eval = std::move(eval);
    return pe;
}

PointEval evaluate_at(
    const std::vector<double>& z,
    const NlpProblem& problem,
    NlpEvaluator& evaluator,
    std::vector<std::string>* messages)
{
    PointEval pe = point_eval_from_nlp_eval(evaluator.evaluate(problem, z));
    if (!pe.ok && !pe.error.empty()) {
        append_limited_message(messages, pe.error);
    }
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

// AL/BFGS uses the shared scalar FD helper; each probe evaluates through the
// NLP evaluator so variable write-back, metrics, and simulation are centralized.
bool gradient_forward(
    const std::vector<double>& z,                    // in [-1, 1] per variable
    const NlpProblem& problem,
    NlpEvaluator& evaluator,
    const ConstraintSet& cs,
    double mu,
    const std::vector<double>& lambda_eq,
    const std::vector<double>& lambda_ineq,
    const PointEval& pe0,
    double L0,
    const FiniteDifferenceOptions& fd_options,
    std::vector<std::string>* messages,
    std::vector<double>* grad)
{
    FiniteDifferenceOptions local_options = fd_options;
    local_options.step_fraction = std::max(fd_options.step_fraction * 2.0, 1.0e-6);
    local_options.parallel = false;
    auto merit = [&](const std::vector<double>& candidate,
                     double* value,
                     std::string* error) -> bool {
        std::vector<double> bounded = candidate;
        for (auto& zi : bounded) {
            zi = clamp(zi, -1.0, 1.0);
        }
        PointEval pe = evaluate_at(bounded, problem, evaluator, messages);
        if (!pe.ok) {
            if (error) {
                *error = pe.error;
            }
            return false;
        }
        *value = augmented_lagrangian(pe, cs, mu, lambda_eq, lambda_ineq);
        return true;
    };
    FiniteDifferenceResult fd = finite_difference_gradient(z, merit, local_options);
    for (const auto& message : fd.messages) {
        append_limited_message(messages, message);
    }
    if (!fd.ok) {
        append_limited_message(messages, fd.error);
        return false;
    }
    *grad = std::move(fd.gradient);
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
    const NlpProblem& problem,
    NlpEvaluator& evaluator,
    const ConstraintSet& cs,
    double mu,
    const std::vector<double>& lambda_eq,
    const std::vector<double>& lambda_ineq,
    double L0,
    const std::vector<double>& g0,
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
        PointEval pe = evaluate_at(z_new, problem, evaluator, messages);
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
    const NlpProblem& problem,
    NlpEvaluator& evaluator,
    const ConstraintSet& cs,
    double mu,
    const std::vector<double>& lambda_eq,
    const std::vector<double>& lambda_ineq,
    int max_iter,
    double tol_grad,
    const FiniteDifferenceOptions& fd_options,
    std::vector<std::string>* messages)
{
    BfgsResult res;
    const std::size_t n = z.size();
    std::vector<std::vector<double>> H(n, std::vector<double>(n, 0.0));
    for (std::size_t i = 0; i < n; ++i) {
        H[i][i] = 1.0;
    }
    PointEval pe = evaluate_at(z, problem, evaluator, messages);
    if (!pe.ok) {
        res.pe = std::move(pe);
        res.z = std::move(z);
        return res;
    }
    double L = augmented_lagrangian(pe, cs, mu, lambda_eq, lambda_ineq);
    std::vector<double> g;
    if (!gradient_forward(z, problem, evaluator, cs, mu, lambda_eq, lambda_ineq, pe, L,
            fd_options, messages, &g)) {
        res.pe = std::move(pe);
        res.L = L;
        res.z = std::move(z);
        return res;
    }

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
        auto ls = line_search(z, d, problem, evaluator, cs, mu, lambda_eq, lambda_ineq,
            L, g, messages);
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
        if (!gradient_forward(z_new, problem, evaluator, cs, mu, lambda_eq, lambda_ineq,
                ls.pe, ls.L, fd_options, messages, &g_new)) {
            break;
        }
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
    const NlpProblem& problem,
    NlpEvaluator& evaluator,
    const ConstraintSet& cs,
    const OptimizerOptions& options,
    int iteration_budget,
    double tol,
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
    PointEval init_pe = evaluate_at(z, problem, evaluator, messages);
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
        BfgsResult inner = bfgs_inner(z, problem, evaluator, cs, mu, lambda_eq, lambda_ineq,
            /*max_iter=*/per_outer_iters,
            /*tol_grad=*/std::max(1.0e-6, tol),
            options.finite_difference,
            messages);
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
// Derivatives are delegated to the shared NLP finite-difference module so
// objective and constraint probes use the same evaluator path as base points.

struct SqpJacobians {
    std::vector<double> grad_f;                       // size n
    std::vector<std::vector<double>> J_eq;            // m_eq x n
    std::vector<std::vector<double>> J_ineq;          // m_ineq x n
    bool ok = false;
};

SqpJacobians compute_fd_jacobians(
    const NlpProblem& problem,
    NlpEvaluator& evaluator,
    const std::vector<double>& z,
    const ConstraintSet& cs,
    const PointEval& pe0,
    const FiniteDifferenceOptions& fd_options,
    std::vector<std::string>* messages)
{
    const std::size_t n = z.size();
    SqpJacobians j;
    j.grad_f.assign(n, 0.0);
    j.J_eq.assign(cs.eq.size(), std::vector<double>(n, 0.0));
    j.J_ineq.assign(cs.ineq.size(), std::vector<double>(n, 0.0));
    NlpDerivativeResult deriv = finite_difference_nlp(
        problem,
        evaluator,
        z,
        pe0.nlp_eval,
        fd_options);
    for (const auto& message : deriv.messages) {
        append_limited_message(messages, message);
    }
    if (!deriv.ok) {
        append_limited_message(messages, deriv.error);
        return j;
    }
    j.grad_f = std::move(deriv.grad_f);
    j.J_eq = std::move(deriv.J_eq);
    j.J_ineq = std::move(deriv.J_ineq);
    j.ok = true;
    return j;
}

// ---- SQP: KKT step ------------------------------------------------------
//
// Solve the local dense QP step:
//   min   grad_f^T dz + 0.5 dz^T (B + reg*I) dz
//   s.t.  J_eq   * dz + c_eq   = 0
//         J_ineq * dz + c_ineq <= 0
//         -1 <= z + dz <= 1
//
// through the dense QP interface. If the QP solve fails, the caller falls
// back to steepest descent on the merit model.

bool solve_qp_step(
    IDenseQpSolver& qp_solver,
    const std::vector<std::vector<double>>& B,
    const std::vector<double>& grad_f,
    const std::vector<std::vector<double>>& J_eq,
    const std::vector<double>& c_eq,
    const std::vector<std::vector<double>>& J_ineq,
    const std::vector<double>& c_ineq,
    const std::vector<double>& z,
    std::vector<double>* dz,
    std::vector<double>* lambda_eq,
    std::vector<double>* lambda_ineq,
    int* active_ineq_count,
    std::vector<std::string>* messages)
{
    QpProblem qp;
    qp.H = B;
    qp.g = grad_f;
    qp.lower.reserve(z.size());
    qp.upper.reserve(z.size());
    for (const double zi : z) {
        qp.lower.push_back(-1.0 - zi);
        qp.upper.push_back(1.0 - zi);
    }
    qp.Aeq = J_eq;
    qp.beq.reserve(c_eq.size());
    for (double c : c_eq) {
        qp.beq.push_back(-c);
    }
    qp.Aineq = J_ineq;
    qp.bineq.reserve(c_ineq.size());
    for (double c : c_ineq) {
        qp.bineq.push_back(-c);
    }
    QpResult qp_result = qp_solver.solve(qp);
    bool active_set_solver = false;
    for (const auto& message : qp_result.messages) {
        if (message == "qp: using active-set") {
            active_set_solver = true;
        }
        append_limited_message(messages, message);
    }
    if (!qp_result.ok || qp_result.step.size() != grad_f.size()) {
        if (active_set_solver) {
            append_limited_message(messages, "qp: active-set failed; using merit fallback");
        } else {
            append_limited_message(messages, "qp: solver failed; using merit fallback");
        }
        if (!qp_result.error.empty()) {
            append_limited_message(messages, qp_result.error);
        }
        return false;
    }
    *dz = std::move(qp_result.step);
    lambda_eq->assign(c_eq.size(), 0.0);
    lambda_ineq->assign(c_ineq.size(), 0.0);
    const std::size_t eq_count = std::min(c_eq.size(), qp_result.multipliers.size());
    for (std::size_t i = 0; i < eq_count; ++i) {
        (*lambda_eq)[i] = qp_result.multipliers[i];
    }
    for (std::size_t i = 0; i < c_ineq.size(); ++i) {
        const std::size_t src = c_eq.size() + i;
        if (src < qp_result.multipliers.size()) {
            (*lambda_ineq)[i] = std::max(0.0, qp_result.multipliers[src]);
        }
    }
    if (active_ineq_count) {
        *active_ineq_count = static_cast<int>(qp_result.active_set.size());
    }
    return true;
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
    const NlpProblem& problem,
    NlpEvaluator& evaluator,
    const ConstraintSet& cs,
    const OptimizerOptions& options,
    IDenseQpSolver& qp_solver,
    int iteration_budget,
    double tol,
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

    PointEval pe = evaluate_at(z, problem, evaluator, messages);
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

        SqpJacobians jac = compute_fd_jacobians(
            problem, evaluator, z, cs, pe, options.finite_difference, messages);
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
            }
        }
        active_eq_count = static_cast<int>(cs.eq.size());
        active_ineq_count = static_cast<int>(c_active.size() - cs.eq.size());

        // Solve KKT, with fallback to steepest descent on the L1 merit.
        std::vector<double> dz;
        std::vector<double> lambda_eq_next;
        std::vector<double> lambda_ineq_next;
        const bool kkt_ok = solve_qp_step(
            qp_solver,
            B,
            jac.grad_f,
            jac.J_eq,
            pe.c_eq,
            jac.J_ineq,
            pe.c_ineq,
            z,
            &dz,
            &lambda_eq_next,
            &lambda_ineq_next,
            &active_ineq_count,
            messages);
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
            lambda_eq_next.assign(cs.eq.size(), 0.0);
            lambda_ineq_next.assign(cs.ineq.size(), 0.0);
        }

        // Update full-size multipliers from this step.
        lambda_eq = std::move(lambda_eq_next);
        lambda_ineq = std::move(lambda_ineq_next);

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
                pe_new = evaluate_at(z_new, problem, evaluator, messages);
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
                        pe_fr = evaluate_at(z_fr, problem, evaluator, messages);
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
            PointEval pe_sd = evaluate_at(z_sd, problem, evaluator, messages);
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
    if (optimization.qp_solver != "kkt-fallback" && optimization.qp_solver != "active-set") {
        return fail(error, "unsupported qp solver: " + optimization.qp_solver);
    }
    if (optimization.fd_mode != "forward" &&
        optimization.fd_mode != "backward" &&
        optimization.fd_mode != "central" &&
        optimization.fd_mode != "auto") {
        return fail(error, "unsupported finite difference mode: " + optimization.fd_mode);
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
    if (optimization.continuation.enabled) {
        if (optimization.continuation.variable_path.empty()) {
            return fail(error, "continuation variable path is required");
        }
        if (optimization.continuation.direction != "increase" &&
            optimization.continuation.direction != "decrease") {
            return fail(error, "unsupported continuation direction: " +
                optimization.continuation.direction);
        }
        if (optimization.continuation.steps <= 0) {
            return fail(error, "continuation steps must be positive");
        }
        if (optimization.continuation.multistart_count <= 0) {
            return fail(error, "continuation multistart count must be positive");
        }
        if (optimization.targets.empty()) {
            return fail(error, "continuation requires at least one target");
        }
    }
    return true;
}

OptimizerOptions optimizer_options_from_config(
    const OptimizationConfig& optimization,
    const OptimizationRunOptions& run_options,
    ITrajectoryService& service)
{
    OptimizerOptions options;
    options.max_iterations = run_options.max_iterations > 0
        ? run_options.max_iterations
        : optimization.max_iterations;
    options.tolerance = run_options.tolerance > 0.0
        ? run_options.tolerance
        : optimization.tolerance;
    options.stationarity_tolerance = optimization.stationarity_tolerance > 0.0
        ? optimization.stationarity_tolerance
        : options.tolerance;
    options.feasibility_tolerance = optimization.feasibility_tolerance > 0.0
        ? optimization.feasibility_tolerance
        : options.tolerance;
    options.constraint_tolerance = optimization.constraint_tolerance > 0.0
        ? optimization.constraint_tolerance
        : options.tolerance;
    options.initial_step_fraction = run_options.initial_step_fraction > 0.0
        ? run_options.initial_step_fraction
        : optimization.initial_step_fraction;
    options.max_restoration_iterations = optimization.max_restoration_iterations;
    options.qp_solver = optimization.qp_solver;
    if (!parse_finite_difference_mode(optimization.fd_mode, &options.finite_difference.mode)) {
        options.finite_difference.mode = FiniteDifferenceMode::Auto;
    }
    options.finite_difference.step_fraction = options.initial_step_fraction > 0.0
        ? std::min(0.1, options.initial_step_fraction * 0.25)
        : 0.01;
    options.finite_difference.parallel =
        optimization.parallel_fd && service.supports_parallel_simulation();
    return options;
}

const OptimizationVariableConfig* find_enabled_variable(
    const OptimizationConfig& optimization,
    const std::string& path)
{
    const auto found = std::find_if(
        optimization.variables.begin(),
        optimization.variables.end(),
        [&](const OptimizationVariableConfig& variable) {
            return variable.path == path && variable.enabled;
        });
    return found == optimization.variables.end() ? nullptr : &(*found);
}

bool has_enabled_variables(const OptimizationConfig& optimization)
{
    return std::any_of(
        optimization.variables.begin(),
        optimization.variables.end(),
        [](const OptimizationVariableConfig& variable) {
            return variable.enabled;
        });
}

OptimizationConfig target_only_continuation_config(
    const OptimizationConfig& optimization,
    const std::string& fixed_variable_path)
{
    OptimizationConfig target_only = optimization;
    target_only.mode = "target";
    target_only.objective.enabled = false;
    target_only.continuation.enabled = false;
    target_only.variables.clear();
    for (const auto& variable : optimization.variables) {
        if (variable.path != fixed_variable_path) {
            target_only.variables.push_back(variable);
        }
    }
    return target_only;
}

void add_requested_metrics(
    OptimizationResult* result,
    const OptimizationConfig& optimization,
    const CaseConfig& config)
{
    if (!result || !result->final_simulation.ok) {
        return;
    }
    auto add_metric = [&](const std::string& metric) {
        const auto existing = std::find_if(
            result->final_metrics.begin(),
            result->final_metrics.end(),
            [&](const OptimizationMetricValue& candidate) {
                return candidate.metric == metric;
            });
        if (existing != result->final_metrics.end()) {
            return;
        }
        double value = 0.0;
        if (!evaluate_trajectory_metric(result->final_simulation.state_log, config, metric, &value)) {
            return;
        }
        const auto payload_it = std::find_if(
            result->final_metrics.begin(),
            result->final_metrics.end(),
            [](const OptimizationMetricValue& candidate) {
                return candidate.metric == "payload_mass_kg";
            });
        result->final_metrics.insert(payload_it, {metric, value});
    };
    for (const auto& target : optimization.targets) {
        add_metric(target.metric);
    }
    if (optimization.objective.enabled) {
        add_metric(optimization.objective.metric);
    }
}

void add_variable_changes(
    const CaseConfig& before,
    const CaseConfig& after,
    const OptimizationConfig& optimization,
    std::vector<OptimizationVariableChange>* changes)
{
    if (!changes) {
        return;
    }
    changes->clear();
    for (const auto& variable : optimization.variables) {
        if (!variable.enabled) {
            continue;
        }
        double old_value = 0.0;
        double new_value = 0.0;
        std::string error;
        if (!read_optimization_variable(before, variable.path, &old_value, &error) ||
            !read_optimization_variable(after, variable.path, &new_value, &error)) {
            continue;
        }
        changes->push_back({variable.path, old_value, new_value});
    }
}

OptimizationResult evaluate_fixed_target_case(
    CaseConfig* config,
    ITrajectoryService& service,
    const OptimizationRunOptions& run_options)
{
    OptimizationResult result;
    if (!config) {
        result.error = "case config is null";
        return result;
    }

    NlpProblem problem;
    if (!build_nlp_problem_from_case(*config, &problem, &result.error)) {
        return result;
    }
    OptimizerOptions optimizer_options =
        optimizer_options_from_config(config->optimization, run_options, service);
    NlpEvaluator evaluator(*config, service);
    const auto eval = evaluator.evaluate(problem, {});
    result.evaluations = evaluator.evaluations();
    result.final_simulation = eval.simulation;
    result.final_metrics = eval.metrics;
    result.best_score = eval.objective + eval.max_violation;
    result.max_constraint_violation = eval.max_violation;
    result.l1_constraint_violation = eval.l1_violation;
    result.found_feasible = eval.ok &&
        eval.max_violation <= optimizer_options.constraint_tolerance;
    result.ok = result.found_feasible;
    append_limited_message(&result.messages,
        "continuation: fixed target evaluation with no free variables");
    if (!eval.ok) {
        result.error = eval.error.empty() ? "fixed target evaluation failed" : eval.error;
    } else if (!result.found_feasible) {
        std::ostringstream msg;
        msg << "infeasible: best max_violation = " << eval.max_violation
            << " > tolerance = " << optimizer_options.constraint_tolerance;
        result.error = msg.str();
        append_limited_message(&result.messages, result.error);
    }
    return result;
}

struct ContinuationCandidate {
    bool valid = false;
    double value = 0.0;
    CaseConfig case_config;
    OptimizationResult result;
};

bool farther_along_continuation(double candidate, double best, bool increasing)
{
    return increasing ? candidate > best : candidate < best;
}

double signed_distance_to_target(double value, double target, bool increasing)
{
    return increasing ? target - value : value - target;
}

OptimizationResult optimize_case_with_continuation(
    CaseConfig* config,
    ITrajectoryService& service,
    const OptimizationConfig& optimization,
    OptimizationRunOptions options)
{
    OptimizationResult result;
    const CaseConfig initial_case = *config;
    const auto& continuation = optimization.continuation;
    append_limited_message(&result.messages, "continuation: enabled");

    const OptimizationVariableConfig* fixed_variable =
        find_enabled_variable(optimization, continuation.variable_path);
    if (!fixed_variable) {
        result.error = "continuation variable must be an enabled optimization variable: " +
            continuation.variable_path;
        return result;
    }
    if (fixed_variable->min_value >= fixed_variable->max_value) {
        result.error = "continuation variable bounds must have min < max: " +
            fixed_variable->path;
        return result;
    }
    if (optimization.targets.empty()) {
        result.error = "continuation requires at least one target";
        return result;
    }

    std::string read_error;
    double current_value = 0.0;
    if (!read_optimization_variable(*config, fixed_variable->path, &current_value, &read_error)) {
        result.error = read_error;
        return result;
    }

    const bool increasing = continuation.direction != "decrease";
    const double lower = fixed_variable->min_value;
    const double upper = fixed_variable->max_value;
    const double range = upper - lower;
    const double start_value = clamp(current_value, lower, upper);
    const double target_value = increasing ? upper : lower;
    const int steps = std::max(1, continuation.steps);
    const int start_count = continuation.multistart_enabled
        ? std::max(1, continuation.multistart_count)
        : 1;

    OptimizationRunOptions local_options = options;
    local_options.run_final_simulation = true;

    auto absorb_result = [&](const OptimizationResult& step_result) {
        result.iterations += step_result.iterations;
        result.evaluations += step_result.evaluations;
        for (const auto& message : step_result.messages) {
            append_limited_message(&result.messages, message);
        }
    };

    ContinuationCandidate best_feasible;
    bool have_feasible = false;
    ContinuationCandidate best_infeasible;
    bool have_infeasible = false;

    auto consider_candidate = [&](const ContinuationCandidate& candidate) {
        if (!candidate.valid) {
            return;
        }
        if (candidate.result.found_feasible) {
            if (!have_feasible ||
                farther_along_continuation(candidate.value, best_feasible.value, increasing) ||
                (std::abs(candidate.value - best_feasible.value) <=
                    std::max(1.0e-9, range * 1.0e-9) &&
                    candidate.result.max_constraint_violation <
                        best_feasible.result.max_constraint_violation)) {
                best_feasible = candidate;
                have_feasible = true;
            }
        } else if (!have_infeasible ||
            candidate.result.max_constraint_violation <
                best_infeasible.result.max_constraint_violation) {
            best_infeasible = candidate;
            have_infeasible = true;
        }
    };

    auto solve_fixed_value = [&](const CaseConfig& base_case,
                                 double value,
                                 const char* label) {
        ContinuationCandidate candidate;
        candidate.value = clamp(value, lower, upper);
        candidate.case_config = base_case;

        std::string write_error;
        if (!write_optimization_variable(
                &candidate.case_config,
                fixed_variable->path,
                candidate.value,
                &write_error)) {
            candidate.result.error = write_error;
            return candidate;
        }
        candidate.case_config.optimization =
            target_only_continuation_config(optimization, fixed_variable->path);

        {
            std::ostringstream msg;
            msg << "continuation: " << label << ' ' << fixed_variable->path
                << " = " << candidate.value;
            append_limited_message(&result.messages, msg.str());
        }

        if (has_enabled_variables(candidate.case_config.optimization)) {
            candidate.result =
                optimize_case(&candidate.case_config, service, local_options);
        } else {
            candidate.result =
                evaluate_fixed_target_case(&candidate.case_config, service, local_options);
        }
        absorb_result(candidate.result);
        candidate.valid = candidate.result.final_simulation.ok ||
            candidate.result.found_feasible ||
            !candidate.result.variable_changes.empty();
        return candidate;
    };

    auto run_sweep_from_seed = [&](double seed_value) {
        seed_value = clamp(seed_value, lower, upper);
        {
            std::ostringstream msg;
            msg << "continuation: trying seed " << seed_value;
            append_limited_message(&result.messages, msg.str());
        }

        ContinuationCandidate seed =
            solve_fixed_value(initial_case, seed_value, "seed");
        consider_candidate(seed);

        if (!seed.result.found_feasible) {
            bool found_retreat = false;
            const double retreat_target = start_value;
            for (int i = 1; i <= steps; ++i) {
                const double alpha = static_cast<double>(i) / static_cast<double>(steps);
                const double try_value = seed_value + (retreat_target - seed_value) * alpha;
                if (std::abs(try_value - seed.value) <= std::max(1.0e-9, range * 1.0e-9)) {
                    continue;
                }
                ContinuationCandidate retreat =
                    solve_fixed_value(initial_case, try_value, "retreat");
                consider_candidate(retreat);
                if (retreat.result.found_feasible) {
                    seed = retreat;
                    found_retreat = true;
                    break;
                }
            }
            if (!found_retreat && !seed.result.found_feasible) {
                append_limited_message(&result.messages,
                    "continuation: seed did not find a feasible target-only trajectory");
                return;
            }
        }

        CaseConfig warm_case = seed.case_config;
        double accepted_value = seed.value;
        double step = std::max(range / static_cast<double>(steps), range * 1.0e-4);
        const double min_step = std::max(1.0e-9, range * 1.0e-4);
        const int max_attempts = steps * 3 + 8;
        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            const double remaining =
                signed_distance_to_target(accepted_value, target_value, increasing);
            if (remaining <= min_step) {
                break;
            }
            const double step_now = std::min(step, remaining);
            const double try_value = increasing
                ? accepted_value + step_now
                : accepted_value - step_now;
            if (std::abs(try_value - accepted_value) <= min_step * 0.25) {
                break;
            }

            ContinuationCandidate trial =
                solve_fixed_value(warm_case, try_value, "step");
            consider_candidate(trial);
            if (trial.result.found_feasible) {
                warm_case = trial.case_config;
                accepted_value = trial.value;
                step = std::min(step * 1.5,
                    std::max(min_step, signed_distance_to_target(
                        accepted_value, target_value, increasing)));
                std::ostringstream msg;
                msg << "continuation: accepted " << fixed_variable->path
                    << " = " << accepted_value;
                append_limited_message(&result.messages, msg.str());
            } else {
                step *= 0.5;
                std::ostringstream msg;
                msg << "continuation: rejected " << fixed_variable->path
                    << " = " << try_value << ", reducing step";
                append_limited_message(&result.messages, msg.str());
                if (step <= min_step) {
                    break;
                }
            }
        }

        std::ostringstream msg;
        msg << "continuation: settled seed at " << fixed_variable->path
            << " = " << accepted_value;
        append_limited_message(&result.messages, msg.str());
    };

    std::vector<double> seeds;
    auto add_seed = [&](double seed) {
        seed = clamp(seed, lower, upper);
        const double duplicate_tolerance = std::max(1.0e-9, range * 1.0e-6);
        for (double existing : seeds) {
            if (std::abs(existing - seed) <= duplicate_tolerance) {
                return;
            }
        }
        seeds.push_back(seed);
    };
    add_seed(start_value);
    if (continuation.multistart_enabled && start_count > 1) {
        for (int i = 1; i < start_count; ++i) {
            const double alpha = static_cast<double>(i) /
                static_cast<double>(start_count - 1);
            add_seed(start_value + (target_value - start_value) * alpha);
        }
    }

    {
        std::ostringstream msg;
        msg << "continuation: " << seeds.size()
            << (continuation.multistart_enabled ? " multistart seeds" : " seed");
        append_limited_message(&result.messages, msg.str());
    }
    for (double seed : seeds) {
        run_sweep_from_seed(seed);
    }

    ContinuationCandidate final_candidate;
    if (have_feasible) {
        final_candidate = best_feasible;
        result.found_feasible = true;
        result.ok = true;
    } else if (have_infeasible) {
        final_candidate = best_infeasible;
        result.found_feasible = false;
        result.ok = false;
        result.error =
            "continuation failed to find a feasible target-only warm start";
    } else {
        result.error = "continuation failed before producing a valid candidate";
        return result;
    }

    *config = final_candidate.case_config;
    config->optimization = optimization;
    result.best_score = final_candidate.result.best_score;
    result.max_constraint_violation =
        final_candidate.result.max_constraint_violation;
    result.l1_constraint_violation =
        final_candidate.result.l1_constraint_violation;

    add_variable_changes(initial_case, *config, optimization, &result.variable_changes);

    if (options.run_final_simulation) {
        result.final_simulation = service.simulate(*config);
        ++result.evaluations;
        if (!result.final_simulation.ok) {
            result.error = "final simulation failed: " + result.final_simulation.error;
            result.ok = false;
            return result;
        }
        result.final_metrics = evaluate_trajectory_metrics(
            result.final_simulation.state_log,
            *config);
    } else {
        result.final_simulation = final_candidate.result.final_simulation;
        result.final_metrics = final_candidate.result.final_metrics;
    }
    add_requested_metrics(&result, optimization, *config);

    if (!result.found_feasible && result.error.empty()) {
        result.error = "continuation: no feasible candidate";
    }
    return result;
}

} // namespace

std::vector<OptimizationMetricValue> evaluate_trajectory_metrics(
    const StateLog& state_log,
    const CaseConfig& config)
{
    return evaluate_extended_trajectory_metrics(state_log, config);
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
    if (optimization.continuation.enabled) {
        return optimize_case_with_continuation(config, service, optimization, options);
    }

    NlpProblem problem;
    if (!build_nlp_problem_from_case(*config, &problem, &result.error)) {
        return result;
    }
    append_limited_message(&result.messages, "nlp: problem built");
    append_limited_message(&result.messages, "evaluator: trajectory service");
    append_limited_message(&result.messages, "optimizer: " + optimization.optimizer);

    OptimizerOptions optimizer_options =
        optimizer_options_from_config(optimization, options, service);
    NlpEvaluator evaluator(*config, service);
    std::unique_ptr<IOptimizer> optimizer = make_optimizer(optimization.optimizer);
    if (!optimizer) {
        result.error = "unsupported optimizer: " + optimization.optimizer;
        return result;
    }

    OptimizerResult optimizer_result =
        optimizer->solve(problem, evaluator, optimizer_options);
    for (const auto& message : optimizer_result.messages) {
        append_limited_message(&result.messages, message);
    }
    result.iterations = optimizer_result.iterations;
    result.evaluations = optimizer_result.evaluations;
    result.best_score = optimizer_result.best_score;
    result.found_feasible = optimizer_result.found_feasible;
    result.max_constraint_violation = optimizer_result.max_constraint_violation;
    result.l1_constraint_violation = optimizer_result.l1_constraint_violation;

    if (optimizer_result.best_z.empty()) {
        result.error = optimizer_result.error.empty()
            ? "optimization failed before producing a valid candidate"
            : optimizer_result.error;
        return result;
    }

    CaseConfig solved;
    if (!evaluator.write_point_to_case(problem, optimizer_result.best_z, &solved, &result.error)) {
        return result;
    }
    solved.optimization = optimization;
    *config = solved;

    for (std::size_t i = 0; i < problem.variables.size(); ++i) {
        result.variable_changes.push_back({
            problem.variables[i].path,
            problem.variables[i].initial_value,
            problem.variables[i].from_scaled(optimizer_result.best_z[i]),
        });
    }

    if (options.run_final_simulation) {
        result.final_simulation = service.simulate(*config);
        ++result.evaluations;
        if (!result.final_simulation.ok) {
            result.error = "final simulation failed: " + result.final_simulation.error;
            return result;
        }
        result.final_metrics = evaluate_trajectory_metrics(result.final_simulation.state_log, *config);
    } else {
        result.final_simulation = optimizer_result.final_eval.simulation;
        result.final_metrics = optimizer_result.final_eval.metrics;
    }
    add_requested_metrics(&result, optimization, *config);

    if (!optimizer_result.found_feasible) {
        std::ostringstream msg;
        msg << "infeasible: best max_violation = " << optimizer_result.max_constraint_violation
            << " > tolerance = " << optimizer_options.constraint_tolerance;
        result.error = msg.str();
        append_limited_message(&result.messages, result.error);
        result.ok = false;
        return result;
    }
    result.ok = optimizer_result.ok;
    if (!result.ok && !optimizer_result.error.empty()) {
        result.error = optimizer_result.error;
    }
    return result;
}

OptimizerResult AugmentedLagrangianBfgsOptimizer::solve(
    const NlpProblem& problem,
    NlpEvaluator& evaluator,
    const OptimizerOptions& options)
{
    OptimizerResult result;
    result.messages.push_back("fmincon: interface solve");
    if (problem.variables.empty()) {
        result.error = "fmincon: no variables";
        return result;
    }

    const ConstraintSet cs = constraint_set_from_problem(problem);
    const NlpPoint point = make_initial_nlp_point(problem);
    const std::vector<double>& z = point.z;
    const int total_iteration_budget = std::max(1, options.max_iterations);
    const double tol = options.constraint_tolerance > 0.0
        ? options.constraint_tolerance
        : options.tolerance;

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

    LocalOptimResult run = run_local_augmented_lagrangian(
        z,
        problem,
        evaluator,
        cs,
        options,
        total_iteration_budget,
        tol,
        &result.messages);
    result.iterations += run.iterations;
    if (!run.error.empty()) {
        append_limited_message(&result.messages, run.error);
    }
    consider_run(std::move(run));

    result.evaluations = evaluator.evaluations();
    if (!have_report) {
        result.error = "fmincon: no valid candidate";
        return result;
    }
    result.found_feasible = best_run.found_feasible;
    result.best_z = best_run.report_z;
    result.final_eval = best_run.report_pe.nlp_eval;
    result.best_score = best_run.report_pe.objective_f + best_run.report_pe.max_violation;
    result.max_constraint_violation = best_run.report_pe.max_violation;
    result.l1_constraint_violation = sqp_violation_l1(best_run.report_pe);
    result.ok = result.found_feasible;
    if (!result.ok) {
        result.error = "fmincon: no feasible candidate";
    }
    return result;
}

OptimizerResult DenseSqpOptimizer::solve(
    const NlpProblem& problem,
    NlpEvaluator& evaluator,
    const OptimizerOptions& options)
{
    OptimizerResult result;
    result.messages.push_back("sqp: interface solve");
    if (problem.variables.empty()) {
        result.error = "sqp: no variables";
        return result;
    }

    const ConstraintSet cs = constraint_set_from_problem(problem);
    const NlpPoint point = make_initial_nlp_point(problem);
    std::unique_ptr<IDenseQpSolver> qp_solver = make_dense_qp_solver(options.qp_solver);
    LocalOptimResult run = run_sqp_nlp(
        point.z,
        problem,
        evaluator,
        cs,
        options,
        *qp_solver,
        std::max(1, options.max_iterations),
        options.constraint_tolerance > 0.0 ? options.constraint_tolerance : options.tolerance,
        &result.messages);
    result.iterations = run.iterations;
    result.evaluations = evaluator.evaluations();
    if (!run.error.empty()) {
        append_limited_message(&result.messages, run.error);
    }
    if (!run.report_pe.ok || run.report_z.empty()) {
        result.error = run.error.empty() ? "sqp: no valid candidate" : run.error;
        return result;
    }
    result.found_feasible = run.found_feasible;
    result.best_z = run.report_z;
    result.final_eval = run.report_pe.nlp_eval;
    result.best_score = run.report_pe.objective_f + run.report_pe.max_violation;
    result.max_constraint_violation = run.report_pe.max_violation;
    result.l1_constraint_violation = sqp_violation_l1(run.report_pe);
    result.ok = result.found_feasible;
    if (!result.ok) {
        result.error = "sqp: no feasible candidate";
    }
    return result;
}

std::unique_ptr<IOptimizer> make_optimizer(const std::string& name)
{
    if (name == "sqp") {
        return std::make_unique<DenseSqpOptimizer>();
    }
    return std::make_unique<AugmentedLagrangianBfgsOptimizer>();
}

} // namespace post2::core
