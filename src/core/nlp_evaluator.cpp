#include "post2/core/nlp_evaluator.hpp"

#include "post2/core/frames.hpp"
#include "post2/core/optimization.hpp"
#include "post2/vehicle/vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <string_view>
#include <utility>

namespace post2::core {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

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

bool consume_identifier(std::string_view* text, std::string_view id)
{
    if (!consume_literal(text, id)) {
        return false;
    }
    return text->empty() || text->front() == '.' || text->front() == '[';
}

bool consume_indexed(std::string_view* text, std::string_view id, int* index)
{
    std::string_view working = *text;
    if (!consume_identifier(&working, id) || working.empty() || working.front() != '[') {
        return false;
    }
    working.remove_prefix(1);
    int parsed = 0;
    int digits = 0;
    while (!working.empty() && working.front() >= '0' && working.front() <= '9') {
        parsed = parsed * 10 + static_cast<int>(working.front() - '0');
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

std::vector<const LaunchVehicleStateLogEntry*> entries_for_metric(
    const StateLog& state_log,
    const MetricQuery& query)
{
    std::vector<const LaunchVehicleStateLogEntry*> entries;
    for (const auto& entry : state_log.entries()) {
        if (query.phase_index < 0 || entry.phase_index == query.phase_index) {
            entries.push_back(&entry);
        }
    }
    return entries;
}

const LaunchVehicleStateLogEntry* terminal_entry_for_metric(
    const StateLog& state_log,
    const MetricQuery& query)
{
    if (state_log.empty()) {
        return nullptr;
    }
    if (query.phase_index < 0) {
        return &state_log.back();
    }
    for (auto it = state_log.entries().rbegin(); it != state_log.entries().rend(); ++it) {
        if (it->phase_index == query.phase_index) {
            return &(*it);
        }
    }
    return nullptr;
}

double clamp_unit(double value)
{
    return std::clamp(value, -1.0, 1.0);
}

struct OrbitalElements {
    double eccentricity = std::numeric_limits<double>::quiet_NaN();
    double periapsis_altitude_m = std::numeric_limits<double>::quiet_NaN();
    double apoapsis_altitude_m = std::numeric_limits<double>::quiet_NaN();
};

OrbitalElements orbital_elements_for_entry(
    const LaunchVehicleStateLogEntry& entry,
    const CaseConfig& config)
{
    OrbitalElements out;
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
        return out;
    }
    const Vec3 e_vec =
        ((v_norm * v_norm - config.earth_mu_m3s2 / r_norm) * r - rv * v) /
        config.earth_mu_m3s2;
    out.eccentricity = post2::vehicle::norm(e_vec);
    const double p = h_norm * h_norm / config.earth_mu_m3s2;
    if (1.0 + out.eccentricity > 0.0) {
        out.periapsis_altitude_m = p / (1.0 + out.eccentricity) - config.earth_radius_m;
    }
    if (out.eccentricity < 1.0 && 1.0 - out.eccentricity > 0.0) {
        out.apoapsis_altitude_m = p / (1.0 - out.eccentricity) - config.earth_radius_m;
    }
    return out;
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
    return std::acos(clamp_unit(h.z / h_norm)) * 180.0 / kPi;
}

double flight_path_angle_deg_for_entry(const LaunchVehicleStateLogEntry& entry)
{
    const double r_norm = post2::vehicle::norm(entry.state.position_m);
    const double v_norm = post2::vehicle::norm(entry.state.velocity_mps);
    if (r_norm <= 0.0 || v_norm <= 0.0) {
        return 0.0;
    }
    const double sine = post2::vehicle::dot(entry.state.position_m, entry.state.velocity_mps) /
        (r_norm * v_norm);
    return std::asin(clamp_unit(sine)) * 180.0 / kPi;
}

frames::Geodetic geodetic_for_entry(
    const LaunchVehicleStateLogEntry& entry,
    const CaseConfig& config)
{
    const double theta = frames::earth_rotation_angle_rad(
        config.earth_rotation_at_epoch_rad,
        config.earth_rotation_rad_per_s,
        entry.time_s);
    return frames::ecef_to_geodetic(
        frames::eci_to_ecef_position(entry.state.position_m, theta));
}

double great_circle_downrange_m(
    const LaunchVehicleStateLogEntry& first,
    const LaunchVehicleStateLogEntry& entry,
    const CaseConfig& config)
{
    const frames::Geodetic a = geodetic_for_entry(first, config);
    const frames::Geodetic b = geodetic_for_entry(entry, config);
    const double dlat = b.latitude_rad - a.latitude_rad;
    const double dlon = b.longitude_rad - a.longitude_rad;
    const double hav =
        std::sin(dlat * 0.5) * std::sin(dlat * 0.5) +
        std::cos(a.latitude_rad) * std::cos(b.latitude_rad) *
            std::sin(dlon * 0.5) * std::sin(dlon * 0.5);
    return 2.0 * config.earth_radius_m *
        std::asin(std::sqrt(std::min(1.0, std::max(0.0, hav))));
}

template <typename Select>
bool max_over_entries(
    const std::vector<const LaunchVehicleStateLogEntry*>& entries,
    Select select,
    double* value)
{
    if (entries.empty()) {
        return false;
    }
    double best = -std::numeric_limits<double>::infinity();
    for (const auto* entry : entries) {
        best = std::max(best, select(*entry));
    }
    *value = best;
    return true;
}

template <typename Select>
bool min_over_entries(
    const std::vector<const LaunchVehicleStateLogEntry*>& entries,
    Select select,
    double* value)
{
    if (entries.empty()) {
        return false;
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto* entry : entries) {
        best = std::min(best, select(*entry));
    }
    *value = best;
    return true;
}

double objective_value(
    const NlpProblem& problem,
    const StateLog& state_log,
    const CaseConfig& config)
{
    if (problem.optimization.mode != "optimize" || problem.objectives.empty()) {
        return 0.0;
    }
    double total = 0.0;
    for (const auto& objective : problem.objectives) {
        if (!objective.enabled || objective.weight <= 0.0) {
            continue;
        }
        double metric = 0.0;
        if (!evaluate_trajectory_metric(state_log, config, objective.metric, &metric)) {
            continue;
        }
        const double scaled = metric / nlp_default_metric_scale(objective.metric);
        if (objective.direction == NlpObjectiveDirection::Maximize) {
            total -= objective.weight * scaled;
        } else {
            total += objective.weight * scaled;
        }
    }
    return total;
}

void append_constraint_eval(
    const NlpConstraint& constraint,
    double metric_value,
    NlpEvalResult* result)
{
    auto add_ineq = [&](double residual) {
        result->inequality_residuals.push_back(residual);
        result->constraint_values.push_back(residual);
        result->max_violation = std::max(result->max_violation, std::max(0.0, residual));
        result->l1_violation += std::max(0.0, residual);
    };
    auto add_eq = [&](double residual) {
        result->equality_residuals.push_back(residual);
        result->constraint_values.push_back(residual);
        result->max_violation = std::max(result->max_violation, std::abs(residual));
        result->l1_violation += std::abs(residual);
    };

    const double scale = constraint.scale > 0.0 ? constraint.scale : 1.0;
    switch (constraint.type) {
    case NlpConstraintType::Equality:
        add_eq((metric_value - constraint.target) / scale);
        break;
    case NlpConstraintType::LowerBound:
    case NlpConstraintType::Path:
        add_ineq((constraint.min_value - metric_value) / scale);
        break;
    case NlpConstraintType::UpperBound:
        add_ineq((metric_value - constraint.max_value) / scale);
        break;
    case NlpConstraintType::Range:
        add_ineq((constraint.min_value - metric_value) / scale);
        add_ineq((metric_value - constraint.max_value) / scale);
        break;
    }
}

} // namespace

bool parse_metric_query(const std::string& metric, MetricQuery* query)
{
    if (!query) {
        return false;
    }
    query->metric = metric;
    query->base_metric = query->metric;
    query->phase_index = -1;
    std::string_view text(query->metric);
    int phase_index = -1;
    if (consume_indexed(&text, "phases", &phase_index) && !text.empty() && text.front() == '.') {
        text.remove_prefix(1);
        query->phase_index = phase_index;
        query->base_metric = text;
    }
    return true;
}

bool evaluate_trajectory_metric(
    const StateLog& state_log,
    const CaseConfig& config,
    const std::string& metric,
    double* value)
{
    if (!value) {
        return false;
    }
    MetricQuery query;
    parse_metric_query(metric, &query);

    if (query.base_metric == "payload_mass_kg") {
        *value = post2::vehicle::payload_stage_dry_mass_kg(config.vehicle);
        return true;
    }

    const std::vector<const LaunchVehicleStateLogEntry*> entries =
        entries_for_metric(state_log, query);
    const LaunchVehicleStateLogEntry* terminal =
        terminal_entry_for_metric(state_log, query);
    if (!terminal) {
        return false;
    }

    if (query.base_metric == "terminal_altitude_m") {
        *value = terminal->altitude_m;
        return true;
    }
    if (query.base_metric == "terminal_speed_mps") {
        *value = terminal->speed_mps;
        return true;
    }
    if (query.base_metric == "inclination_deg") {
        *value = inclination_deg_for_entry(*terminal);
        return true;
    }
    if (query.base_metric == "periapsis_altitude_m") {
        *value = orbital_elements_for_entry(*terminal, config).periapsis_altitude_m;
        return true;
    }
    if (query.base_metric == "apoapsis_altitude_m") {
        *value = orbital_elements_for_entry(*terminal, config).apoapsis_altitude_m;
        return true;
    }
    if (query.base_metric == "eccentricity") {
        *value = orbital_elements_for_entry(*terminal, config).eccentricity;
        return true;
    }
    if (query.base_metric == "flight_path_angle_deg") {
        *value = flight_path_angle_deg_for_entry(*terminal);
        return true;
    }
    if (query.base_metric == "latitude_deg") {
        *value = geodetic_for_entry(*terminal, config).latitude_rad * 180.0 / kPi;
        return true;
    }
    if (query.base_metric == "longitude_deg") {
        *value = geodetic_for_entry(*terminal, config).longitude_rad * 180.0 / kPi;
        return true;
    }
    if (query.base_metric == "downrange_m") {
        const auto all_entries = state_log.entries();
        if (all_entries.empty()) {
            return false;
        }
        *value = great_circle_downrange_m(all_entries.front(), *terminal, config);
        return true;
    }
    if (query.base_metric == "propellant_remaining_kg") {
        *value = terminal->propellant_mass_kg;
        return true;
    }
    if (query.base_metric == "duration_s") {
        if (entries.empty()) {
            return false;
        }
        *value = entries.back()->time_s - entries.front()->time_s;
        return true;
    }
    if (query.base_metric == "max_q_pa" ||
        query.base_metric == "max_dynamic_pressure_pa") {
        return max_over_entries(entries, [](const LaunchVehicleStateLogEntry& entry) {
            return entry.dynamic_pressure_pa;
        }, value);
    }
    if (query.base_metric == "max_accel_mps2") {
        return max_over_entries(entries, [](const LaunchVehicleStateLogEntry& entry) {
            return entry.acceleration_mps2;
        }, value);
    }
    if (query.base_metric == "min_altitude_m") {
        return min_over_entries(entries, [](const LaunchVehicleStateLogEntry& entry) {
            return entry.altitude_m;
        }, value);
    }
    if (query.base_metric == "min_throttle") {
        return min_over_entries(entries, [](const LaunchVehicleStateLogEntry& entry) {
            return entry.throttle;
        }, value);
    }
    if (query.base_metric == "max_throttle") {
        return max_over_entries(entries, [](const LaunchVehicleStateLogEntry& entry) {
            return entry.throttle;
        }, value);
    }
    return false;
}

std::vector<OptimizationMetricValue> evaluate_extended_trajectory_metrics(
    const StateLog& state_log,
    const CaseConfig& config)
{
    const std::vector<std::string> metrics = {
        "terminal_altitude_m",
        "terminal_speed_mps",
        "inclination_deg",
        "periapsis_altitude_m",
        "apoapsis_altitude_m",
        "eccentricity",
        "flight_path_angle_deg",
        "downrange_m",
        "latitude_deg",
        "longitude_deg",
        "payload_mass_kg",
        "propellant_remaining_kg",
        "max_q_pa",
        "max_dynamic_pressure_pa",
        "max_accel_mps2",
        "min_altitude_m",
        "min_throttle",
        "max_throttle",
    };
    std::vector<OptimizationMetricValue> out;
    out.reserve(metrics.size());
    for (const auto& metric : metrics) {
        double v = std::numeric_limits<double>::quiet_NaN();
        evaluate_trajectory_metric(state_log, config, metric, &v);
        out.push_back({metric, v});
    }
    return out;
}

NlpEvaluator::NlpEvaluator(CaseConfig base_case, ITrajectoryService& service)
    : base_case_(std::move(base_case))
    , service_(service)
{
}

NlpEvaluator NlpEvaluator::clone_with_base_case(CaseConfig base_case) const
{
    return NlpEvaluator(std::move(base_case), service_);
}

bool NlpEvaluator::write_point_to_case(
    const NlpProblem& problem,
    const std::vector<double>& z,
    CaseConfig* output,
    std::string* error) const
{
    if (!output) {
        if (error) {
            *error = "case output is null";
        }
        return false;
    }
    if (z.size() != problem.variables.size()) {
        if (error) {
            *error = "nlp point dimension does not match variables";
        }
        return false;
    }
    *output = base_case_;
    for (std::size_t i = 0; i < problem.variables.size(); ++i) {
        const double x = problem.variables[i].from_scaled(z[i]);
        if (!write_optimization_variable(output, problem.variables[i].path, x, error)) {
            return false;
        }
    }
    return true;
}

NlpEvalResult NlpEvaluator::evaluate(const NlpProblem& problem, const std::vector<double>& z)
{
    NlpEvalResult result = evaluate_no_count(problem, z);
    if (result.error != "nlp point dimension does not match variables" &&
        result.error != "case output is null") {
        ++evaluations_;
    }
    return result;
}

std::vector<NlpEvalResult> NlpEvaluator::evaluate_many(
    const NlpProblem& problem,
    const std::vector<std::vector<double>>& points,
    bool parallel)
{
    std::vector<NlpEvalResult> out(points.size());
    if (points.empty()) {
        return out;
    }
    if (parallel && supports_parallel_probes() && points.size() > 1) {
        std::vector<std::future<NlpEvalResult>> futures;
        futures.reserve(points.size());
        for (const auto& point : points) {
            futures.push_back(std::async(std::launch::async, [this, &problem, point]() {
                return evaluate_no_count(problem, point);
            }));
        }
        for (std::size_t i = 0; i < futures.size(); ++i) {
            out[i] = futures[i].get();
        }
    } else {
        for (std::size_t i = 0; i < points.size(); ++i) {
            out[i] = evaluate_no_count(problem, points[i]);
        }
    }
    evaluations_ += static_cast<int>(points.size());
    return out;
}

NlpEvalResult NlpEvaluator::evaluate_no_count(
    const NlpProblem& problem,
    const std::vector<double>& z) const
{
    NlpEvalResult result;
    CaseConfig working;
    if (!write_point_to_case(problem, z, &working, &result.error)) {
        return result;
    }

    result.simulation = service_.simulate(working);
    if (!result.simulation.ok) {
        result.error = result.simulation.error;
        return result;
    }
    if (result.simulation.state_log.empty()) {
        result.error = "simulation produced an empty StateLog";
        return result;
    }

    result.metrics = evaluate_extended_trajectory_metrics(result.simulation.state_log, working);
    result.objective = objective_value(problem, result.simulation.state_log, working);
    for (const auto& constraint : problem.constraints) {
        double metric_value = 0.0;
        if (!evaluate_trajectory_metric(
                result.simulation.state_log,
                working,
                constraint.metric,
                &metric_value)) {
            result.error = "unsupported metric: " + constraint.metric;
            return result;
        }
        if (!std::isfinite(metric_value)) {
            result.error = "non-finite metric: " + constraint.metric;
            return result;
        }
        append_constraint_eval(constraint, metric_value, &result);
    }
    result.ok = std::isfinite(result.objective);
    return result;
}

} // namespace post2::core
