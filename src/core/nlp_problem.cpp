#include "post2/core/nlp_problem.hpp"

#include "post2/core/optimization.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace post2::core {

namespace {

double clamp(double value, double lower, double upper)
{
    return std::min(std::max(value, lower), upper);
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

int phase_index_from_path(const std::string& path)
{
    std::string_view text(path);
    int index = -1;
    if (consume_indexed(&text, "phases", &index)) {
        return index;
    }
    return -1;
}

bool metric_has_phase_prefix(const std::string& metric)
{
    std::string_view text(metric);
    int index = -1;
    return consume_indexed(&text, "phases", &index) && !text.empty() && text.front() == '.';
}

std::string scoped_metric(const OptimizationTargetConfig& target)
{
    if (target.phase_index >= 0 && !metric_has_phase_prefix(target.metric)) {
        return "phases[" + std::to_string(target.phase_index) + "]." + target.metric;
    }
    return target.metric;
}

NlpConstraint make_constraint(
    const OptimizationTargetConfig& target,
    NlpConstraintType type,
    const std::string& suffix)
{
    NlpConstraint c;
    c.metric = scoped_metric(target);
    c.name = c.metric + suffix;
    c.type = type;
    c.target = target.value;
    c.min_value = target.min_value;
    c.max_value = target.max_value;
    c.weight = target.weight;
    c.scope = target.scope;
    c.phase_index = target.phase_index;
    double reference = target.value;
    if (type == NlpConstraintType::LowerBound) {
        reference = target.min_value != 0.0 ? target.min_value : target.value;
    } else if (type == NlpConstraintType::UpperBound) {
        reference = target.max_value != 0.0 ? target.max_value : target.value;
    } else if (type == NlpConstraintType::Range) {
        reference = std::max(std::abs(target.min_value), std::abs(target.max_value));
    }
    c.scale = nlp_metric_scale(c.metric, reference);
    return c;
}

} // namespace

double NlpVariable::clamp_real(double value) const
{
    return clamp(value, lower, upper);
}

double NlpVariable::to_scaled(double value) const
{
    if (upper <= lower) {
        return 0.0;
    }
    return 2.0 * (clamp_real(value) - lower) / (upper - lower) - 1.0;
}

double NlpVariable::from_scaled(double value) const
{
    if (upper <= lower) {
        return lower;
    }
    const double z = clamp(value, -1.0, 1.0);
    return lower + (z + 1.0) * 0.5 * (upper - lower);
}

const char* nlp_constraint_type_name(NlpConstraintType type)
{
    switch (type) {
    case NlpConstraintType::Equality:
        return "equality";
    case NlpConstraintType::LowerBound:
        return "lower_bound";
    case NlpConstraintType::UpperBound:
        return "upper_bound";
    case NlpConstraintType::Range:
        return "range";
    case NlpConstraintType::Path:
        return "path";
    }
    return "unknown";
}

bool parse_nlp_constraint_type(const std::string& text, NlpConstraintType* type)
{
    if (text == "equal" || text == "equality") {
        *type = NlpConstraintType::Equality;
        return true;
    }
    if (text == "lower" || text == "lower_bound") {
        *type = NlpConstraintType::LowerBound;
        return true;
    }
    if (text == "upper" || text == "upper_bound") {
        *type = NlpConstraintType::UpperBound;
        return true;
    }
    if (text == "range") {
        *type = NlpConstraintType::Range;
        return true;
    }
    if (text == "path") {
        *type = NlpConstraintType::Path;
        return true;
    }
    return false;
}

double nlp_default_metric_scale(const std::string& metric)
{
    std::string_view name(metric);
    if (metric_has_phase_prefix(metric)) {
        std::string_view text(metric);
        int index = -1;
        consume_indexed(&text, "phases", &index);
        if (!text.empty() && text.front() == '.') {
            text.remove_prefix(1);
            name = text;
        }
    }
    if (name == "terminal_altitude_m" ||
        name == "periapsis_altitude_m" ||
        name == "apoapsis_altitude_m" ||
        name == "downrange_m" ||
        name == "min_altitude_m") {
        return 1000.0;
    }
    if (name == "terminal_speed_mps") {
        return 100.0;
    }
    if (name == "max_q_pa" || name == "max_dynamic_pressure_pa") {
        return 1000.0;
    }
    if (name == "max_accel_mps2") {
        return 10.0;
    }
    if (name == "payload_mass_kg" || name == "propellant_remaining_kg") {
        return 1000.0;
    }
    return 1.0;
}

double nlp_metric_scale(const std::string& metric, double reference)
{
    return std::max(nlp_default_metric_scale(metric), std::abs(reference));
}

std::vector<OptimizationObjectiveConfig> nlp_effective_objective_configs(
    const OptimizationConfig& optimization)
{
    if (!optimization.objectives.empty()) {
        return optimization.objectives;
    }
    return {optimization.objective};
}

NlpObjective make_objective(const OptimizationObjectiveConfig& objective)
{
    NlpObjective out;
    out.enabled = objective.enabled;
    out.metric = objective.metric;
    out.weight = objective.weight;
    out.direction = objective.direction == "maximize"
        ? NlpObjectiveDirection::Maximize
        : NlpObjectiveDirection::Minimize;
    return out;
}

bool build_nlp_problem_from_case(
    const CaseConfig& config,
    NlpProblem* problem,
    std::string* error)
{
    if (!problem) {
        if (error) {
            *error = "nlp problem output is null";
        }
        return false;
    }
    NlpProblem out;
    out.base_case = config;
    out.optimization = config.optimization;
    for (const auto& objective : nlp_effective_objective_configs(config.optimization)) {
        NlpObjective nlp_objective = make_objective(objective);
        if (!nlp_objective.enabled || nlp_objective.weight <= 0.0) {
            continue;
        }
        out.objectives.push_back(std::move(nlp_objective));
    }
    if (!out.objectives.empty()) {
        out.objective = out.objectives.front();
    } else {
        out.objective = make_objective(config.optimization.objective);
        out.objective.enabled = false;
    }

    for (const auto& variable : config.optimization.variables) {
        if (!variable.enabled) {
            continue;
        }
        if (variable.min_value > variable.max_value) {
            if (error) {
                *error = "variable min is greater than max: " + variable.path;
            }
            return false;
        }
        double initial_value = 0.0;
        if (!read_optimization_variable(config, variable.path, &initial_value, error)) {
            return false;
        }
        const int phase_index = phase_index_from_path(variable.path);
        if (phase_index >= 0 &&
            static_cast<std::size_t>(phase_index) < config.phases.size() &&
            !config.phases[static_cast<std::size_t>(phase_index)].optimize_enabled) {
            continue;
        }
        NlpVariable nlp_var;
        nlp_var.name = variable.path;
        nlp_var.path = variable.path;
        nlp_var.lower = variable.min_value;
        nlp_var.upper = variable.max_value;
        nlp_var.initial_value = initial_value;
        nlp_var.phase_index = phase_index;
        out.variables.push_back(std::move(nlp_var));
    }

    for (const auto& target : config.optimization.targets) {
        if (target.weight <= 0.0) {
            continue;
        }
        if (target.mode == "equal" || target.mode == "equality") {
            out.constraints.push_back(make_constraint(target, NlpConstraintType::Equality, ".eq"));
        } else if (target.mode == "range") {
            out.constraints.push_back(make_constraint(target, NlpConstraintType::Range, ".range"));
        } else if (target.mode == "upper" || target.mode == "upper_bound") {
            out.constraints.push_back(make_constraint(target, NlpConstraintType::UpperBound, ".upper"));
        } else if (target.mode == "lower" || target.mode == "lower_bound") {
            out.constraints.push_back(make_constraint(target, NlpConstraintType::LowerBound, ".lower"));
        } else {
            if (error) {
                *error = "unsupported target mode: " + target.mode;
            }
            return false;
        }
    }

    *problem = std::move(out);
    return true;
}

NlpPoint make_initial_nlp_point(const NlpProblem& problem)
{
    NlpPoint point;
    point.x.reserve(problem.variables.size());
    point.z.reserve(problem.variables.size());
    for (const auto& variable : problem.variables) {
        const double x = variable.clamp_real(variable.initial_value);
        point.x.push_back(x);
        point.z.push_back(variable.to_scaled(x));
    }
    return point;
}

} // namespace post2::core
