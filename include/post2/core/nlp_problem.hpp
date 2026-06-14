#pragma once

#include <string>
#include <vector>

#include "post2/core/types.hpp"

namespace post2::core {

enum class NlpConstraintType {
    Equality,
    LowerBound,
    UpperBound,
    Range,
    Path,
};

enum class NlpObjectiveDirection {
    Minimize,
    Maximize,
};

struct NlpVariable {
    std::string name;
    std::string path;
    double lower = 0.0;
    double upper = 0.0;
    double initial_value = 0.0;
    int phase_index = -1;

    double clamp_real(double value) const;
    double to_scaled(double value) const;
    double from_scaled(double value) const;
};

struct NlpConstraint {
    std::string name;
    std::string metric;
    NlpConstraintType type = NlpConstraintType::Equality;
    double target = 0.0;
    double min_value = 0.0;
    double max_value = 0.0;
    double scale = 1.0;
    double weight = 1.0;
    std::string scope = "terminal";
    int phase_index = -1;
};

struct NlpObjective {
    bool enabled = false;
    std::string metric = "terminal_altitude_m";
    NlpObjectiveDirection direction = NlpObjectiveDirection::Minimize;
    double weight = 1.0;
};

struct NlpPoint {
    std::vector<double> x;
    std::vector<double> z;
};

struct NlpEvalResult {
    bool ok = false;
    std::string error;
    double objective = 0.0;
    std::vector<double> equality_residuals;
    std::vector<double> inequality_residuals;
    std::vector<double> constraint_values;
    double max_violation = 0.0;
    double l1_violation = 0.0;
    std::vector<OptimizationMetricValue> metrics;
    SimulationResult simulation;
};

struct NlpProblem {
    CaseConfig base_case;
    OptimizationConfig optimization;
    std::vector<NlpVariable> variables;
    std::vector<NlpConstraint> constraints;
    std::vector<NlpObjective> objectives;
    NlpObjective objective;
};

const char* nlp_constraint_type_name(NlpConstraintType type);
bool parse_nlp_constraint_type(const std::string& text, NlpConstraintType* type);

double nlp_default_metric_scale(const std::string& metric);
double nlp_metric_scale(const std::string& metric, double reference);

bool build_nlp_problem_from_case(
    const CaseConfig& config,
    NlpProblem* problem,
    std::string* error);

NlpPoint make_initial_nlp_point(const NlpProblem& problem);

} // namespace post2::core
