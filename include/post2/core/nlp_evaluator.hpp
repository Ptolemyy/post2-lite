#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "post2/core/nlp_problem.hpp"
#include "post2/core/trajectory_service.hpp"

namespace post2::core {

struct MetricQuery {
    std::string metric;
    std::string_view base_metric;
    int phase_index = -1;
};

bool parse_metric_query(const std::string& metric, MetricQuery* query);

bool evaluate_trajectory_metric(
    const StateLog& state_log,
    const CaseConfig& config,
    const std::string& metric,
    double* value);

std::vector<OptimizationMetricValue> evaluate_extended_trajectory_metrics(
    const StateLog& state_log,
    const CaseConfig& config);

class NlpEvaluator {
public:
    NlpEvaluator(CaseConfig base_case, ITrajectoryService& service);

    const CaseConfig& base_case() const { return base_case_; }
    int evaluations() const { return evaluations_; }
    bool supports_parallel_probes() const { return service_.supports_parallel_simulation(); }
    NlpEvaluator clone_with_base_case(CaseConfig base_case) const;

    NlpEvalResult evaluate(const NlpProblem& problem, const std::vector<double>& z);
    std::vector<NlpEvalResult> evaluate_many(
        const NlpProblem& problem,
        const std::vector<std::vector<double>>& points,
        bool parallel);
    bool write_point_to_case(
        const NlpProblem& problem,
        const std::vector<double>& z,
        CaseConfig* output,
        std::string* error) const;

private:
    NlpEvalResult evaluate_no_count(const NlpProblem& problem, const std::vector<double>& z) const;

    CaseConfig base_case_;
    ITrajectoryService& service_;
    int evaluations_ = 0;
};

} // namespace post2::core
