#pragma once

#include <memory>
#include <string>
#include <vector>

#include "post2/core/finite_difference.hpp"
#include "post2/core/nlp_evaluator.hpp"

namespace post2::core {

struct OptimizerOptions {
    int max_iterations = 100;
    double tolerance = 1.0e-4;
    double stationarity_tolerance = 1.0e-4;
    double feasibility_tolerance = 1.0e-4;
    double constraint_tolerance = 1.0e-4;
    double initial_step_fraction = 0.25;
    int max_restoration_iterations = 8;
    std::string qp_solver = "kkt-fallback";
    FiniteDifferenceOptions finite_difference;
};

struct OptimizerResult {
    bool ok = false;
    bool found_feasible = false;
    std::string error;
    int iterations = 0;
    int evaluations = 0;
    double best_score = 0.0;
    double max_constraint_violation = 0.0;
    double l1_constraint_violation = 0.0;
    std::vector<double> best_z;
    std::vector<std::string> messages;
    NlpEvalResult final_eval;
};

class IOptimizer {
public:
    virtual ~IOptimizer() = default;
    virtual OptimizerResult solve(
        const NlpProblem& problem,
        NlpEvaluator& evaluator,
        const OptimizerOptions& options) = 0;
};

class AugmentedLagrangianBfgsOptimizer final : public IOptimizer {
public:
    OptimizerResult solve(
        const NlpProblem& problem,
        NlpEvaluator& evaluator,
        const OptimizerOptions& options) override;
};

class DenseSqpOptimizer final : public IOptimizer {
public:
    OptimizerResult solve(
        const NlpProblem& problem,
        NlpEvaluator& evaluator,
        const OptimizerOptions& options) override;
};

std::unique_ptr<IOptimizer> make_optimizer(const std::string& name);

} // namespace post2::core
