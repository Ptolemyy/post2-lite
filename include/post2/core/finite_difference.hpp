#pragma once

#include <functional>
#include <string>
#include <vector>

#include "post2/core/nlp_evaluator.hpp"

namespace post2::core {

enum class FiniteDifferenceMode {
    Forward,
    Backward,
    Central,
    Auto,
};

struct FiniteDifferenceOptions {
    FiniteDifferenceMode mode = FiniteDifferenceMode::Auto;
    double step_fraction = 0.01;
    bool parallel = true;
};

struct FiniteDifferenceResult {
    bool ok = false;
    std::string error;
    std::vector<double> gradient;
    std::vector<std::string> messages;
};

struct NlpDerivativeResult {
    bool ok = false;
    std::string error;
    std::vector<double> grad_f;
    std::vector<std::vector<double>> J_eq;
    std::vector<std::vector<double>> J_ineq;
    std::vector<std::string> messages;
};

const char* finite_difference_mode_name(FiniteDifferenceMode mode);
bool parse_finite_difference_mode(const std::string& text, FiniteDifferenceMode* mode);

FiniteDifferenceResult finite_difference_gradient(
    const std::vector<double>& x,
    const std::function<bool(const std::vector<double>&, double*, std::string*)>& f,
    const FiniteDifferenceOptions& options);

NlpDerivativeResult finite_difference_nlp(
    const NlpProblem& problem,
    NlpEvaluator& evaluator,
    const std::vector<double>& z,
    const NlpEvalResult& base_eval,
    const FiniteDifferenceOptions& options);

} // namespace post2::core
