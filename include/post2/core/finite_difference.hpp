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
    // Per-variable starting step fraction in z-space. When non-empty and
    // sized to the number of NLP variables, each variable's first probe is
    // queued at this fraction (still subject to bounds clamping). Empty
    // means "use step_fraction uniformly". Used to carry per-variable
    // adaptation across SQP iterations: variables whose probes repeatedly
    // crash the simulator get shrunk hints so subsequent iterations don't
    // re-waste the doomed nominal-h probe.
    std::vector<double> step_fraction_per_variable;
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
    // Per-variable effective step (z-space) of the probe that actually
    // contributed to the derivative. 0.0 means no usable probe. Caller uses
    // this to seed step_fraction_per_variable for the next FD call so the
    // adaptive shrinkage is carried across SQP iterations.
    std::vector<double> used_step_per_variable;
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
