#include "post2/core/finite_difference.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>

namespace post2::core {

namespace {

double fd_step(double step_fraction)
{
    return std::max(step_fraction, 1.0e-6);
}

double nlp_fd_step(double step_fraction)
{
    const double requested = step_fraction > 0.0 ? step_fraction : 1.0e-3;
    return std::clamp(requested, 1.0e-5, 5.0e-2);
}

void append_message(std::vector<std::string>* messages, const std::string& message)
{
    if (messages && !message.empty()) {
        messages->push_back(message);
    }
}

bool residual_shape_matches(const NlpEvalResult& candidate, const NlpEvalResult& base)
{
    return candidate.equality_residuals.size() == base.equality_residuals.size() &&
        candidate.inequality_residuals.size() == base.inequality_residuals.size();
}

bool usable_probe(
    const NlpEvalResult& candidate,
    const NlpEvalResult& base,
    std::string* error)
{
    if (!candidate.ok) {
        if (error) {
            *error = candidate.error.empty() ? "probe evaluation failed" : candidate.error;
        }
        return false;
    }
    if (!std::isfinite(candidate.objective)) {
        if (error) {
            *error = "probe objective is not finite";
        }
        return false;
    }
    if (!residual_shape_matches(candidate, base)) {
        if (error) {
            *error = "probe residual shape changed";
        }
        return false;
    }
    return true;
}

struct ProbeSlot {
    bool possible = false;
    bool usable = false;
    double h = 0.0;
    NlpEvalResult eval;
};

struct ProbeState {
    ProbeSlot plus;
    ProbeSlot minus;
};

struct ProbeRequest {
    std::size_t variable = 0;
    int side = 1;
    double h = 0.0;
    std::vector<double> point;
};

bool side_requested_initial(FiniteDifferenceMode mode, int side, bool plus_possible, bool minus_possible)
{
    if (mode == FiniteDifferenceMode::Forward) {
        return side > 0 ? plus_possible : !plus_possible && minus_possible;
    }
    if (mode == FiniteDifferenceMode::Backward) {
        return side < 0 ? minus_possible : !minus_possible && plus_possible;
    }
    if (mode == FiniteDifferenceMode::Central) {
        return side > 0 ? plus_possible : minus_possible;
    }
    if (plus_possible && minus_possible) {
        return true;
    }
    return side > 0 ? plus_possible : minus_possible;
}

template <typename PlusValueAt, typename MinusValueAt>
double asymmetric_central_derivative(
    double f0,
    double hp,
    double hm,
    PlusValueAt value_at_plus,
    MinusValueAt value_at_minus)
{
    return (hm * hm * (value_at_plus() - f0) +
               hp * hp * (f0 - value_at_minus())) /
        (hp * hm * (hp + hm));
}

} // namespace

const char* finite_difference_mode_name(FiniteDifferenceMode mode)
{
    switch (mode) {
    case FiniteDifferenceMode::Forward:
        return "forward";
    case FiniteDifferenceMode::Backward:
        return "backward";
    case FiniteDifferenceMode::Central:
        return "central";
    case FiniteDifferenceMode::Auto:
        return "auto";
    }
    return "unknown";
}

bool parse_finite_difference_mode(const std::string& text, FiniteDifferenceMode* mode)
{
    if (text == "forward") {
        *mode = FiniteDifferenceMode::Forward;
        return true;
    }
    if (text == "backward") {
        *mode = FiniteDifferenceMode::Backward;
        return true;
    }
    if (text == "central") {
        *mode = FiniteDifferenceMode::Central;
        return true;
    }
    if (text == "auto") {
        *mode = FiniteDifferenceMode::Auto;
        return true;
    }
    return false;
}

FiniteDifferenceResult finite_difference_gradient(
    const std::vector<double>& x,
    const std::function<bool(const std::vector<double>&, double*, std::string*)>& f,
    const FiniteDifferenceOptions& options)
{
    FiniteDifferenceResult out;
    out.gradient.assign(x.size(), 0.0);

    double f0 = 0.0;
    if (!f(x, &f0, &out.error)) {
        return out;
    }
    const double h = fd_step(options.step_fraction);
    for (std::size_t i = 0; i < x.size(); ++i) {
        std::vector<double> xp = x;
        std::vector<double> xm = x;
        xp[i] += h;
        xm[i] -= h;

        double fp = 0.0;
        double fm = 0.0;
        std::string ep;
        std::string em;
        const bool want_backward = options.mode == FiniteDifferenceMode::Backward;
        const bool want_forward = options.mode == FiniteDifferenceMode::Forward;
        const bool have_plus = !want_backward && f(xp, &fp, &ep);
        const bool have_minus = !want_forward && f(xm, &fm, &em);

        if ((options.mode == FiniteDifferenceMode::Central ||
             options.mode == FiniteDifferenceMode::Auto) &&
            have_plus && have_minus) {
            out.gradient[i] = (fp - fm) / (2.0 * h);
        } else if (have_plus) {
            out.gradient[i] = (fp - f0) / h;
            if (!em.empty()) {
                append_message(&out.messages, em);
            }
        } else if (have_minus) {
            out.gradient[i] = (f0 - fm) / h;
            if (!ep.empty()) {
                append_message(&out.messages, ep);
            }
        } else {
            out.error = "finite difference failed for variable " + std::to_string(i);
            if (!ep.empty()) {
                append_message(&out.messages, ep);
            }
            if (!em.empty()) {
                append_message(&out.messages, em);
            }
            return out;
        }
    }
    out.ok = true;
    return out;
}

NlpDerivativeResult finite_difference_nlp(
    const NlpProblem& problem,
    NlpEvaluator& evaluator,
    const std::vector<double>& z,
    const NlpEvalResult& base_eval,
    const FiniteDifferenceOptions& options)
{
    NlpDerivativeResult out;
    const std::size_t n = z.size();
    out.grad_f.assign(n, 0.0);
    out.J_eq.assign(base_eval.equality_residuals.size(), std::vector<double>(n, 0.0));
    out.J_ineq.assign(base_eval.inequality_residuals.size(), std::vector<double>(n, 0.0));

    if (!base_eval.ok) {
        out.error = "base NLP evaluation is not ok";
        return out;
    }
    if (z.size() != problem.variables.size()) {
        out.error = "finite difference NLP point dimension does not match problem";
        return out;
    }

    const double h_nominal = nlp_fd_step(options.step_fraction);
    constexpr double min_step = 1.0e-8;
    std::vector<ProbeState> probes(n);
    int failed_probe_count = 0;
    int one_sided_fallback_count = 0;
    bool parallel_used = false;
    std::vector<std::string> failure_examples;

    auto record_failure = [&](std::size_t variable, int side, const std::string& error) {
        ++failed_probe_count;
        if (failure_examples.size() >= 4) {
            return;
        }
        std::ostringstream msg;
        msg << "finite difference probe failed for NLP variable " << variable
            << (side > 0 ? " (+): " : " (-): ")
            << (error.empty() ? "evaluation failed" : error);
        if (std::find(failure_examples.begin(), failure_examples.end(), msg.str()) ==
            failure_examples.end()) {
            failure_examples.push_back(msg.str());
        }
    };

    auto bounded_step = [](double room, double requested) {
        if (room <= 1.0e-12) {
            return 0.0;
        }
        return std::min(requested, room);
    };

    auto queue_probe = [&](std::size_t variable,
                           int side,
                           double requested_h,
                           std::vector<ProbeRequest>* requests) {
        ProbeSlot& slot = side > 0 ? probes[variable].plus : probes[variable].minus;
        if (!slot.possible || slot.usable) {
            return;
        }
        const double room = side > 0 ? (1.0 - z[variable]) : (z[variable] + 1.0);
        const double h = bounded_step(room, requested_h);
        if (h <= min_step) {
            return;
        }
        if (slot.h > 0.0 && h >= 0.8 * slot.h) {
            return;
        }
        std::vector<double> point = z;
        point[variable] = std::clamp(point[variable] + static_cast<double>(side) * h,
            -1.0,
            1.0);
        requests->push_back({variable, side, std::abs(point[variable] - z[variable]), std::move(point)});
        slot.h = h;
    };

    auto evaluate_requests = [&](std::vector<ProbeRequest>* requests) {
        if (requests->empty()) {
            return;
        }
        std::vector<std::vector<double>> points;
        points.reserve(requests->size());
        for (const auto& request : *requests) {
            points.push_back(request.point);
        }
        const bool use_parallel =
            options.parallel && evaluator.supports_parallel_probes() && requests->size() > 1;
        parallel_used = parallel_used || use_parallel;
        std::vector<NlpEvalResult> evals =
            evaluator.evaluate_many(problem, points, use_parallel);
        for (std::size_t i = 0; i < requests->size(); ++i) {
            const auto& request = (*requests)[i];
            ProbeSlot& slot =
                request.side > 0 ? probes[request.variable].plus : probes[request.variable].minus;
            std::string error;
            if (i < evals.size() && usable_probe(evals[i], base_eval, &error)) {
                slot.usable = true;
                slot.h = request.h;
                slot.eval = std::move(evals[i]);
            } else {
                record_failure(request.variable, request.side, error);
            }
        }
        requests->clear();
    };

    std::vector<ProbeRequest> requests;
    requests.reserve(2 * n);
    for (std::size_t i = 0; i < n; ++i) {
        const bool plus_possible = (1.0 - z[i]) > min_step;
        const bool minus_possible = (z[i] + 1.0) > min_step;
        probes[i].plus.possible =
            side_requested_initial(options.mode, 1, plus_possible, minus_possible);
        probes[i].minus.possible =
            side_requested_initial(options.mode, -1, plus_possible, minus_possible);
        queue_probe(i, 1, h_nominal, &requests);
        queue_probe(i, -1, h_nominal, &requests);
    }
    evaluate_requests(&requests);

    const double retry_factors[] = {0.25, 0.0625, 0.015625};
    for (const double factor : retry_factors) {
        for (std::size_t i = 0; i < n; ++i) {
            queue_probe(i, 1, h_nominal * factor, &requests);
            queue_probe(i, -1, h_nominal * factor, &requests);
        }
        evaluate_requests(&requests);
    }

    auto write_derivative = [&](std::size_t variable,
                                const ProbeSlot& plus,
                                const ProbeSlot& minus) {
        const bool have_plus = plus.usable && plus.h > min_step;
        const bool have_minus = minus.usable && minus.h > min_step;
        if ((options.mode == FiniteDifferenceMode::Central ||
             options.mode == FiniteDifferenceMode::Auto) &&
            have_plus && have_minus) {
            const double hp = plus.h;
            const double hm = minus.h;
            out.grad_f[variable] = asymmetric_central_derivative(
                base_eval.objective,
                hp,
                hm,
                [&]() { return plus.eval.objective; },
                [&]() { return minus.eval.objective; });
            for (std::size_t k = 0; k < out.J_eq.size(); ++k) {
                out.J_eq[k][variable] = asymmetric_central_derivative(
                    base_eval.equality_residuals[k],
                    hp,
                    hm,
                    [&]() { return plus.eval.equality_residuals[k]; },
                    [&]() { return minus.eval.equality_residuals[k]; });
            }
            for (std::size_t k = 0; k < out.J_ineq.size(); ++k) {
                out.J_ineq[k][variable] = asymmetric_central_derivative(
                    base_eval.inequality_residuals[k],
                    hp,
                    hm,
                    [&]() { return plus.eval.inequality_residuals[k]; },
                    [&]() { return minus.eval.inequality_residuals[k]; });
            }
            return true;
        }
        if (have_plus) {
            const double inv = 1.0 / plus.h;
            out.grad_f[variable] = (plus.eval.objective - base_eval.objective) * inv;
            for (std::size_t k = 0; k < out.J_eq.size(); ++k) {
                out.J_eq[k][variable] =
                    (plus.eval.equality_residuals[k] - base_eval.equality_residuals[k]) * inv;
            }
            for (std::size_t k = 0; k < out.J_ineq.size(); ++k) {
                out.J_ineq[k][variable] =
                    (plus.eval.inequality_residuals[k] - base_eval.inequality_residuals[k]) * inv;
            }
            if (options.mode == FiniteDifferenceMode::Central) {
                ++one_sided_fallback_count;
            }
            return true;
        }
        if (have_minus) {
            const double inv = 1.0 / minus.h;
            out.grad_f[variable] = (base_eval.objective - minus.eval.objective) * inv;
            for (std::size_t k = 0; k < out.J_eq.size(); ++k) {
                out.J_eq[k][variable] =
                    (base_eval.equality_residuals[k] - minus.eval.equality_residuals[k]) * inv;
            }
            for (std::size_t k = 0; k < out.J_ineq.size(); ++k) {
                out.J_ineq[k][variable] =
                    (base_eval.inequality_residuals[k] - minus.eval.inequality_residuals[k]) * inv;
            }
            if (options.mode == FiniteDifferenceMode::Central) {
                ++one_sided_fallback_count;
            }
            return true;
        }
        return false;
    };

    int unresolved = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (!write_derivative(i, probes[i].plus, probes[i].minus)) {
            ++unresolved;
            append_message(
                &out.messages,
                "finite difference failed for NLP variable " + std::to_string(i));
        }
    }

    if (parallel_used) {
        append_message(&out.messages, "finite difference: parallel NLP probes");
    }
    if (failed_probe_count > 0) {
        std::ostringstream msg;
        msg << "finite difference: " << failed_probe_count
            << " NLP probe(s) failed; using available probes";
        append_message(&out.messages, msg.str());
        for (const auto& example : failure_examples) {
            append_message(&out.messages, example);
        }
    }
    if (one_sided_fallback_count > 0) {
        std::ostringstream msg;
        msg << "finite difference: one-sided fallback used for "
            << one_sided_fallback_count << " NLP variable(s)";
        append_message(&out.messages, msg.str());
    }
    if (unresolved > 0) {
        std::ostringstream msg;
        msg << "finite difference: " << unresolved
            << " NLP variable derivative(s) left at zero after probe failures";
        append_message(&out.messages, msg.str());
    }

    out.ok = true;
    return out;
}

} // namespace post2::core
