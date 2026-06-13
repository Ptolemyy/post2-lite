#include "post2/core/qp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace post2::core {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

double clamp(double value, double lower, double upper)
{
    return std::min(std::max(value, lower), upper);
}

double dot(const std::vector<double>& a, const std::vector<double>& b)
{
    double out = 0.0;
    const std::size_t n = std::min(a.size(), b.size());
    for (std::size_t i = 0; i < n; ++i) {
        out += a[i] * b[i];
    }
    return out;
}

double norm_inf(const std::vector<double>& values)
{
    double out = 0.0;
    for (const double v : values) {
        out = std::max(out, std::abs(v));
    }
    return out;
}

bool solve_linear_system(
    std::vector<std::vector<double>> A,
    std::vector<double> b,
    std::vector<double>* x)
{
    const std::size_t n = b.size();
    if (A.size() != n) {
        return false;
    }
    if (n == 0) {
        x->clear();
        return true;
    }
    for (const auto& row : A) {
        if (row.size() != n) {
            return false;
        }
    }

    std::vector<double> row_scale(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            row_scale[i] = std::max(row_scale[i], std::abs(A[i][j]));
        }
        if (row_scale[i] == 0.0) {
            row_scale[i] = 1.0;
        }
    }

    for (std::size_t col = 0; col < n; ++col) {
        std::size_t pivot = col;
        double best = std::abs(A[col][col]) / row_scale[col];
        for (std::size_t row = col + 1; row < n; ++row) {
            const double candidate = std::abs(A[row][col]) / row_scale[row];
            if (candidate > best) {
                best = candidate;
                pivot = row;
            }
        }
        if (best < 1.0e-13) {
            return false;
        }
        if (pivot != col) {
            std::swap(A[pivot], A[col]);
            std::swap(b[pivot], b[col]);
            std::swap(row_scale[pivot], row_scale[col]);
        }
        const double diag = A[col][col];
        for (std::size_t j = col; j < n; ++j) {
            A[col][j] /= diag;
        }
        b[col] /= diag;
        for (std::size_t row = 0; row < n; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = A[row][col];
            if (std::abs(factor) <= 0.0) {
                continue;
            }
            for (std::size_t j = col; j < n; ++j) {
                A[row][j] -= factor * A[col][j];
            }
            b[row] -= factor * b[col];
        }
    }
    *x = std::move(b);
    return true;
}

std::vector<std::vector<double>> symmetrized_hessian(const QpProblem& problem)
{
    const std::size_t n = problem.g.size();
    std::vector<std::vector<double>> H(n, std::vector<double>(n, 0.0));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            const double hij = i < problem.H.size() && j < problem.H[i].size()
                ? problem.H[i][j]
                : 0.0;
            const double hji = j < problem.H.size() && i < problem.H[j].size()
                ? problem.H[j][i]
                : 0.0;
            H[i][j] = 0.5 * (hij + hji);
        }
    }
    return H;
}

bool validate_qp_dimensions(const QpProblem& problem, std::string* error)
{
    const std::size_t n = problem.g.size();
    if (problem.H.size() != n) {
        if (error) {
            *error = "qp: Hessian row count does not match gradient size";
        }
        return false;
    }
    for (const auto& row : problem.H) {
        if (row.size() != n) {
            if (error) {
                *error = "qp: Hessian is not square";
            }
            return false;
        }
    }
    if (!problem.lower.empty() && problem.lower.size() != n) {
        if (error) {
            *error = "qp: lower bound size does not match gradient size";
        }
        return false;
    }
    if (!problem.upper.empty() && problem.upper.size() != n) {
        if (error) {
            *error = "qp: upper bound size does not match gradient size";
        }
        return false;
    }
    if (problem.Aeq.size() != problem.beq.size()) {
        if (error) {
            *error = "qp: equality matrix/vector size mismatch";
        }
        return false;
    }
    if (problem.Aineq.size() != problem.bineq.size()) {
        if (error) {
            *error = "qp: inequality matrix/vector size mismatch";
        }
        return false;
    }
    for (const auto& row : problem.Aeq) {
        if (row.size() != n) {
            if (error) {
                *error = "qp: equality row width does not match gradient size";
            }
            return false;
        }
    }
    for (const auto& row : problem.Aineq) {
        if (row.size() != n) {
            if (error) {
                *error = "qp: inequality row width does not match gradient size";
            }
            return false;
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        const double lo = problem.lower.empty() ? -kInf : problem.lower[i];
        const double hi = problem.upper.empty() ? kInf : problem.upper[i];
        if (lo > hi) {
            if (error) {
                *error = "qp: lower bound is greater than upper bound";
            }
            return false;
        }
    }
    return true;
}

bool solve_active_kkt(
    const QpProblem& problem,
    const std::vector<std::vector<double>>& H,
    const std::vector<std::vector<double>>& A,
    const std::vector<double>& b,
    double regularization,
    std::vector<double>* step,
    std::vector<double>* multipliers)
{
    const std::size_t n = problem.g.size();
    const std::size_t m = b.size();
    const std::size_t total = n + m;
    std::vector<std::vector<double>> K(total, std::vector<double>(total, 0.0));
    std::vector<double> rhs(total, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            K[i][j] = H[i][j];
        }
        K[i][i] += regularization;
        rhs[i] = -problem.g[i];
    }
    for (std::size_t r = 0; r < m; ++r) {
        if (A[r].size() != n) {
            return false;
        }
        for (std::size_t c = 0; c < n; ++c) {
            K[n + r][c] = A[r][c];
            K[c][n + r] = A[r][c];
        }
        rhs[n + r] = b[r];
    }

    std::vector<double> sol;
    if (!solve_linear_system(std::move(K), std::move(rhs), &sol)) {
        return false;
    }
    step->assign(sol.begin(), sol.begin() + static_cast<std::ptrdiff_t>(n));
    multipliers->assign(sol.begin() + static_cast<std::ptrdiff_t>(n), sol.end());
    return true;
}

bool solve_active_kkt_regularized(
    const QpProblem& problem,
    const std::vector<std::vector<double>>& H,
    const std::vector<std::vector<double>>& A,
    const std::vector<double>& b,
    std::vector<double>* step,
    std::vector<double>* multipliers)
{
    double scale = 1.0;
    for (const auto& row : H) {
        scale = std::max(scale, norm_inf(row));
    }
    for (int attempt = 0; attempt < 8; ++attempt) {
        const double reg = scale * std::pow(10.0, -12.0 + attempt);
        if (solve_active_kkt(problem, H, A, b, reg, step, multipliers)) {
            return true;
        }
    }
    return false;
}

double inequality_residual(
    const std::vector<double>& row,
    double bound,
    const std::vector<double>& step)
{
    return dot(row, step) - bound;
}

struct InternalInequality {
    std::vector<double> row;
    double bound = 0.0;
    int result_index = -1;
    int active_code = -1;
};

std::vector<InternalInequality> collect_inequalities(const QpProblem& problem)
{
    const std::size_t n = problem.g.size();
    std::vector<InternalInequality> out;
    out.reserve(problem.Aineq.size() + 2 * n);
    for (std::size_t i = 0; i < problem.Aineq.size(); ++i) {
        out.push_back({
            problem.Aineq[i],
            problem.bineq[i],
            static_cast<int>(i),
            static_cast<int>(i),
        });
    }
    for (std::size_t i = 0; i < n; ++i) {
        const double lo = problem.lower.empty() ? -kInf : problem.lower[i];
        const double hi = problem.upper.empty() ? kInf : problem.upper[i];
        if (std::isfinite(lo)) {
            std::vector<double> row(n, 0.0);
            row[i] = -1.0;
            out.push_back({
                std::move(row),
                -lo,
                -1,
                static_cast<int>(problem.Aineq.size() + 2 * i),
            });
        }
        if (std::isfinite(hi)) {
            std::vector<double> row(n, 0.0);
            row[i] = 1.0;
            out.push_back({
                std::move(row),
                hi,
                -1,
                static_cast<int>(problem.Aineq.size() + 2 * i + 1),
            });
        }
    }
    return out;
}

enum class RowKind {
    Equality,
    Inequality,
};

struct ActiveRowInfo {
    RowKind kind = RowKind::Equality;
    int index = -1;
};

struct RowDependency {
    bool dependent = false;
    bool consistent = false;
};

RowDependency row_dependency(
    const std::vector<std::vector<double>>& existing_rows,
    const std::vector<double>& existing_bounds,
    const std::vector<double>& candidate,
    double candidate_bound)
{
    const double row_norm = std::max(1.0, std::sqrt(dot(candidate, candidate)));
    if (existing_rows.empty()) {
        return {false, true};
    }

    const std::size_t m = existing_rows.size();
    std::vector<std::vector<double>> G(m, std::vector<double>(m, 0.0));
    std::vector<double> rhs(m, 0.0);
    for (std::size_t i = 0; i < m; ++i) {
        rhs[i] = dot(existing_rows[i], candidate);
        for (std::size_t j = 0; j < m; ++j) {
            G[i][j] = dot(existing_rows[i], existing_rows[j]);
        }
    }
    std::vector<double> coeffs;
    if (!solve_linear_system(G, rhs, &coeffs)) {
        double scale = 1.0;
        for (const auto& row : G) {
            scale = std::max(scale, norm_inf(row));
        }
        bool solved = false;
        for (int attempt = 0; attempt < 6 && !solved; ++attempt) {
            std::vector<std::vector<double>> regularized = G;
            const double reg = scale * std::pow(10.0, -12.0 + attempt);
            for (std::size_t i = 0; i < regularized.size(); ++i) {
                regularized[i][i] += reg;
            }
            solved = solve_linear_system(std::move(regularized), rhs, &coeffs);
        }
        if (!solved) {
            return {true, false};
        }
    }

    std::vector<double> residual(candidate.size(), 0.0);
    for (std::size_t i = 0; i < candidate.size(); ++i) {
        residual[i] = candidate[i];
    }
    double bound_combo = 0.0;
    for (std::size_t r = 0; r < m; ++r) {
        bound_combo += coeffs[r] * existing_bounds[r];
        for (std::size_t c = 0; c < candidate.size(); ++c) {
            residual[c] -= coeffs[r] * existing_rows[r][c];
        }
    }
    const double residual_norm = std::sqrt(dot(residual, residual));
    const bool dependent = residual_norm <= 1.0e-9 * row_norm;
    const double bound_scale = std::max({1.0, std::abs(bound_combo), std::abs(candidate_bound)});
    const bool consistent = std::abs(bound_combo - candidate_bound) <= 1.0e-8 * bound_scale;
    return {dependent, consistent};
}

bool append_independent_row(
    const std::vector<double>& row,
    double bound,
    ActiveRowInfo info,
    std::vector<std::vector<double>>* rows,
    std::vector<double>* bounds,
    std::vector<ActiveRowInfo>* infos,
    std::string* error)
{
    const double row_norm = std::sqrt(dot(row, row));
    if (row_norm <= 1.0e-14) {
        if (std::abs(bound) <= 1.0e-10) {
            return true;
        }
        if (error) {
            *error = "qp: zero active constraint row has nonzero bound";
        }
        return false;
    }
    const RowDependency dep = row_dependency(*rows, *bounds, row, bound);
    if (dep.dependent) {
        if (dep.consistent) {
            return true;
        }
        if (error) {
            *error = "qp: active constraints are inconsistent";
        }
        return false;
    }
    rows->push_back(row);
    bounds->push_back(bound);
    infos->push_back(info);
    return true;
}

bool build_active_rows(
    const QpProblem& problem,
    const std::vector<InternalInequality>& inequalities,
    const std::vector<int>& active,
    std::vector<std::vector<double>>* rows,
    std::vector<double>* bounds,
    std::vector<ActiveRowInfo>* infos,
    std::string* error)
{
    rows->clear();
    bounds->clear();
    infos->clear();
    rows->reserve(problem.Aeq.size() + active.size());
    bounds->reserve(problem.beq.size() + active.size());
    infos->reserve(problem.beq.size() + active.size());

    for (std::size_t i = 0; i < problem.Aeq.size(); ++i) {
        if (!append_independent_row(
                problem.Aeq[i],
                problem.beq[i],
                {RowKind::Equality, static_cast<int>(i)},
                rows,
                bounds,
                infos,
                error)) {
            return false;
        }
    }
    for (const int id : active) {
        if (id < 0 || static_cast<std::size_t>(id) >= inequalities.size()) {
            if (error) {
                *error = "qp: invalid active-set index";
            }
            return false;
        }
        const auto& c = inequalities[static_cast<std::size_t>(id)];
        if (!append_independent_row(
                c.row,
                c.bound,
                {RowKind::Inequality, id},
                rows,
                bounds,
                infos,
                error)) {
            return false;
        }
    }
    return true;
}

std::vector<double> full_multipliers(
    const QpProblem& problem,
    const std::vector<InternalInequality>& inequalities,
    const std::vector<ActiveRowInfo>& infos,
    const std::vector<double>& reduced_multipliers)
{
    std::vector<double> out(problem.Aeq.size() + problem.Aineq.size(), 0.0);
    const std::size_t count = std::min(infos.size(), reduced_multipliers.size());
    for (std::size_t i = 0; i < count; ++i) {
        const auto& info = infos[i];
        if (info.kind == RowKind::Equality) {
            if (info.index >= 0 && static_cast<std::size_t>(info.index) < problem.Aeq.size()) {
                out[static_cast<std::size_t>(info.index)] = reduced_multipliers[i];
            }
        } else if (info.index >= 0 &&
                   static_cast<std::size_t>(info.index) < inequalities.size()) {
            const int result_index =
                inequalities[static_cast<std::size_t>(info.index)].result_index;
            if (result_index >= 0 &&
                static_cast<std::size_t>(result_index) < problem.Aineq.size()) {
                out[problem.Aeq.size() + static_cast<std::size_t>(result_index)] =
                    reduced_multipliers[i];
            }
        }
    }
    return out;
}

void mark_active_codes(
    const std::vector<InternalInequality>& inequalities,
    const std::vector<int>& active,
    std::vector<int>* active_set)
{
    active_set->clear();
    active_set->reserve(active.size());
    for (const int id : active) {
        if (id >= 0 && static_cast<std::size_t>(id) < inequalities.size()) {
            active_set->push_back(inequalities[static_cast<std::size_t>(id)].active_code);
        }
    }
}

void fail_active_set(QpResult* result, std::string error)
{
    result->error = std::move(error);
    if (result->error.empty()) {
        result->error = "qp: active-set failed";
    }
    result->messages.push_back(result->error);
}

} // namespace

QpResult KktFallbackDenseQpSolver::solve(const QpProblem& problem)
{
    QpResult result;
    result.messages.push_back("qp: using kkt-fallback");
    std::string validation_error;
    if (!validate_qp_dimensions(problem, &validation_error)) {
        result.error = validation_error;
        return result;
    }

    const std::size_t n = problem.g.size();
    result.multipliers.assign(problem.Aeq.size() + problem.Aineq.size(), 0.0);
    if (n == 0) {
        result.ok = true;
        return result;
    }

    const std::vector<std::vector<double>> H = symmetrized_hessian(problem);
    std::vector<int> active_ineq;
    std::vector<double> best_step(n, 0.0);
    std::vector<double> best_lambda;
    bool solved = false;

    for (std::size_t attempt = 0; attempt <= problem.Aineq.size(); ++attempt) {
        std::vector<std::vector<double>> A = problem.Aeq;
        std::vector<double> b = problem.beq;
        for (const int idx : active_ineq) {
            A.push_back(problem.Aineq[static_cast<std::size_t>(idx)]);
            b.push_back(problem.bineq[static_cast<std::size_t>(idx)]);
        }

        std::vector<double> step;
        std::vector<double> multipliers;
        if (!solve_active_kkt_regularized(problem, H, A, b, &step, &multipliers)) {
            result.error = "qp: kkt fallback system is singular";
            return result;
        }
        for (std::size_t i = 0; i < n; ++i) {
            const double lower = problem.lower.empty() ? -kInf : problem.lower[i];
            const double upper = problem.upper.empty() ? kInf : problem.upper[i];
            step[i] = clamp(step[i], lower, upper);
        }

        solved = true;
        best_step = step;
        best_lambda = multipliers;

        double worst = 1.0e-9;
        int worst_index = -1;
        for (std::size_t i = 0; i < problem.Aineq.size(); ++i) {
            if (std::find(active_ineq.begin(), active_ineq.end(), static_cast<int>(i)) != active_ineq.end()) {
                continue;
            }
            const double residual = inequality_residual(problem.Aineq[i], problem.bineq[i], step);
            if (residual > worst) {
                worst = residual;
                worst_index = static_cast<int>(i);
            }
        }
        if (worst_index < 0) {
            break;
        }
        active_ineq.push_back(worst_index);
    }

    if (!solved) {
        result.error = "qp: fallback failed";
        return result;
    }
    result.ok = true;
    result.step = std::move(best_step);
    result.active_set = active_ineq;
    result.multipliers.assign(problem.Aeq.size() + problem.Aineq.size(), 0.0);
    const std::size_t eq_count = std::min(problem.Aeq.size(), best_lambda.size());
    for (std::size_t i = 0; i < eq_count; ++i) {
        result.multipliers[i] = best_lambda[i];
    }
    for (std::size_t k = 0; k < active_ineq.size(); ++k) {
        const std::size_t lambda_index = problem.Aeq.size() + k;
        if (lambda_index >= best_lambda.size()) {
            continue;
        }
        const std::size_t ineq_index = static_cast<std::size_t>(active_ineq[k]);
        result.multipliers[problem.Aeq.size() + ineq_index] = best_lambda[lambda_index];
    }
    return result;
}

QpResult ActiveSetDenseQpSolver::solve(const QpProblem& problem)
{
    QpResult result;
    result.messages.push_back("qp: using active-set");
    std::string validation_error;
    if (!validate_qp_dimensions(problem, &validation_error)) {
        fail_active_set(&result, validation_error);
        return result;
    }

    const std::size_t n = problem.g.size();
    result.multipliers.assign(problem.Aeq.size() + problem.Aineq.size(), 0.0);
    if (n == 0) {
        result.ok = true;
        return result;
    }

    const std::vector<std::vector<double>> H = symmetrized_hessian(problem);
    const std::vector<InternalInequality> inequalities = collect_inequalities(problem);
    std::vector<int> active;
    std::vector<double> x(n, 0.0);
    std::vector<double> reduced_lambda;
    std::vector<ActiveRowInfo> infos;

    const int max_iterations = std::max(20, static_cast<int>(
        4 * (n + problem.Aeq.size() + inequalities.size() + 1) *
        (n + problem.Aeq.size() + inequalities.size() + 1)));
    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<std::vector<double>> A;
        std::vector<double> b;
        std::string active_error;
        if (!build_active_rows(problem, inequalities, active, &A, &b, &infos, &active_error)) {
            fail_active_set(&result, active_error);
            return result;
        }
        if (!solve_active_kkt_regularized(problem, H, A, b, &x, &reduced_lambda)) {
            fail_active_set(&result, "qp: active-set KKT system is singular");
            return result;
        }

        double worst = 1.0e-8;
        int worst_id = -1;
        for (std::size_t i = 0; i < inequalities.size(); ++i) {
            const int id = static_cast<int>(i);
            if (std::find(active.begin(), active.end(), id) != active.end()) {
                continue;
            }
            const double residual =
                inequality_residual(inequalities[i].row, inequalities[i].bound, x);
            if (residual > worst) {
                worst = residual;
                worst_id = id;
            }
        }
        if (worst_id >= 0) {
            active.push_back(worst_id);
            continue;
        }

        std::vector<double> lambda_full =
            full_multipliers(problem, inequalities, infos, reduced_lambda);
        std::vector<double> lambda_internal(inequalities.size(), 0.0);
        const std::size_t info_count = std::min(infos.size(), reduced_lambda.size());
        for (std::size_t i = 0; i < info_count; ++i) {
            if (infos[i].kind == RowKind::Inequality &&
                infos[i].index >= 0 &&
                static_cast<std::size_t>(infos[i].index) < inequalities.size()) {
                lambda_internal[static_cast<std::size_t>(infos[i].index)] =
                    reduced_lambda[i];
            }
        }

        double most_negative = -1.0e-8;
        int drop_id = -1;
        for (const int id : active) {
            if (id < 0 || static_cast<std::size_t>(id) >= lambda_internal.size()) {
                continue;
            }
            const double lambda = lambda_internal[static_cast<std::size_t>(id)];
            if (lambda < most_negative) {
                most_negative = lambda;
                drop_id = id;
            }
        }
        if (drop_id >= 0) {
            active.erase(std::remove(active.begin(), active.end(), drop_id), active.end());
            continue;
        }

        result.ok = true;
        result.step = std::move(x);
        result.multipliers = std::move(lambda_full);
        mark_active_codes(inequalities, active, &result.active_set);
        return result;
    }

    std::ostringstream msg;
    msg << "qp: active-set exceeded iteration limit (" << max_iterations << ")";
    fail_active_set(&result, msg.str());
    return result;
}

std::unique_ptr<IDenseQpSolver> make_dense_qp_solver(const std::string& name)
{
    if (name == "active-set") {
        return std::make_unique<ActiveSetDenseQpSolver>();
    }
    return std::make_unique<KktFallbackDenseQpSolver>();
}

} // namespace post2::core
