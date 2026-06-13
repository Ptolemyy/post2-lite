#pragma once

#include <memory>
#include <string>
#include <vector>

namespace post2::core {

struct QpProblem {
    std::vector<std::vector<double>> H;
    std::vector<double> g;
    std::vector<double> lower;
    std::vector<double> upper;
    std::vector<std::vector<double>> Aeq;
    std::vector<double> beq;
    std::vector<std::vector<double>> Aineq;
    std::vector<double> bineq;
};

struct QpResult {
    bool ok = false;
    std::string error;
    std::vector<double> step;
    std::vector<double> multipliers;
    std::vector<int> active_set;
    std::vector<std::string> messages;
};

class IDenseQpSolver {
public:
    virtual ~IDenseQpSolver() = default;
    virtual QpResult solve(const QpProblem& problem) = 0;
};

class KktFallbackDenseQpSolver final : public IDenseQpSolver {
public:
    QpResult solve(const QpProblem& problem) override;
};

class ActiveSetDenseQpSolver final : public IDenseQpSolver {
public:
    QpResult solve(const QpProblem& problem) override;
};

std::unique_ptr<IDenseQpSolver> make_dense_qp_solver(const std::string& name);

} // namespace post2::core
