#include "post2/core/case_config_io.hpp"
#include "post2/core/finite_difference.hpp"
#include "post2/core/nlp_evaluator.hpp"
#include "post2/core/nlp_problem.hpp"
#include "post2/core/optimization.hpp"
#include "post2/core/optimizer.hpp"
#include "post2/core/qp_solver.hpp"
#include "post2/core/trajectory_service.hpp"
#include "post2/vehicle/vehicle.hpp"
#include "post2/vehicle/vehicle_config_io.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

bool near(double lhs, double rhs, double tolerance)
{
    return std::abs(lhs - rhs) <= tolerance;
}

bool has_message(
    const std::vector<std::string>& messages,
    const std::string& expected)
{
    return std::find(messages.begin(), messages.end(), expected) != messages.end();
}

post2::core::CaseConfig make_drop_case()
{
    post2::core::CaseConfig config;
    config.step_s = 0.5;
    config.vehicle = post2::vehicle::default_vehicle_config();
    config.vehicle.dry_mass_kg = 1000.0;
    post2::core::PhaseConfig phase;
    phase.name = "drop";
    phase.duration_s = 1.0;
    phase.inherit_initial_state = false;
    phase.initial_state_eci = post2::core::State{
        {post2::core::kEarthRadiusM + 1000.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
    };
    phase.force_models.thrust = false;
    phase.force_models.normal_force = false;
    config.phases = {phase};
    return config;
}

} // namespace

int main()
{
    {
        post2::core::NlpVariable variable;
        variable.lower = 10.0;
        variable.upper = 20.0;
        if (!near(variable.to_scaled(10.0), -1.0, 1.0e-12) ||
            !near(variable.to_scaled(15.0), 0.0, 1.0e-12) ||
            !near(variable.from_scaled(1.0), 20.0, 1.0e-12) ||
            !near(variable.clamp_real(25.0), 20.0, 1.0e-12)) {
            std::cerr << "NLP scaling failed\n";
            return 1;
        }
    }

    {
        const std::vector<double> x = {0.7};
        auto f = [](const std::vector<double>& value, double* out, std::string*) {
            *out = std::sin(value[0]);
            return true;
        };
        post2::core::FiniteDifferenceOptions forward;
        forward.mode = post2::core::FiniteDifferenceMode::Forward;
        forward.step_fraction = 1.0e-3;
        post2::core::FiniteDifferenceOptions central = forward;
        central.mode = post2::core::FiniteDifferenceMode::Central;
        const auto gf = post2::core::finite_difference_gradient(x, f, forward);
        const auto gc = post2::core::finite_difference_gradient(x, f, central);
        const double exact = std::cos(x[0]);
        if (!gf.ok || !gc.ok ||
            std::abs(gc.gradient[0] - exact) >= std::abs(gf.gradient[0] - exact)) {
            std::cerr << "finite difference accuracy failed\n";
            return 1;
        }
    }

    {
        post2::core::KktFallbackDenseQpSolver solver;
        post2::core::QpProblem qp;
        qp.H = {{2.0}};
        qp.g = {-4.0};
        qp.lower = {-10.0};
        qp.upper = {10.0};
        auto result = solver.solve(qp);
        if (!result.ok || !near(result.step[0], 2.0, 1.0e-8)) {
            std::cerr << "unconstrained QP failed\n";
            return 1;
        }
        qp.Aeq = {{1.0}};
        qp.beq = {1.0};
        result = solver.solve(qp);
        if (!result.ok || !near(result.step[0], 1.0, 1.0e-8)) {
            std::cerr << "equality QP failed\n";
            return 1;
        }
        qp.Aeq.clear();
        qp.beq.clear();
        qp.Aineq = {{-1.0}};
        qp.bineq = {-3.0};
        result = solver.solve(qp);
        if (!result.ok || !near(result.step[0], 3.0, 1.0e-8) || result.active_set.empty()) {
            std::cerr << "inequality QP failed\n";
            return 1;
        }
    }

    {
        auto solver = post2::core::make_dense_qp_solver("active-set");
        post2::core::QpProblem qp;
        qp.H = {{2.0}};
        qp.g = {-4.0};
        auto result = solver->solve(qp);
        if (!result.ok ||
            !has_message(result.messages, "qp: using active-set") ||
            !near(result.step[0], 2.0, 1.0e-8)) {
            std::cerr << "active-set unconstrained QP failed\n";
            return 1;
        }

        qp.H = {{2.0, 0.0}, {0.0, 2.0}};
        qp.g = {-4.0, -2.0};
        qp.Aeq = {{1.0, 1.0}};
        qp.beq = {1.0};
        result = solver->solve(qp);
        if (!result.ok ||
            !near(result.step[0], 1.0, 1.0e-8) ||
            !near(result.step[1], 0.0, 1.0e-8)) {
            std::cerr << "active-set equality QP failed\n";
            return 1;
        }

        qp = post2::core::QpProblem{};
        qp.H = {{2.0}};
        qp.g = {-4.0};
        qp.Aineq = {{1.0}};
        qp.bineq = {1.0};
        result = solver->solve(qp);
        if (!result.ok ||
            !near(result.step[0], 1.0, 1.0e-8) ||
            result.active_set.empty() ||
            result.multipliers.size() != 1 ||
            result.multipliers[0] <= 0.0) {
            std::cerr << "active-set inequality QP failed\n";
            return 1;
        }

        qp = post2::core::QpProblem{};
        qp.H = {{2.0}};
        qp.g = {-6.0};
        qp.lower = {-10.0};
        qp.upper = {1.0};
        result = solver->solve(qp);
        if (!result.ok || !near(result.step[0], 1.0, 1.0e-8)) {
            std::cerr << "active-set box QP failed\n";
            return 1;
        }

        qp.lower = {2.0};
        qp.upper = {1.0};
        result = solver->solve(qp);
        if (result.ok || result.error.empty()) {
            std::cerr << "active-set infeasible QP did not fail clearly\n";
            return 1;
        }
    }

    post2::core::LocalTrajectoryService service;
    post2::core::CaseConfig drop_case = make_drop_case();
    const auto simulation = service.simulate(drop_case);
    if (!simulation.ok) {
        std::cerr << "drop simulation failed: " << simulation.error << '\n';
        return 1;
    }
    double max_accel = -1.0;
    double propellant = -1.0;
    if (!post2::core::evaluate_trajectory_metric(
            simulation.state_log, drop_case, "max_accel_mps2", &max_accel) ||
        !post2::core::evaluate_trajectory_metric(
            simulation.state_log, drop_case, "propellant_remaining_kg", &propellant) ||
        max_accel < 0.0 ||
        !std::isfinite(propellant)) {
        std::cerr << "extended metric extraction failed\n";
        return 1;
    }

    {
        drop_case.optimization.qp_solver = "active-set";
        drop_case.optimization.fd_mode = "central";
        drop_case.optimization.parallel_fd = false;
        drop_case.optimization.constraint_tolerance = 0.01;
        drop_case.optimization.stationarity_tolerance = 0.02;
        drop_case.optimization.max_restoration_iterations = 3;
        drop_case.optimization.targets.push_back({
            "terminal_altitude_m",
            "upper",
            0.0,
            0.0,
            1200.0,
            1.0,
            "terminal",
            -1,
        });
        const std::string json = post2::core::case_config_to_json(drop_case);
        post2::core::CaseConfig parsed;
        std::string error;
        if (!post2::core::case_config_from_json(json, &parsed, &error) ||
            parsed.optimization.qp_solver != "active-set" ||
            parsed.optimization.fd_mode != "central" ||
            parsed.optimization.parallel_fd ||
            !near(parsed.optimization.constraint_tolerance, 0.01, 1.0e-12) ||
            parsed.optimization.max_restoration_iterations != 3 ||
            parsed.optimization.targets.back().scope != "terminal") {
            std::cerr << "optimization JSON round trip failed: " << error << '\n';
            return 1;
        }
    }

    {
        post2::core::CaseConfig nlp_case = make_drop_case();
        nlp_case.optimization.variables.push_back({"phases[0].duration_s", true, 0.5, 2.0});
        nlp_case.optimization.targets.push_back({"terminal_altitude_m", "equal", 950.0, 0.0, 0.0, 1.0});
        post2::core::NlpProblem problem;
        std::string error;
        if (!post2::core::build_nlp_problem_from_case(nlp_case, &problem, &error)) {
            std::cerr << "NLP build failed: " << error << '\n';
            return 1;
        }
        post2::core::NlpEvaluator evaluator(nlp_case, service);
        post2::core::DenseSqpOptimizer optimizer;
        post2::core::OptimizerOptions options;
        options.max_iterations = 4;
        options.constraint_tolerance = 1.0e-8;
        options.finite_difference.mode = post2::core::FiniteDifferenceMode::Auto;
        const auto result = optimizer.solve(problem, evaluator, options);
        if (result.best_z.empty() || result.messages.empty()) {
            std::cerr << "optimizer interface did not report a candidate\n";
            return 1;
        }
    }

    {
        post2::core::CaseConfig nlp_case = make_drop_case();
        nlp_case.optimization.variables.push_back({"phases[0].duration_s", true, 0.5, 2.0});
        nlp_case.optimization.targets.push_back({"terminal_altitude_m", "equal", 950.0, 0.0, 0.0, 1.0});
        post2::core::NlpProblem problem;
        std::string error;
        if (!post2::core::build_nlp_problem_from_case(nlp_case, &problem, &error)) {
            std::cerr << "NLP build for FD failed: " << error << '\n';
            return 1;
        }
        const std::vector<double> z = post2::core::make_initial_nlp_point(problem).z;
        post2::core::NlpEvaluator serial_evaluator(nlp_case, service);
        const auto base_serial = serial_evaluator.evaluate(problem, z);
        post2::core::FiniteDifferenceOptions serial_options;
        serial_options.mode = post2::core::FiniteDifferenceMode::Central;
        serial_options.step_fraction = 1.0e-3;
        serial_options.parallel = false;
        const auto serial_fd = post2::core::finite_difference_nlp(
            problem, serial_evaluator, z, base_serial, serial_options);

        post2::core::NlpEvaluator parallel_evaluator(nlp_case, service);
        const auto base_parallel = parallel_evaluator.evaluate(problem, z);
        post2::core::FiniteDifferenceOptions parallel_options = serial_options;
        parallel_options.parallel = true;
        const auto parallel_fd = post2::core::finite_difference_nlp(
            problem, parallel_evaluator, z, base_parallel, parallel_options);
        if (!serial_fd.ok || !parallel_fd.ok ||
            serial_fd.J_eq.empty() ||
            parallel_fd.J_eq.empty() ||
            !near(serial_fd.J_eq[0][0], parallel_fd.J_eq[0][0], 1.0e-6) ||
            !has_message(parallel_fd.messages, "finite difference: parallel NLP probes")) {
            std::cerr << "parallel NLP finite difference mismatch\n";
            return 1;
        }
    }

    {
        post2::core::CaseConfig sqp_case = make_drop_case();
        sqp_case.optimization.optimizer = "sqp";
        sqp_case.optimization.max_iterations = 2;
        sqp_case.optimization.variables.push_back({"phases[0].duration_s", true, 0.5, 2.0});
        sqp_case.optimization.targets.push_back({"terminal_altitude_m", "equal", 950.0, 0.0, 0.0, 1.0});
        const auto result = post2::core::optimize_case(&sqp_case, service);
        if (!has_message(result.messages, "nlp: problem built") ||
            !has_message(result.messages, "evaluator: trajectory service") ||
            !has_message(result.messages, "optimizer: sqp") ||
            !has_message(result.messages, "qp: using kkt-fallback") ||
            result.variable_changes.empty()) {
            std::cerr << "public SQP NLP pipeline messages failed\n";
            return 1;
        }
    }

    {
        post2::core::CaseConfig active_set_case = make_drop_case();
        active_set_case.optimization.optimizer = "sqp";
        active_set_case.optimization.qp_solver = "active-set";
        active_set_case.optimization.max_iterations = 1;
        active_set_case.optimization.variables.push_back({"phases[0].duration_s", true, 0.5, 2.0});
        active_set_case.optimization.targets.push_back({"terminal_altitude_m", "equal", 950.0, 0.0, 0.0, 1.0});
        const auto result = post2::core::optimize_case(&active_set_case, service);
        if (!has_message(result.messages, "qp: using active-set") ||
            result.variable_changes.empty()) {
            std::cerr << "active-set solver message failed\n";
            return 1;
        }
    }

    {
        post2::core::CaseConfig fmincon_case = make_drop_case();
        fmincon_case.optimization.optimizer = "fmincon";
        fmincon_case.optimization.max_iterations = 2;
        fmincon_case.optimization.variables.push_back({"phases[0].duration_s", true, 0.5, 2.0});
        fmincon_case.optimization.targets.push_back({"terminal_altitude_m", "equal", 950.0, 0.0, 0.0, 1.0});
        const auto result = post2::core::optimize_case(&fmincon_case, service);
        if (result.variable_changes.empty() ||
            result.final_metrics.empty() ||
            !has_message(result.messages, "optimizer: fmincon")) {
            std::cerr << "public fmincon pipeline failed\n";
            return 1;
        }
    }

    {
        post2::core::CaseConfig impossible = make_drop_case();
        impossible.optimization.optimizer = "sqp";
        impossible.optimization.max_iterations = 2;
        impossible.optimization.tolerance = 1.0e-8;
        impossible.optimization.variables.push_back({"phases[0].duration_s", true, 0.5, 1.0});
        impossible.optimization.targets.push_back({"terminal_altitude_m", "equal", 1.0e9, 0.0, 0.0, 1.0});
        const auto result = post2::core::optimize_case(&impossible, service);
        if (result.ok || result.found_feasible || result.max_constraint_violation <= 0.0 ||
            result.l1_constraint_violation <= 0.0 ||
            result.variable_changes.empty() ||
            result.final_metrics.empty() ||
            result.messages.empty()) {
            std::cerr << "infeasible optimize report failed\n";
            return 1;
        }
    }

    return 0;
}
