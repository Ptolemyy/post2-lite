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
    phase.termination.value = 1.0;
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

post2::core::CaseConfig make_payload_drop_case()
{
    post2::core::CaseConfig config = make_drop_case();
    post2::vehicle::StageConfig bus;
    bus.name = "bus";
    bus.dry_mass_kg = 1000.0;
    bus.active = false;
    bus.attached = true;
    post2::vehicle::StageConfig payload;
    payload.name = "payload";
    payload.dry_mass_kg = 100.0;
    payload.active = false;
    payload.attached = true;
    config.vehicle.stages = {bus, payload};
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
        post2::core::CaseConfig objective_case = make_payload_drop_case();
        objective_case.optimization.mode = "optimize";
        post2::core::OptimizationObjectiveConfig payload;
        payload.enabled = true;
        payload.metric = "payload_mass_kg";
        payload.direction = "maximize";
        payload.weight = 2.0;
        post2::core::OptimizationObjectiveConfig altitude;
        altitude.enabled = true;
        altitude.metric = "terminal_altitude_m";
        altitude.direction = "minimize";
        altitude.weight = 0.5;
        objective_case.optimization.objectives = {payload, altitude};

        post2::core::NlpProblem problem;
        std::string error;
        if (!post2::core::build_nlp_problem_from_case(objective_case, &problem, &error)) {
            std::cerr << "multi-objective NLP build failed: " << error << '\n';
            return 1;
        }
        post2::core::NlpEvaluator evaluator(objective_case, service);
        const auto eval = evaluator.evaluate(problem, {});
        double payload_mass = 0.0;
        double terminal_altitude = 0.0;
        if (!eval.ok ||
            !post2::core::evaluate_trajectory_metric(
                eval.simulation.state_log,
                objective_case,
                "payload_mass_kg",
                &payload_mass) ||
            !post2::core::evaluate_trajectory_metric(
                eval.simulation.state_log,
                objective_case,
                "terminal_altitude_m",
                &terminal_altitude)) {
            std::cerr << "multi-objective evaluation failed\n";
            return 1;
        }
        const double expected =
            -2.0 * payload_mass / post2::core::nlp_default_metric_scale("payload_mass_kg") +
            0.5 * terminal_altitude / post2::core::nlp_default_metric_scale("terminal_altitude_m");
        if (!near(eval.objective, expected, 1.0e-9) || problem.objectives.size() != 2) {
            std::cerr << "multi-objective weighted sum mismatch\n";
            return 1;
        }
    }

    {
        drop_case.optimization.qp_solver = "active-set";
        drop_case.optimization.fd_mode = "central";
        drop_case.optimization.parallel_fd = false;
        drop_case.optimization.constraint_tolerance = 0.01;
        drop_case.optimization.stationarity_tolerance = 0.02;
        drop_case.optimization.max_restoration_iterations = 3;
        drop_case.optimization.continuation.enabled = true;
        drop_case.optimization.continuation.variable_path = "phases[0].termination.value";
        drop_case.optimization.continuation.direction = "decrease";
        drop_case.optimization.continuation.steps = 5;
        drop_case.optimization.continuation.multistart_enabled = true;
        drop_case.optimization.continuation.multistart_count = 3;
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
            !parsed.optimization.continuation.enabled ||
            parsed.optimization.continuation.variable_path != "phases[0].termination.value" ||
            parsed.optimization.continuation.direction != "decrease" ||
            parsed.optimization.continuation.steps != 5 ||
            !parsed.optimization.continuation.multistart_enabled ||
            parsed.optimization.continuation.multistart_count != 3 ||
            parsed.optimization.targets.back().scope != "terminal") {
            std::cerr << "optimization JSON round trip failed: " << error << '\n';
            return 1;
        }
    }

    {
        post2::core::CaseConfig target_case = make_drop_case();
        target_case.phases[0].termination.value = 2.0;
        const auto target_sim = service.simulate(target_case);
        if (!target_sim.ok) {
            std::cerr << "envelope target simulation failed: " << target_sim.error << '\n';
            return 1;
        }

        post2::core::CaseConfig envelope_case = make_drop_case();
        envelope_case.optimization.optimizer = "sqp";
        envelope_case.optimization.max_iterations = 1;
        envelope_case.optimization.tolerance = 1.0e-6;
        envelope_case.optimization.constraint_tolerance = 1.0e-6;
        envelope_case.optimization.variables.push_back({"phases[0].termination.value", true, 0.5, 2.0});
        envelope_case.optimization.targets.push_back({
            "terminal_altitude_m",
            "equal",
            target_sim.state_log.back().altitude_m,
            0.0,
            0.0,
            1.0,
        });
        envelope_case.optimization.envelope_search.enabled = true;
        envelope_case.optimization.envelope_search.sample_count = 4;
        const auto result = post2::core::optimize_case(&envelope_case, service);
        double duration = 0.0;
        if (!result.found_feasible ||
            !has_message(result.messages, "envelope: enabled") ||
            !has_message(result.messages, "envelope: sampled 4 candidates") ||
            !post2::core::read_optimization_variable(
                envelope_case,
                "phases[0].termination.value",
                &duration,
                nullptr) ||
            duration < 1.9) {
            std::cerr << "envelope warm start did not select the expected candidate\n";
            return 1;
        }
    }

    {
        post2::core::CaseConfig nlp_case = make_drop_case();
        nlp_case.optimization.variables.push_back({"phases[0].termination.value", true, 0.5, 2.0});
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
        nlp_case.optimization.variables.push_back({"phases[0].termination.value", true, 0.5, 2.0});
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
        sqp_case.optimization.variables.push_back({"phases[0].termination.value", true, 0.5, 2.0});
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
        active_set_case.optimization.variables.push_back({"phases[0].termination.value", true, 0.5, 2.0});
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
        fmincon_case.optimization.variables.push_back({"phases[0].termination.value", true, 0.5, 2.0});
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
        post2::core::CaseConfig continuation_case = make_payload_drop_case();
        const auto baseline = service.simulate(continuation_case);
        if (!baseline.ok) {
            std::cerr << "payload continuation baseline simulation failed: " << baseline.error << '\n';
            return 1;
        }
        continuation_case.optimization.optimizer = "sqp";
        continuation_case.optimization.max_iterations = 2;
        continuation_case.optimization.tolerance = 0.02;
        continuation_case.optimization.constraint_tolerance = 0.02;
        continuation_case.optimization.mode = "optimize";
        continuation_case.optimization.variables.push_back({"phases[0].termination.value", true, 0.5, 2.0});
        continuation_case.optimization.variables.push_back({"vehicle.stages[1].dry_mass_kg", true, 100.0, 180.0});
        continuation_case.optimization.targets.push_back({
            "terminal_altitude_m",
            "equal",
            baseline.state_log.back().altitude_m,
            0.0,
            0.0,
            1.0,
        });
        continuation_case.optimization.objective.enabled = true;
        continuation_case.optimization.objective.metric = "payload_mass_kg";
        continuation_case.optimization.objective.direction = "maximize";
        continuation_case.optimization.continuation.enabled = true;
        continuation_case.optimization.continuation.variable_path = "vehicle.stages[1].dry_mass_kg";
        continuation_case.optimization.continuation.direction = "increase";
        continuation_case.optimization.continuation.steps = 2;
        continuation_case.optimization.continuation.multistart_enabled = true;
        continuation_case.optimization.continuation.multistart_count = 2;

        const auto result = post2::core::optimize_case(&continuation_case, service);
        double payload_mass = 0.0;
        if (!result.ok ||
            !result.found_feasible ||
            !has_message(result.messages, "continuation: enabled") ||
            !post2::core::read_optimization_variable(
                continuation_case,
                "vehicle.stages[1].dry_mass_kg",
                &payload_mass,
                nullptr) ||
            payload_mass < 179.0 ||
            result.variable_changes.size() < 2) {
            std::cerr << "payload continuation did not reach the upper bound\n";
            return 1;
        }
    }

    {
        post2::core::CaseConfig continuation_case = make_drop_case();
        const auto baseline = service.simulate(continuation_case);
        if (!baseline.ok) {
            std::cerr << "non-payload continuation baseline simulation failed: " << baseline.error << '\n';
            return 1;
        }
        continuation_case.optimization.optimizer = "fmincon";
        continuation_case.optimization.max_iterations = 2;
        continuation_case.optimization.tolerance = 0.02;
        continuation_case.optimization.constraint_tolerance = 0.02;
        continuation_case.optimization.variables.push_back({"phases[0].termination.value", true, 0.5, 2.0});
        continuation_case.optimization.variables.push_back({"vehicle.dry_mass_kg", true, 500.0, 1000.0});
        continuation_case.optimization.targets.push_back({
            "terminal_altitude_m",
            "equal",
            baseline.state_log.back().altitude_m,
            0.0,
            0.0,
            1.0,
        });
        continuation_case.optimization.continuation.enabled = true;
        continuation_case.optimization.continuation.variable_path = "vehicle.dry_mass_kg";
        continuation_case.optimization.continuation.direction = "decrease";
        continuation_case.optimization.continuation.steps = 2;

        const auto result = post2::core::optimize_case(&continuation_case, service);
        double dry_mass = 0.0;
        if (!result.ok ||
            !result.found_feasible ||
            !post2::core::read_optimization_variable(
                continuation_case,
                "vehicle.dry_mass_kg",
                &dry_mass,
                nullptr) ||
            dry_mass > 501.0 ||
            result.variable_changes.size() < 2) {
            std::cerr << "non-payload decreasing continuation did not reach the lower bound\n";
            return 1;
        }
    }

    {
        post2::core::CaseConfig invalid_continuation = make_drop_case();
        invalid_continuation.optimization.variables.push_back({"phases[0].termination.value", true, 0.5, 2.0});
        invalid_continuation.optimization.targets.push_back({"terminal_altitude_m", "equal", 995.0, 0.0, 0.0, 1.0});
        invalid_continuation.optimization.continuation.enabled = true;
        invalid_continuation.optimization.continuation.variable_path = "vehicle.dry_mass_kg";
        const auto result = post2::core::optimize_case(&invalid_continuation, service);
        if (result.ok ||
            result.error.find("continuation variable must be an enabled optimization variable") ==
                std::string::npos) {
            std::cerr << "invalid continuation variable did not fail clearly\n";
            return 1;
        }
    }

    {
        post2::core::CaseConfig impossible = make_drop_case();
        impossible.optimization.optimizer = "sqp";
        impossible.optimization.max_iterations = 2;
        impossible.optimization.tolerance = 1.0e-8;
        impossible.optimization.variables.push_back({"phases[0].termination.value", true, 0.5, 1.0});
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
