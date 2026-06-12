#pragma once

#include <string>
#include <vector>

#include "post2/core/trajectory_service.hpp"
#include "post2/core/types.hpp"

namespace post2::core {

std::vector<OptimizationMetricValue> evaluate_trajectory_metrics(
    const StateLog& state_log,
    const CaseConfig& config);

bool read_optimization_variable(
    const CaseConfig& config,
    const std::string& path,
    double* value,
    std::string* error);

bool write_optimization_variable(
    CaseConfig* config,
    const std::string& path,
    double value,
    std::string* error);

OptimizationResult optimize_case(
    CaseConfig* config,
    ITrajectoryService& service,
    OptimizationRunOptions options = {});

} // namespace post2::core
