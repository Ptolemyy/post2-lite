#pragma once

#include <memory>
#include <vector>

#include "post2/propagation/force_model.hpp"

namespace post2::propagation {

class ForceModelSet {
public:
    void add(std::unique_ptr<IForceModel> model);

    ForceModelOutput evaluate_all(
        const ForceModelContext& context,
        const post2::integrators::ExtendedState& state) const;

private:
    std::vector<std::unique_ptr<IForceModel>> models_;
};

ForceModelSet make_force_model_set(
    const post2::core::CaseConfig& case_config,
    const post2::core::PhaseConfig& phase);

} // namespace post2::propagation
