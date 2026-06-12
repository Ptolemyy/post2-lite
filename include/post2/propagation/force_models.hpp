#pragma once

#include "post2/core/types.hpp"

namespace post2::propagation {

post2::core::Vec3 gravity_acceleration_mps2(
    const post2::core::SimulationConfig& config,
    const post2::core::Vec3& position_m);

post2::core::Vec3 surface_normal_acceleration_mps2(
    const post2::core::SimulationConfig& config,
    const post2::core::State& state);

post2::core::State apply_surface_contact_constraint(
    const post2::core::SimulationConfig& config,
    const post2::core::State& state);

} // namespace post2::propagation
