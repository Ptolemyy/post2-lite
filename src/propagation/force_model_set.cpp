#include "post2/propagation/force_model_set.hpp"

#include <stdexcept>
#include <utility>

namespace post2::propagation {

void ForceModelSet::add(std::unique_ptr<IForceModel> model)
{
    if (model) {
        models_.push_back(std::move(model));
    }
}

ForceModelOutput ForceModelSet::evaluate_all(
    const ForceModelContext& context,
    const post2::integrators::ExtendedState& state) const
{
    ForceModelOutput total;
    for (const auto& model : models_) {
        const ForceModelOutput output = model->evaluate(context, state);
        total.acceleration_eci_mps2 =
            total.acceleration_eci_mps2 + output.acceleration_eci_mps2;
    }
    return total;
}

ForceModelSet make_force_model_set(
    const post2::core::CaseConfig& case_config,
    const post2::core::PhaseConfig& phase)
{
    (void)case_config;

    ForceModelSet set;
    const post2::core::ForceModelSwitches& switches = phase.force_models;

    if (switches.gravity) {
        if (switches.gravity_model.type == "point_mass") {
            set.add(std::make_unique<PointMassGravityModel>());
        } else if (switches.gravity_model.type == "j2") {
            set.add(std::make_unique<J2GravityModel>());
        } else if (switches.gravity_model.type == "spherical_harmonic") {
            set.add(std::make_unique<SphericalHarmonicGravityModel>());
        } else {
            throw std::runtime_error(
                "unsupported gravity model type: " + switches.gravity_model.type);
        }
    }

    if (switches.thrust) {
        set.add(std::make_unique<ThrustForceModel>());
    }
    if (switches.normal_force) {
        set.add(std::make_unique<SurfaceContactModel>());
    }
    if (switches.aerodynamic) {
        set.add(std::make_unique<AtmosphericDragModel>());
        set.add(std::make_unique<LiftAeroModel>());
    }
    if (switches.third_body) {
        set.add(std::make_unique<ThirdBodyGravityModel>());
    }
    if (switches.solar_radiation_pressure) {
        set.add(std::make_unique<SolarRadiationPressureModel>());
    }

    return set;
}

} // namespace post2::propagation
