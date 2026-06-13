#pragma once

#include "post2/core/types.hpp"
#include "post2/integrators/ode_integrator.hpp"
#include "post2/vehicle/runtime_state.hpp"

namespace post2::propagation {

struct EnvironmentState {
    double time_s = 0.0;
    post2::core::Vec3 position_eci_m;
    post2::core::Vec3 velocity_eci_mps;

    post2::core::Vec3 position_ecef_m;
    post2::core::Vec3 velocity_ecef_mps;

    double radius_m = 0.0;
    double altitude_m = 0.0;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;

    double density_kgpm3 = 0.0;
    double pressure_pa = 0.0;
    double temperature_k = 0.0;
    double speed_of_sound_mps = 0.0;

    post2::core::Vec3 wind_ecef_mps;
};

struct ForceModelContext {
    const post2::core::CaseConfig* case_config = nullptr;
    const post2::core::PhaseConfig* phase = nullptr;
    const post2::vehicle::VehicleRuntimeState* runtime = nullptr;
    const EnvironmentState* environment = nullptr;
    post2::core::Vec3 thrust_acceleration_eci_mps2;
};

struct ForceModelOutput {
    post2::core::Vec3 acceleration_eci_mps2;
};

class IForceModel {
public:
    virtual ~IForceModel() = default;

    virtual ForceModelOutput evaluate(
        const ForceModelContext& context,
        const post2::integrators::ExtendedState& state) const = 0;
};

class PointMassGravityModel final : public IForceModel {
public:
    ForceModelOutput evaluate(
        const ForceModelContext& context,
        const post2::integrators::ExtendedState& state) const override;
};

class J2GravityModel final : public IForceModel {
public:
    ForceModelOutput evaluate(
        const ForceModelContext& context,
        const post2::integrators::ExtendedState& state) const override;
};

class SphericalHarmonicGravityModel final : public IForceModel {
public:
    ForceModelOutput evaluate(
        const ForceModelContext& context,
        const post2::integrators::ExtendedState& state) const override;
};

class AtmosphericDragModel final : public IForceModel {
public:
    ForceModelOutput evaluate(
        const ForceModelContext& context,
        const post2::integrators::ExtendedState& state) const override;
};

class LiftAeroModel final : public IForceModel {
public:
    ForceModelOutput evaluate(
        const ForceModelContext& context,
        const post2::integrators::ExtendedState& state) const override;
};

class ThrustForceModel final : public IForceModel {
public:
    ForceModelOutput evaluate(
        const ForceModelContext& context,
        const post2::integrators::ExtendedState& state) const override;
};

class SurfaceContactModel final : public IForceModel {
public:
    ForceModelOutput evaluate(
        const ForceModelContext& context,
        const post2::integrators::ExtendedState& state) const override;
};

class SolarRadiationPressureModel final : public IForceModel {
public:
    ForceModelOutput evaluate(
        const ForceModelContext& context,
        const post2::integrators::ExtendedState& state) const override;
};

class ThirdBodyGravityModel final : public IForceModel {
public:
    ForceModelOutput evaluate(
        const ForceModelContext& context,
        const post2::integrators::ExtendedState& state) const override;
};

} // namespace post2::propagation
