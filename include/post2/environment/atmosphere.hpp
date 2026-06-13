#pragma once

#include "post2/core/types.hpp"

namespace post2::environment {

struct AtmosphereSample {
    double density_kgpm3 = 0.0;
    double pressure_pa = 0.0;
    double temperature_k = 0.0;
    double speed_of_sound_mps = 0.0;
    post2::core::Vec3 wind_ecef_mps;
};

class IAtmosphereModel {
public:
    virtual ~IAtmosphereModel() = default;

    virtual AtmosphereSample sample(
        double time_s,
        const post2::core::Vec3& position_eci_m,
        const post2::core::Vec3& velocity_eci_mps) const = 0;
};

class ExponentialAtmosphereModel final : public IAtmosphereModel {
public:
    explicit ExponentialAtmosphereModel(
        double earth_radius_m = post2::core::kEarthRadiusM,
        double rho0_kgpm3 = 1.225,
        double scale_height_m = 7200.0);

    AtmosphereSample sample(
        double time_s,
        const post2::core::Vec3& position_eci_m,
        const post2::core::Vec3& velocity_eci_mps) const override;

private:
    double earth_radius_m_ = post2::core::kEarthRadiusM;
    double rho0_kgpm3_ = 1.225;
    double scale_height_m_ = 7200.0;
};

} // namespace post2::environment
