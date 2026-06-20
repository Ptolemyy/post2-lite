#pragma once

#include <string>
#include <vector>

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

// U.S. Standard Atmosphere 1976, valid 0..86 km geometric altitude. Returns
// density, pressure, temperature and speed of sound at the given geometric
// altitude. Above 86 km the lowest-density layer value is clamped (near-vacuum).
AtmosphereSample us_standard_1976(double altitude_m);

// Dynamic viscosity of air via Sutherland's law [Pa.s], for Reynolds numbers.
double air_dynamic_viscosity_sutherland(double temperature_k);

class UsStandardAtmosphere1976Model final : public IAtmosphereModel {
public:
    explicit UsStandardAtmosphere1976Model(double earth_radius_m = post2::core::kEarthRadiusM);

    AtmosphereSample sample(
        double time_s,
        const post2::core::Vec3& position_eci_m,
        const post2::core::Vec3& velocity_eci_mps) const override;

private:
    double earth_radius_m_ = post2::core::kEarthRadiusM;
};

// Atmosphere read from a CSV table: "altitude_m,density_kgpm3,pressure_pa,
// temperature_k[,speed_of_sound_mps]" with ascending altitude. Missing speed of
// sound is derived from temperature. Linearly interpolated, clamped at the ends.
class TabulatedAtmosphereModel final : public IAtmosphereModel {
public:
    TabulatedAtmosphereModel(double earth_radius_m,
                             std::vector<double> altitude_m,
                             std::vector<AtmosphereSample> samples);

    static bool load_csv(const std::string& path,
                         double earth_radius_m,
                         TabulatedAtmosphereModel* model,
                         std::string* error);

    bool empty() const { return altitude_m_.empty(); }

    AtmosphereSample sample(
        double time_s,
        const post2::core::Vec3& position_eci_m,
        const post2::core::Vec3& velocity_eci_mps) const override;

private:
    double earth_radius_m_ = post2::core::kEarthRadiusM;
    std::vector<double> altitude_m_;
    std::vector<AtmosphereSample> samples_;
};

} // namespace post2::environment
