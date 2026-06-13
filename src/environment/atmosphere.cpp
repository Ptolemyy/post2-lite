#include "post2/environment/atmosphere.hpp"

#include "post2/vehicle/vehicle.hpp"

#include <cmath>

namespace post2::environment {

namespace {

constexpr double kTemperatureK = 288.15;
constexpr double kSpecificGasConstantJPerKgK = 287.05287;
constexpr double kGamma = 1.4;

} // namespace

ExponentialAtmosphereModel::ExponentialAtmosphereModel(
    double earth_radius_m,
    double rho0_kgpm3,
    double scale_height_m)
    : earth_radius_m_(earth_radius_m)
    , rho0_kgpm3_(rho0_kgpm3)
    , scale_height_m_(scale_height_m)
{
}

AtmosphereSample ExponentialAtmosphereModel::sample(
    double time_s,
    const post2::core::Vec3& position_eci_m,
    const post2::core::Vec3& velocity_eci_mps) const
{
    (void)time_s;
    (void)velocity_eci_mps;

    AtmosphereSample sample;
    const double radius_m = post2::vehicle::norm(position_eci_m);
    const double altitude_m = radius_m - earth_radius_m_;
    sample.density_kgpm3 = rho0_kgpm3_ * std::exp(-altitude_m / scale_height_m_);
    sample.temperature_k = kTemperatureK;
    sample.pressure_pa =
        sample.density_kgpm3 * kSpecificGasConstantJPerKgK * sample.temperature_k;
    sample.speed_of_sound_mps =
        std::sqrt(kGamma * kSpecificGasConstantJPerKgK * sample.temperature_k);
    return sample;
}

} // namespace post2::environment
