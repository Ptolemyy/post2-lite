#include "post2/environment/atmosphere.hpp"

#include "post2/vehicle/vehicle.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace post2::environment {

namespace {

constexpr double kTemperatureK = 288.15;
constexpr double kSpecificGasConstantJPerKgK = 287.05287;
constexpr double kGamma = 1.4;

// U.S. Standard Atmosphere 1976 geopotential layer base data (0..86 km).
constexpr double kGeopotentialEarthRadiusM = 6356766.0;
constexpr double kGravity0 = 9.80665;
constexpr double kMolarMass = 0.0289644;     // kg/mol
constexpr double kUniversalGasR = 8.31432;   // J/(mol.K)

struct StdAtmoLayer {
    double base_geopotential_m;
    double base_temperature_k;
    double lapse_rate_k_per_m;
    double base_pressure_pa;
};

constexpr StdAtmoLayer kStdLayers[] = {
    {0.0,     288.15, -0.0065, 101325.0},
    {11000.0, 216.65, 0.0,     22632.06},
    {20000.0, 216.65, 0.001,   5474.889},
    {32000.0, 228.65, 0.0028,  868.0187},
    {47000.0, 270.65, 0.0,     110.9063},
    {51000.0, 270.65, -0.0028, 66.93887},
    {71000.0, 214.65, -0.002,  3.956420},
};
constexpr double kStdTopGeopotentialM = 84852.0;

} // namespace

AtmosphereSample us_standard_1976(double altitude_m) {
    const double z = std::max(altitude_m, 0.0);
    // Geometric -> geopotential altitude.
    double h = kGeopotentialEarthRadiusM * z / (kGeopotentialEarthRadiusM + z);
    h = std::min(h, kStdTopGeopotentialM);

    const double gMR = kGravity0 * kMolarMass / kUniversalGasR;

    // Select the layer containing h.
    int layer = 0;
    const int n = static_cast<int>(sizeof(kStdLayers) / sizeof(kStdLayers[0]));
    for (int i = 0; i < n; ++i) {
        if (h >= kStdLayers[i].base_geopotential_m) {
            layer = i;
        }
    }
    const StdAtmoLayer& lr = kStdLayers[layer];
    const double dh = h - lr.base_geopotential_m;
    double temperature = lr.base_temperature_k + lr.lapse_rate_k_per_m * dh;
    if (temperature < 1.0) {
        temperature = 1.0;
    }
    double pressure;
    if (std::fabs(lr.lapse_rate_k_per_m) > 1e-12) {
        pressure = lr.base_pressure_pa *
                   std::pow(lr.base_temperature_k / temperature, gMR / lr.lapse_rate_k_per_m);
    } else {
        pressure = lr.base_pressure_pa * std::exp(-gMR * dh / lr.base_temperature_k);
    }

    AtmosphereSample sample;
    sample.temperature_k = temperature;
    sample.pressure_pa = pressure;
    sample.density_kgpm3 = pressure / (kSpecificGasConstantJPerKgK * temperature);
    sample.speed_of_sound_mps = std::sqrt(kGamma * kSpecificGasConstantJPerKgK * temperature);
    return sample;
}

double air_dynamic_viscosity_sutherland(double temperature_k) {
    const double t = std::max(temperature_k, 1.0);
    return 1.458e-6 * std::pow(t, 1.5) / (t + 110.4);
}

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

UsStandardAtmosphere1976Model::UsStandardAtmosphere1976Model(double earth_radius_m)
    : earth_radius_m_(earth_radius_m) {}

AtmosphereSample UsStandardAtmosphere1976Model::sample(
    double time_s,
    const post2::core::Vec3& position_eci_m,
    const post2::core::Vec3& velocity_eci_mps) const {
    (void)time_s;
    (void)velocity_eci_mps;
    const double altitude_m = post2::vehicle::norm(position_eci_m) - earth_radius_m_;
    return us_standard_1976(altitude_m);
}

TabulatedAtmosphereModel::TabulatedAtmosphereModel(double earth_radius_m,
                                                   std::vector<double> altitude_m,
                                                   std::vector<AtmosphereSample> samples)
    : earth_radius_m_(earth_radius_m)
    , altitude_m_(std::move(altitude_m))
    , samples_(std::move(samples)) {}

bool TabulatedAtmosphereModel::load_csv(const std::string& path,
                                        double earth_radius_m,
                                        TabulatedAtmosphereModel* model,
                                        std::string* error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (error) {
            *error = "could not open atmosphere table: " + path;
        }
        return false;
    }
    std::vector<double> alt;
    std::vector<AtmosphereSample> samples;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        // Skip a header row (non-numeric first token).
        if (!(std::isdigit(static_cast<unsigned char>(line[0])) || line[0] == '-' ||
              line[0] == '+' || line[0] == '.')) {
            continue;
        }
        std::stringstream ss(line);
        std::string tok;
        double vals[5] = {0, 0, 0, 0, -1.0};
        int n = 0;
        while (n < 5 && std::getline(ss, tok, ',')) {
            vals[n++] = std::strtod(tok.c_str(), nullptr);
        }
        if (n < 4) {
            continue;
        }
        AtmosphereSample s;
        s.density_kgpm3 = vals[1];
        s.pressure_pa = vals[2];
        s.temperature_k = vals[3];
        s.speed_of_sound_mps = (n >= 5 && vals[4] > 0.0)
            ? vals[4]
            : std::sqrt(kGamma * kSpecificGasConstantJPerKgK * std::max(vals[3], 1.0));
        alt.push_back(vals[0]);
        samples.push_back(s);
    }
    if (alt.size() < 2) {
        if (error) {
            *error = "atmosphere table needs >= 2 rows: " + path;
        }
        return false;
    }
    *model = TabulatedAtmosphereModel(earth_radius_m, std::move(alt), std::move(samples));
    return true;
}

AtmosphereSample TabulatedAtmosphereModel::sample(
    double time_s,
    const post2::core::Vec3& position_eci_m,
    const post2::core::Vec3& velocity_eci_mps) const {
    (void)time_s;
    (void)velocity_eci_mps;
    AtmosphereSample result;
    if (altitude_m_.empty()) {
        return result;
    }
    const double altitude_m = post2::vehicle::norm(position_eci_m) - earth_radius_m_;
    if (altitude_m <= altitude_m_.front()) {
        return samples_.front();
    }
    if (altitude_m >= altitude_m_.back()) {
        return samples_.back();
    }
    const auto it = std::upper_bound(altitude_m_.begin(), altitude_m_.end(), altitude_m);
    const std::size_t hi = static_cast<std::size_t>(it - altitude_m_.begin());
    const std::size_t lo = hi - 1;
    const double span = altitude_m_[hi] - altitude_m_[lo];
    const double t = span > 0.0 ? (altitude_m - altitude_m_[lo]) / span : 0.0;
    const AtmosphereSample& a = samples_[lo];
    const AtmosphereSample& b = samples_[hi];
    result.density_kgpm3 = a.density_kgpm3 + (b.density_kgpm3 - a.density_kgpm3) * t;
    result.pressure_pa = a.pressure_pa + (b.pressure_pa - a.pressure_pa) * t;
    result.temperature_k = a.temperature_k + (b.temperature_k - a.temperature_k) * t;
    result.speed_of_sound_mps =
        a.speed_of_sound_mps + (b.speed_of_sound_mps - a.speed_of_sound_mps) * t;
    return result;
}

} // namespace post2::environment
