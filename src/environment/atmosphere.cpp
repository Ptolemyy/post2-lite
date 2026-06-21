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
// Top of the analytic geopotential layers == 86 km geometric altitude.
constexpr double kStdTopGeopotentialM = 84852.0;
constexpr double kStdTopGeometricM = 86000.0;
// USSA76 temperature at 86 km (top of the molecular-scale region). Held constant
// above 86 km so pressure and speed of sound stay continuous and finite; both
// are negligible for thrust/drag at those altitudes (p ~ 0.37 Pa and falling).
constexpr double kTop86kmTemperatureK = 186.946;

// Vallado, "Fundamentals of Astrodynamics and Applications", Table 8-4:
// piecewise-exponential fit to the USSA76 density from 86 km to 1000 km. Each
// band gives rho(h) = base_density * exp(-(h - base_alt_km) / scale_km) for
// h (geometric, km) in [base_alt_km, next band). This is the standard
// approximation used for upper-atmosphere drag / orbital-decay work; without it
// the model would freeze density at the ~86 km value (off by ~5 orders of
// magnitude near 200 km).
struct ExpDensityBand {
    double base_alt_km;
    double base_density_kgpm3;
    double scale_km;
};
constexpr ExpDensityBand kUpperDensityBands[] = {
    {80.0,   1.905e-5,  5.799},
    {90.0,   3.396e-6,  5.382},
    {100.0,  5.297e-7,  5.877},
    {110.0,  9.661e-8,  7.263},
    {120.0,  2.438e-8,  9.473},
    {130.0,  8.484e-9,  12.636},
    {140.0,  3.845e-9,  16.149},
    {150.0,  2.070e-9,  22.523},
    {180.0,  5.464e-10, 29.740},
    {200.0,  2.789e-10, 37.105},
    {250.0,  7.248e-11, 45.546},
    {300.0,  2.418e-11, 53.628},
    {350.0,  9.518e-12, 53.298},
    {400.0,  3.725e-12, 58.515},
    {450.0,  1.585e-12, 60.828},
    {500.0,  6.967e-13, 63.822},
    {600.0,  1.454e-13, 71.835},
    {700.0,  3.614e-14, 88.667},
    {800.0,  1.170e-14, 124.64},
    {900.0,  5.245e-15, 181.05},
    {1000.0, 3.019e-15, 268.00},
};

// Density above 86 km from the piecewise-exponential bands. Extrapolates with
// the top (1000 km) band above 1000 km.
double upper_atmosphere_density(double altitude_km) {
    const ExpDensityBand* band = &kUpperDensityBands[0];
    for (const auto& candidate : kUpperDensityBands) {
        if (altitude_km >= candidate.base_alt_km) {
            band = &candidate;
        }
    }
    return band->base_density_kgpm3 *
           std::exp(-(altitude_km - band->base_alt_km) / band->scale_km);
}

} // namespace

AtmosphereSample us_standard_1976(double altitude_m) {
    const double z = std::max(altitude_m, 0.0);

    // Above 86 km the analytic geopotential layers stop; continue the density
    // decay with the Vallado piecewise-exponential bands instead of freezing at
    // the 86 km value. Pressure/temperature/speed-of-sound there are negligible
    // for the dynamics, so we hold the 86 km boundary temperature for continuity.
    if (z > kStdTopGeometricM) {
        AtmosphereSample sample;
        sample.density_kgpm3 = upper_atmosphere_density(z / 1000.0);
        sample.temperature_k = kTop86kmTemperatureK;
        sample.pressure_pa =
            sample.density_kgpm3 * kSpecificGasConstantJPerKgK * kTop86kmTemperatureK;
        sample.speed_of_sound_mps =
            std::sqrt(kGamma * kSpecificGasConstantJPerKgK * kTop86kmTemperatureK);
        return sample;
    }

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
