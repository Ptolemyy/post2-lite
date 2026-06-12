#include "post2/core/trajectory_service.hpp"

#include "post2/core/simulation_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <memory>
#include <string>

namespace post2::core {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

std::string lowercase(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

} // namespace

double circular_orbit_speed_mps(double mu_m3s2, double radius_m)
{
    return std::sqrt(mu_m3s2 / radius_m);
}

State make_default_leo_state(const SimulationConfig& config)
{
    const double radius_m = config.earth_radius_m + config.initial_altitude_m;
    const double speed_mps = config.initial_speed_mps > 0.0
        ? config.initial_speed_mps
        : circular_orbit_speed_mps(config.earth_mu_m3s2, radius_m);
    const double inclination_rad = config.inclination_deg * kPi / 180.0;

    return {
        {radius_m, 0.0, 0.0},
        {0.0, speed_mps * std::cos(inclination_rad), speed_mps * std::sin(inclination_rad)},
    };
}

const char* core_mode_name(CoreMode mode)
{
    switch (mode) {
    case CoreMode::Local:
        return "local";
    case CoreMode::Remote:
        return "remote";
    }
    return "unknown";
}

bool parse_core_mode(const std::string& text, CoreMode* mode)
{
    const std::string normalized = lowercase(text);
    if (normalized == "local") {
        *mode = CoreMode::Local;
        return true;
    }
    if (normalized == "remote") {
        *mode = CoreMode::Remote;
        return true;
    }
    return false;
}

SimulationResult LocalTrajectoryService::simulate(const SimulationConfig& config)
{
    SimulationDriver driver;
    return driver.run(config);
}

SimulationResult LocalTrajectoryService::simulate(const CaseConfig& config)
{
    SimulationDriver driver;
    return driver.run(config);
}

std::unique_ptr<ITrajectoryService> make_trajectory_service(CoreMode mode, const std::string& host, int port)
{
    if (mode == CoreMode::Remote) {
        return std::make_unique<RemoteTrajectoryService>(host, port);
    }
    return std::make_unique<LocalTrajectoryService>();
}

} // namespace post2::core
