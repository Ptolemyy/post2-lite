#pragma once

#include <memory>
#include <string>

#include "post2/core/types.hpp"

namespace post2::core {

enum class CoreMode {
    Local,
    Remote,
};

const char* core_mode_name(CoreMode mode);
bool parse_core_mode(const std::string& text, CoreMode* mode);

class ITrajectoryService {
public:
    virtual ~ITrajectoryService() = default;
    virtual SimulationResult simulate(const SimulationConfig& config) = 0;
    virtual SimulationResult simulate(const CaseConfig& config) = 0;
};

class LocalTrajectoryService final : public ITrajectoryService {
public:
    SimulationResult simulate(const SimulationConfig& config) override;
    SimulationResult simulate(const CaseConfig& config) override;
};

class RemoteTrajectoryService final : public ITrajectoryService {
public:
    RemoteTrajectoryService(std::string host, int port);
    SimulationResult simulate(const SimulationConfig& config) override;
    SimulationResult simulate(const CaseConfig& config) override;

private:
    std::string host_;
    int port_;
};

std::unique_ptr<ITrajectoryService> make_trajectory_service(CoreMode mode, const std::string& host, int port);

std::string make_remote_request(const SimulationConfig& config);
bool parse_remote_request(const std::string& request, SimulationConfig* config, std::string* error);
std::string make_remote_request(const CaseConfig& config);
bool parse_remote_request(const std::string& request, CaseConfig* config, std::string* error);

} // namespace post2::core
