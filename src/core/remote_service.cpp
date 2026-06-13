#include "post2/core/trajectory_service.hpp"

#include "post2/core/case_config_io.hpp"
#include "post2/core/io.hpp"
#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle_config_io.hpp"

#include <cstring>
#include <exception>
#include <iomanip>
#include <sstream>
#include <string>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace post2::core {

namespace {

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;

class SocketRuntime {
public:
    SocketRuntime()
    {
        WSADATA data;
        ok_ = WSAStartup(MAKEWORD(2, 2), &data) == 0;
    }

    ~SocketRuntime()
    {
        if (ok_) {
            WSACleanup();
        }
    }

    bool ok() const { return ok_; }

private:
    bool ok_ = false;
};

void close_socket(SocketHandle socket)
{
    closesocket(socket);
}

std::string last_socket_error()
{
    return "winsock error " + std::to_string(WSAGetLastError());
}
#else
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;

class SocketRuntime {
public:
    bool ok() const { return true; }
};

void close_socket(SocketHandle socket)
{
    close(socket);
}

std::string last_socket_error()
{
    return std::strerror(errno);
}
#endif

bool send_all(SocketHandle socket, const std::string& data, std::string* error)
{
    const char* cursor = data.data();
    std::size_t remaining = data.size();
    while (remaining > 0) {
#ifdef _WIN32
        const int sent = send(socket, cursor, static_cast<int>(remaining), 0);
#else
        const ssize_t sent = send(socket, cursor, remaining, 0);
#endif
        if (sent <= 0) {
            if (error) {
                *error = "send failed: " + last_socket_error();
            }
            return false;
        }
        cursor += sent;
        remaining -= static_cast<std::size_t>(sent);
    }
    return true;
}

SimulationResult receive_csv_response(SocketHandle socket)
{
    std::string response;
    char buffer[4096];
    for (;;) {
#ifdef _WIN32
        const int received = recv(socket, buffer, static_cast<int>(sizeof(buffer)), 0);
#else
        const ssize_t received = recv(socket, buffer, sizeof(buffer), 0);
#endif
        if (received < 0) {
            return {false, "receive failed: " + last_socket_error(), {}};
        }
        if (received == 0) {
            break;
        }
        response.append(buffer, buffer + received);
    }

    if (response.rfind("ERR ", 0) == 0) {
        return {false, response.substr(4), {}};
    }

    return trajectory_from_csv(response);
}

SocketHandle connect_to_server(const std::string& host, int port, std::string* error)
{
    addrinfo hints = {};
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_family = AF_UNSPEC;

    addrinfo* result = nullptr;
    const std::string port_text = std::to_string(port);
    const int rc = getaddrinfo(host.c_str(), port_text.c_str(), &hints, &result);
    if (rc != 0) {
        if (error) {
#ifdef _WIN32
            *error = "getaddrinfo failed: " + std::to_string(rc);
#else
            *error = "getaddrinfo failed: " + std::string(gai_strerror(rc));
#endif
        }
        return kInvalidSocket;
    }

    SocketHandle socket = kInvalidSocket;
    for (addrinfo* ptr = result; ptr != nullptr; ptr = ptr->ai_next) {
        socket = ::socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
        if (socket == kInvalidSocket) {
            continue;
        }
        if (connect(socket, ptr->ai_addr, static_cast<int>(ptr->ai_addrlen)) == 0) {
            break;
        }
        close_socket(socket);
        socket = kInvalidSocket;
    }

    freeaddrinfo(result);

    if (socket == kInvalidSocket && error) {
        *error = "connect failed: " + last_socket_error();
    }
    return socket;
}

} // namespace

RemoteTrajectoryService::RemoteTrajectoryService(std::string host, int port)
    : host_(std::move(host))
    , port_(port)
{
}

SimulationResult RemoteTrajectoryService::simulate(const SimulationConfig& config)
{
    SocketRuntime runtime;
    if (!runtime.ok()) {
        return {false, "failed to initialize socket runtime", {}};
    }

    std::string error;
    const SocketHandle socket = connect_to_server(host_, port_, &error);
    if (socket == kInvalidSocket) {
        return {false, error, {}};
    }

    const std::string request = make_remote_request(config);
    if (!send_all(socket, request, &error)) {
        close_socket(socket);
        return {false, error, {}};
    }

#ifdef _WIN32
    shutdown(socket, SD_SEND);
#else
    shutdown(socket, SHUT_WR);
#endif

    SimulationResult result = receive_csv_response(socket);
    close_socket(socket);
    return result;
}

SimulationResult RemoteTrajectoryService::simulate(const CaseConfig& config)
{
    SocketRuntime runtime;
    if (!runtime.ok()) {
        return {false, "failed to initialize socket runtime", {}};
    }

    std::string error;
    const SocketHandle socket = connect_to_server(host_, port_, &error);
    if (socket == kInvalidSocket) {
        return {false, error, {}};
    }

    const std::string request = make_remote_request(config);
    if (!send_all(socket, request, &error)) {
        close_socket(socket);
        return {false, error, {}};
    }

#ifdef _WIN32
    shutdown(socket, SD_SEND);
#else
    shutdown(socket, SHUT_WR);
#endif

    SimulationResult result = receive_csv_response(socket);
    close_socket(socket);
    return result;
}

std::string make_remote_request(const SimulationConfig& config)
{
    post2::vehicle::VehicleConfig vehicle = config.vehicle;
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&vehicle);
    std::ostringstream output;
    output << std::setprecision(17)
           << "SIMV4 "
           << config.duration_s << ' '
           << config.step_s << ' '
           << config.initial_altitude_m << ' '
           << config.inclination_deg << ' '
           << config.initial_speed_mps << ' '
           << config.launch_site.latitude_deg << ' '
           << config.launch_site.longitude_deg << ' '
           << config.launch_site.altitude_m << ' '
           << (config.hold_down_clamp.enabled ? 1 : 0) << ' '
           << config.hold_down_clamp.release_time_s << ' '
           << (config.normal_force.enabled ? 1 : 0) << ' '
           << std::quoted(vehicle.name) << ' '
           << vehicle.dry_mass_kg << ' '
           << (vehicle.aero.enabled ? 1 : 0) << ' '
           << vehicle.aero.reference_area_m2 << ' '
           << vehicle.aero.cd << ' '
           << vehicle.aero.cl << ' '
           << std::quoted(vehicle.aero.aero_table_path) << ' '
           << (vehicle.engine.enabled ? 1 : 0) << ' '
           << vehicle.engine.max_thrust_n << ' '
           << vehicle.engine.isp_s << ' '
           << vehicle.engine.direction_body.x << ' '
           << vehicle.engine.direction_body.y << ' '
           << vehicle.engine.direction_body.z << ' '
           << vehicle.tanks.size();

    for (const auto& tank : vehicle.tanks) {
        output << ' '
               << std::quoted(tank.name) << ' '
               << std::quoted(tank.propellant) << ' '
               << tank.capacity_kg << ' '
               << tank.initial_kg;
    }
    output << '\n';
    return output.str();
}

std::string make_remote_request(const CaseConfig& config)
{
    const std::string json = case_config_to_json(config);
    std::ostringstream output;
    output << "CASEJSON " << json.size() << '\n' << json;
    return output.str();
}

bool parse_remote_request(const std::string& request, SimulationConfig* config, std::string* error)
{
    std::istringstream input(request);
    std::string command;
    SimulationConfig parsed;
    input >> command;
    if (command != "SIM" && command != "SIMV2" && command != "SIMV3" && command != "SIMV4") {
        if (error) {
            *error = "expected SIM, SIMV2, SIMV3, or SIMV4 request";
        }
        return false;
    }

    input
        >> parsed.duration_s
        >> parsed.step_s
        >> parsed.initial_altitude_m
        >> parsed.inclination_deg
        >> parsed.initial_speed_mps;

    if (!input) {
        if (error) {
            *error = "invalid SIM request fields";
        }
        return false;
    }

    if (command == "SIMV3" || command == "SIMV4") {
        int hold_down_clamp_enabled = 0;
        int normal_force_enabled = 1;
        input
            >> parsed.launch_site.latitude_deg
            >> parsed.launch_site.longitude_deg
            >> parsed.launch_site.altitude_m
            >> hold_down_clamp_enabled
            >> parsed.hold_down_clamp.release_time_s
            >> normal_force_enabled;

        if (!input) {
            if (error) {
                *error = "invalid SIMV3 launch fields";
            }
            return false;
        }

        parsed.hold_down_clamp.enabled = hold_down_clamp_enabled != 0;
        parsed.normal_force.enabled = normal_force_enabled != 0;
    }

    if (command == "SIMV2" || command == "SIMV3" || command == "SIMV4") {
        int engine_enabled = 0;
        int aero_enabled = 0;
        std::size_t tank_count = 0;
        input
            >> std::quoted(parsed.vehicle.name)
            >> parsed.vehicle.dry_mass_kg;
        if (command == "SIMV4") {
            input
                >> aero_enabled
                >> parsed.vehicle.aero.reference_area_m2
                >> parsed.vehicle.aero.cd
                >> parsed.vehicle.aero.cl
                >> std::quoted(parsed.vehicle.aero.aero_table_path);
        }
        input
            >> engine_enabled
            >> parsed.vehicle.engine.max_thrust_n
            >> parsed.vehicle.engine.isp_s
            >> parsed.vehicle.engine.direction_body.x
            >> parsed.vehicle.engine.direction_body.y
            >> parsed.vehicle.engine.direction_body.z
            >> tank_count;

        if (!input) {
            if (error) {
                *error = "invalid SIM vehicle fields";
            }
            return false;
        }

        parsed.vehicle.aero.enabled = aero_enabled != 0;
        parsed.vehicle.engine.enabled = engine_enabled != 0;
        parsed.vehicle.tanks.assign(tank_count, post2::vehicle::TankConfig{});
        for (auto& tank : parsed.vehicle.tanks) {
            input
                >> std::quoted(tank.name)
                >> std::quoted(tank.propellant)
                >> tank.capacity_kg
                >> tank.initial_kg;
            if (!input) {
                if (error) {
                    *error = "invalid SIM tank fields";
                }
                return false;
            }
        }
        if (parsed.vehicle.engine.enabled &&
            parsed.vehicle.engine.max_thrust_n > 0.0 &&
            parsed.vehicle.engine.feed_tanks.empty() &&
            !parsed.vehicle.tanks.empty()) {
            parsed.vehicle.engine.feed_tanks = {{"stage 1", parsed.vehicle.tanks.front().name}};
        }
        parsed.vehicle.stages = post2::vehicle::effective_stage_configs(parsed.vehicle);
    }

    if (!post2::vehicle::validate_vehicle_config(parsed.vehicle, error)) {
        return false;
    }

    *config = parsed;
    return true;
}

bool parse_remote_request(const std::string& request, CaseConfig* config, std::string* error)
{
    if (request.rfind("CASEJSON ", 0) == 0) {
        const std::size_t newline = request.find('\n');
        if (newline == std::string::npos) {
            if (error) {
                *error = "CASEJSON request missing JSON payload";
            }
            return false;
        }

        std::size_t byte_count = 0;
        try {
            std::size_t consumed = 0;
            byte_count = static_cast<std::size_t>(std::stoull(request.substr(9, newline - 9), &consumed));
            if (consumed != newline - 9) {
                if (error) {
                    *error = "invalid CASEJSON byte count";
                }
                return false;
            }
        } catch (const std::exception&) {
            if (error) {
                *error = "invalid CASEJSON byte count";
            }
            return false;
        }

        const std::string payload = request.substr(newline + 1);
        if (payload.size() < byte_count) {
            if (error) {
                *error = "CASEJSON payload shorter than byte count";
            }
            return false;
        }
        return case_config_from_json(payload.substr(0, byte_count), config, error);
    }

    SimulationConfig simulation_config;
    if (!parse_remote_request(request, &simulation_config, error)) {
        return false;
    }
    *config = case_from_simulation_config(simulation_config);
    return true;
}

} // namespace post2::core
