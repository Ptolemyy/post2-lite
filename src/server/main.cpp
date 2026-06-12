#include "post2/core/io.hpp"
#include "post2/core/trajectory_service.hpp"

#include <cstdlib>
#include <cstring>
#include <iostream>
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

bool parse_int(const std::string& text, int* value)
{
    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        *value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

void print_help()
{
    std::cout
        << "Usage: post2_core_server [--port 5050] [--once]\n\n"
        << "Runs the POST2 Lite core as a simple TCP trajectory service.\n";
}

bool send_all(SocketHandle socket, const std::string& data)
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
            return false;
        }
        cursor += sent;
        remaining -= static_cast<std::size_t>(sent);
    }
    return true;
}

std::string receive_request(SocketHandle socket)
{
    std::string request;
    char buffer[4096];
    while (request.size() < 1024 * 1024) {
#ifdef _WIN32
        const int received = recv(socket, buffer, static_cast<int>(sizeof(buffer)), 0);
#else
        const ssize_t received = recv(socket, buffer, sizeof(buffer), 0);
#endif
        if (received <= 0) {
            break;
        }
        request.append(buffer, buffer + received);
    }
    return request;
}

SocketHandle create_listener(int port)
{
    addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;

    addrinfo* result = nullptr;
    const std::string port_text = std::to_string(port);
    const int rc = getaddrinfo(nullptr, port_text.c_str(), &hints, &result);
    if (rc != 0) {
        std::cerr << "getaddrinfo failed: " << rc << '\n';
        return kInvalidSocket;
    }

    SocketHandle listener = kInvalidSocket;
    for (addrinfo* ptr = result; ptr != nullptr; ptr = ptr->ai_next) {
        listener = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
        if (listener == kInvalidSocket) {
            continue;
        }

        int yes = 1;
#ifdef _WIN32
        setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(yes));
#else
        setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#endif

        if (bind(listener, ptr->ai_addr, static_cast<int>(ptr->ai_addrlen)) == 0 &&
            listen(listener, 8) == 0) {
            break;
        }
        close_socket(listener);
        listener = kInvalidSocket;
    }

    freeaddrinfo(result);
    return listener;
}

void serve_client(SocketHandle client)
{
    const std::string request = receive_request(client);

    post2::core::CaseConfig config;
    std::string error;
    if (!post2::core::parse_remote_request(request, &config, &error)) {
        send_all(client, "ERR " + error + "\n");
        return;
    }

    post2::core::LocalTrajectoryService service;
    const auto result = service.simulate(config);
    if (!result.ok) {
        send_all(client, "ERR " + result.error + "\n");
        return;
    }

    send_all(client, post2::core::trajectory_to_csv(result.state_log));
}

} // namespace

int main(int argc, char** argv)
{
    int port = 5050;
    bool once = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_help();
            return 0;
        }
        if (arg == "--once") {
            once = true;
            continue;
        }
        if (arg == "--port" && i + 1 < argc) {
            if (!parse_int(argv[++i], &port)) {
                std::cerr << "Invalid --port value\n";
                return 2;
            }
            continue;
        }
        std::cerr << "Unknown argument: " << arg << '\n';
        print_help();
        return 2;
    }

    SocketRuntime runtime;
    if (!runtime.ok()) {
        std::cerr << "Failed to initialize socket runtime\n";
        return 1;
    }

    const SocketHandle listener = create_listener(port);
    if (listener == kInvalidSocket) {
        std::cerr << "Failed to listen on port " << port << ": " << last_socket_error() << '\n';
        return 1;
    }

    std::cout << "post2_core_server listening on 127.0.0.1:" << port << '\n';
    for (;;) {
        sockaddr_storage client_addr = {};
#ifdef _WIN32
        int addr_len = sizeof(client_addr);
#else
        socklen_t addr_len = sizeof(client_addr);
#endif
        const SocketHandle client = accept(listener, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (client == kInvalidSocket) {
            std::cerr << "accept failed: " << last_socket_error() << '\n';
            continue;
        }

        serve_client(client);
        close_socket(client);

        if (once) {
            break;
        }
    }

    close_socket(listener);
    return 0;
}
