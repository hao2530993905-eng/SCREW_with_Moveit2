// PROJECT FILE HEADER
// 文件：apps/screw_driver/src/screw_server.cpp
// 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
// 用法：请从项目根目录按 README/项目说明.md 中的命令编译或运行。
// 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
// END PROJECT FILE HEADER

#include "screw_driver.hpp"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using socket_t = SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_t = int;
#endif

namespace {

void print_usage(const char* exe) {
    std::cerr
        << "Usage: " << exe << " [--openarm] [--can can0] [--send-id 0x06] "
        << "[--recv-id 0x16] [--motor-type 1] [--port 5055] "
        << "[--max-rpm 1500] [--torque-stop 0] [--command-timeout 0.5] "
        << "[--feedback-timeout 0.1] [--no-can-fd]\n";
}

void close_socket(socket_t sock) {
#ifdef _WIN32
    closesocket(sock);
#else
    close(sock);
#endif
}

std::string make_response(bool ok, const std::string& message, const ScrewDriverBackend& driver) {
    nlohmann::json response = {
        {"ok", ok},
        {"message", message},
        {"status", nlohmann::json::parse(driver.status_json())},
    };
    return response.dump() + "\n";
}

std::string handle_request(const std::string& request, ScrewDriverBackend& driver) {
    nlohmann::json payload;
    try {
        payload = nlohmann::json::parse(request);
    } catch (const nlohmann::json::parse_error& e) {
        return make_response(false, std::string("json_parse_error: ") + e.what(), driver);
    }

    const std::string cmd = payload.value("cmd", "");
    if (cmd == "status") {
        return make_response(true, "status", driver);
    }
    if (cmd == "heartbeat") {
        return make_response(driver.heartbeat(), "heartbeat", driver);
    }
    if (cmd == "forward") {
        const double speed = payload.value("speed", 120.0);
        return make_response(driver.forward(speed), "forward", driver);
    }
    if (cmd == "reverse") {
        const double speed = payload.value("speed", 120.0);
        return make_response(driver.reverse(speed), "reverse", driver);
    }
    if (cmd == "set_speed") {
        const double speed = payload.value("speed", 0.0);
        return make_response(driver.set_speed(speed), "set_speed", driver);
    }
    if (cmd == "set_torque_stop") {
        const double torque_nm = payload.value("torque_nm", 0.0);
        return make_response(
            driver.set_torque_stop(torque_nm), "set_torque_stop", driver
        );
    }
    if (cmd == "hold" || cmd == "stop") {
        return make_response(driver.hold(), "hold", driver);
    }
    if (cmd == "clear_fault") {
        return make_response(driver.clear_fault(), "clear_fault", driver);
    }
    if (cmd == "estop") {
        const bool active = payload.value("active", true);
        return make_response(driver.set_emergency_stop(active), "estop", driver);
    }
    return make_response(false, "unknown_command", driver);
}

}  // namespace

int main(int argc, char** argv) {
    int port = 5055;
    ScrewDriverConfig config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = std::stoi(argv[++i]);
        } else if (arg == "--openarm") {
            config.use_openarm = true;
        } else if (arg == "--can" && i + 1 < argc) {
            config.can_interface = argv[++i];
        } else if (arg == "--send-id" && i + 1 < argc) {
            config.send_can_id = static_cast<uint32_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--recv-id" && i + 1 < argc) {
            config.recv_can_id = static_cast<uint32_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--motor-type" && i + 1 < argc) {
            config.motor_type = std::stoi(argv[++i]);
        } else if (arg == "--max-rpm" && i + 1 < argc) {
            config.max_abs_speed_rpm = std::stod(argv[++i]);
        } else if (arg == "--torque-stop" && i + 1 < argc) {
            config.torque_stop_nm = std::stod(argv[++i]);
        } else if (arg == "--command-timeout" && i + 1 < argc) {
            config.command_timeout_sec = std::stod(argv[++i]);
        } else if (arg == "--feedback-timeout" && i + 1 < argc) {
            config.feedback_timeout_sec = std::stod(argv[++i]);
        } else if (arg == "--no-can-fd") {
            config.use_can_fd = false;
        } else if (arg == "--can-fd") {
            config.use_can_fd = true;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (arg.size() > 0 && arg[0] != '-') {
            port = std::stoi(arg);
        } else {
            std::cerr << "Unknown or incomplete argument: " << arg << "\n";
            print_usage(argv[0]);
            return 2;
        }
    }

#ifdef _WIN32
    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        std::cerr << "WSAStartup failed\n";
        return 1;
    }
#endif

    ScrewDriverBackend driver(config);
    if (!driver.start()) {
        std::cerr << "failed to start screw driver backend\n";
        std::cerr << driver.status_json() << "\n";
        return 1;
    }

    socket_t server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "socket failed\n";
        return 1;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<unsigned short>(port));

    int reuse = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));

    if (bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        std::cerr << "bind failed on port " << port << "\n";
        close_socket(server_fd);
        return 1;
    }
    if (listen(server_fd, 4) < 0) {
        std::cerr << "listen failed\n";
        close_socket(server_fd);
        return 1;
    }

    std::cout << "screw_server listening on 0.0.0.0:" << port << "\n";

    while (true) {
        socket_t client = accept(server_fd, nullptr, nullptr);
        if (client < 0) {
            continue;
        }
        std::string buffer;
        char chunk[1024];
        while (true) {
            const int n = recv(client, chunk, sizeof(chunk), 0);
            if (n <= 0) {
                break;
            }
            buffer.append(chunk, chunk + n);
            size_t newline = 0;
            while ((newline = buffer.find('\n')) != std::string::npos) {
                const std::string line = buffer.substr(0, newline);
                buffer.erase(0, newline + 1);
                const std::string response = handle_request(line, driver);
                send(client, response.c_str(), static_cast<int>(response.size()), 0);
            }
        }
        close_socket(client);
    }
}
