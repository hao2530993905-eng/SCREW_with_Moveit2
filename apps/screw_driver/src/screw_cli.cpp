// PROJECT FILE HEADER
// 文件：apps/screw_driver/src/screw_cli.cpp
// 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
// 用法：请从项目根目录按 README/项目说明.md 中的命令编译或运行。
// 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
// END PROJECT FILE HEADER

#include "screw_driver.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    std::string command = argc > 1 ? argv[1] : "status";
    double speed = argc > 2 ? std::stod(argv[2]) : 120.0;
    ScrewDriverConfig config;
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--openarm") {
            config.use_openarm = true;
        } else if (arg == "--can" && i + 1 < argc) {
            config.can_interface = argv[++i];
        } else if (arg == "--send-id" && i + 1 < argc) {
            config.send_can_id = static_cast<uint32_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--recv-id" && i + 1 < argc) {
            config.recv_can_id = static_cast<uint32_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--motor-type" && i + 1 < argc) {
            config.motor_type = std::stoi(argv[++i]);
        }
    }

    ScrewDriverBackend driver(config);
    if (!driver.start()) {
        std::cerr << "failed to start screw driver backend\n";
        return 1;
    }

    bool ok = true;
    if (command == "forward") {
        ok = driver.forward(speed);
    } else if (command == "reverse") {
        ok = driver.reverse(speed);
    } else if (command == "hold" || command == "stop") {
        ok = driver.hold();
    } else if (command == "estop") {
        ok = driver.set_emergency_stop(true);
    } else if (command == "clear") {
        ok = driver.clear_fault();
    } else if (command != "status") {
        std::cerr << "Unknown command: " << command << "\n";
        return 2;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "{\"ok\":" << (ok ? "true" : "false")
              << ",\"status\":" << driver.status_json() << "}\n";
    driver.stop();
    return ok ? 0 : 1;
}
