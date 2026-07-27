// PROJECT FILE HEADER
// 文件：apps/screw_driver/src/screw_monitor.cpp
// 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
// 用法：请从项目根目录按 README/项目说明.md 中的命令编译或运行。
// 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
// END PROJECT FILE HEADER

#include "screw_driver.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

namespace {

void print_usage(const char* exe) {
    std::cerr
        << "Usage: " << exe << " [--openarm] [--can can0] [--send-id 0x06] "
        << "[--recv-id 0x16] [--motor-type 1] [--speed-rpm 0] "
        << "[--period-ms 20] [--duration-sec 0]\n";
}

bool parse_common_arg(int& i, int argc, char** argv, ScrewDriverConfig& config,
                      double& speed_rpm, int& period_ms, double& duration_sec) {
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
    } else if (arg == "--speed-rpm" && i + 1 < argc) {
        speed_rpm = std::stod(argv[++i]);
    } else if (arg == "--period-ms" && i + 1 < argc) {
        period_ms = std::stoi(argv[++i]);
    } else if (arg == "--duration-sec" && i + 1 < argc) {
        duration_sec = std::stod(argv[++i]);
    } else if (arg == "--no-can-fd") {
        config.use_can_fd = false;
    } else if (arg == "--help" || arg == "-h") {
        return false;
    } else {
        std::cerr << "Unknown or incomplete argument: " << arg << "\n";
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    ScrewDriverConfig config;
    double speed_rpm = 0.0;
    int period_ms = 20;
    double duration_sec = 0.0;

    for (int i = 1; i < argc; ++i) {
        if (!parse_common_arg(i, argc, argv, config, speed_rpm, period_ms, duration_sec)) {
            print_usage(argv[0]);
            return 2;
        }
    }

    ScrewDriverBackend driver(config);
    if (!driver.start()) {
        std::cerr << "failed to start screw driver backend\n";
        std::cerr << driver.status_json() << "\n";
        return 1;
    }

    if (speed_rpm != 0.0) {
        if (!driver.set_speed(speed_rpm)) {
            std::cerr << "failed to set speed\n";
            driver.stop();
            return 1;
        }
    } else {
        driver.heartbeat();
    }

    const auto started = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(period_ms > 0 ? period_ms : 20);

    std::cout << "elapsed_s,mode,target_rpm,measured_rpm,position_rad,"
              << "velocity_rad_s,torque_nm,mos_temperature_c,rotor_temperature_c,tick\n";

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - started).count();
        const ScrewStatus status = driver.status();

        std::cout << elapsed << ","
                  << mode_to_string(status.mode) << ","
                  << status.target_speed_rpm << ","
                  << status.measured_speed_rpm << ","
                  << status.motor_position_rad << ","
                  << status.motor_velocity_rad_s << ","
                  << status.torque_nm << ","
                  << status.mos_temperature_c << ","
                  << status.rotor_temperature_c << ","
                  << status.tick << "\n";

        if (duration_sec > 0.0 && elapsed >= duration_sec) {
            break;
        }
        if (speed_rpm != 0.0) {
            driver.heartbeat();
        }
        std::this_thread::sleep_for(period);
    }

    driver.hold();
    driver.stop();
    return 0;
}
