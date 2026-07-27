// PROJECT FILE HEADER
// 文件：apps/screw_driver/include/screw_state.hpp
// 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
// 用法：请从项目根目录按 README/项目说明.md 中的命令编译或运行。
// 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
// END PROJECT FILE HEADER

#pragma once

#include <string>

enum class DriverMode {
    Idle,
    Forward,
    Reverse,
    Holding,
    Fault,
    EmergencyStop,
};

struct ScrewStatus {
    DriverMode mode = DriverMode::Idle;
    double target_speed_rpm = 0.0;
    double measured_speed_rpm = 0.0;
    double motor_position_rad = 0.0;
    double motor_velocity_rad_s = 0.0;
    double current_amp = 0.0;
    double torque_nm = 0.0;
    double torque_stop_nm = 0.0;
    int mos_temperature_c = 0;
    int rotor_temperature_c = 0;
    bool emergency_stop = false;
    bool fault = false;
    std::string fault_message;
    unsigned long long tick = 0;
    unsigned long long feedback_count = 0;
    double feedback_age_ms = 0.0;
};

inline const char* mode_to_string(DriverMode mode) {
    switch (mode) {
        case DriverMode::Idle:
            return "IDLE";
        case DriverMode::Forward:
            return "FORWARD";
        case DriverMode::Reverse:
            return "REVERSE";
        case DriverMode::Holding:
            return "HOLDING";
        case DriverMode::Fault:
            return "FAULT";
        case DriverMode::EmergencyStop:
            return "EMERGENCY_STOP";
    }
    return "UNKNOWN";
}
