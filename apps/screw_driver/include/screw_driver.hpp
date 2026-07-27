// PROJECT FILE HEADER
// 文件：apps/screw_driver/include/screw_driver.hpp
// 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
// 用法：请从项目根目录按 README/项目说明.md 中的命令编译或运行。
// 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
// END PROJECT FILE HEADER

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "screw_state.hpp"

namespace openarm::can::socket {
class OpenArm;
}

struct ScrewDriverConfig {
    bool use_openarm = false;
    std::string can_interface = "can0";
    bool use_can_fd = true;
    int motor_type = 1;  // openarm::damiao_motor::MotorType::DM4310
    uint32_t send_can_id = 0x06;
    uint32_t recv_can_id = 0x16;
    double rpm_to_rad_per_sec = 0.104719755;
    double max_abs_speed_rpm = 1500.0;
    double torque_stop_nm = 0.0;
    double command_timeout_sec = 0.5;
    double feedback_timeout_sec = 0.1;
};

class ScrewDriverBackend {
public:
    explicit ScrewDriverBackend(ScrewDriverConfig config = {});
    ~ScrewDriverBackend();

    bool start();
    void stop();

    bool set_speed(double rpm);
    bool set_torque_stop(double torque_nm);
    bool forward(double rpm);
    bool reverse(double rpm);
    bool hold();
    bool heartbeat();
    bool clear_fault();
    bool set_emergency_stop(bool active);

    ScrewStatus status() const;
    std::string status_json() const;

private:
    bool initialize_openarm_locked();
    void shutdown_openarm_locked();
    void control_loop();
    bool can_accept_motion() const;
    void mark_motion_command_locked();
    bool motion_command_timed_out_locked(std::chrono::steady_clock::time_point now) const;
    bool enable_motor_locked();
    void disable_motor_locked();

    ScrewDriverConfig config_;
    mutable std::mutex mutex_;
    ScrewStatus status_;
    std::chrono::steady_clock::time_point last_motion_command_;
    std::chrono::steady_clock::time_point last_feedback_;
    std::atomic<bool> running_{false};
    bool motor_enabled_ = false;
    std::thread worker_;
#ifdef USE_OPENARM_CAN
    std::unique_ptr<openarm::can::socket::OpenArm> openarm_;
#endif
};
