// PROJECT FILE HEADER
// 文件：apps/screw_driver/src/screw_driver.cpp
// 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
// 用法：请从项目根目录按 README/项目说明.md 中的命令编译或运行。
// 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
// END PROJECT FILE HEADER

#include "screw_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#ifdef USE_OPENARM_CAN
#include <cstring>
#include <net/if.h>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {
constexpr auto kControlPeriod = std::chrono::microseconds(2000);  // 500 Hz

double clamp_speed(double rpm, double max_abs_speed_rpm) {
    return std::max(-max_abs_speed_rpm, std::min(max_abs_speed_rpm, rpm));
}

#ifdef USE_OPENARM_CAN
bool can_interface_is_up(const std::string& interface, std::string& error) {
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        error = "failed to open socket for interface check";
        return false;
    }

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0) {
        close(fd);
        error = "CAN interface not found or not accessible: " + interface;
        return false;
    }
    close(fd);

    if ((ifr.ifr_flags & IFF_UP) == 0) {
        error = "CAN interface is DOWN: " + interface;
        return false;
    }
    return true;
}
#endif
}

ScrewDriverBackend::ScrewDriverBackend(ScrewDriverConfig config)
    : config_(std::move(config)),
      last_motion_command_(std::chrono::steady_clock::now()),
      last_feedback_(std::chrono::steady_clock::now()) {
    status_.torque_stop_nm = config_.torque_stop_nm;
}

ScrewDriverBackend::~ScrewDriverBackend() {
    stop();
}

bool ScrewDriverBackend::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return true;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialize_openarm_locked()) {
            running_ = false;
            return false;
        }
    }
    worker_ = std::thread(&ScrewDriverBackend::control_loop, this);
    return true;
}

void ScrewDriverBackend::stop() {
    running_ = false;
    if (worker_.joinable()) {
        worker_.join();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_openarm_locked();
}

bool ScrewDriverBackend::can_accept_motion() const {
    return !status_.emergency_stop && !status_.fault;
}

void ScrewDriverBackend::mark_motion_command_locked() {
    last_motion_command_ = std::chrono::steady_clock::now();
}

bool ScrewDriverBackend::motion_command_timed_out_locked(std::chrono::steady_clock::time_point now) const {
    if (config_.command_timeout_sec <= 0.0 || status_.target_speed_rpm == 0.0) {
        return false;
    }
    const auto elapsed = std::chrono::duration<double>(now - last_motion_command_).count();
    return elapsed > config_.command_timeout_sec;
}

bool ScrewDriverBackend::initialize_openarm_locked() {
    if (!config_.use_openarm) {
        return true;
    }
#ifdef USE_OPENARM_CAN
    try {
        std::string interface_error;
        if (!can_interface_is_up(config_.can_interface, interface_error)) {
            status_.fault = true;
            status_.fault_message = interface_error;
            status_.mode = DriverMode::Fault;
            return false;
        }
        openarm_ = std::make_unique<openarm::can::socket::OpenArm>(
            config_.can_interface,
            config_.use_can_fd
        );
        std::vector<openarm::damiao_motor::MotorType> motor_types = {
            static_cast<openarm::damiao_motor::MotorType>(config_.motor_type)
        };
        std::vector<uint32_t> send_can_ids = {config_.send_can_id};
        std::vector<uint32_t> recv_can_ids = {config_.recv_can_id};
        openarm_->init_arm_motors(motor_types, send_can_ids, recv_can_ids);
        openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        openarm_->switch_control_mode_all(openarm::damiao_motor::Control_Mode_Code::VEL);
        openarm_->recv_all(1000);
        openarm_->enable_all();
        openarm_->recv_all(1000);
        motor_enabled_ = true;
        const auto motor = openarm_->get_arm().get_motor(0);
        status_.feedback_count = motor.get_state_update_count();
        if (status_.feedback_count == 0) {
            status_.fault = true;
            status_.fault_message =
                "no valid CAN motor feedback during startup; check red indicator, CAN IDs and CAN-FD settings";
            status_.mode = DriverMode::Fault;
            return false;
        }
        last_feedback_ = std::chrono::steady_clock::now();
        return true;
    } catch (const std::exception& e) {
        status_.fault = true;
        status_.fault_message = e.what();
        status_.mode = DriverMode::Fault;
        return false;
    }
#else
    status_.fault = true;
    status_.fault_message = "Built without USE_OPENARM_CAN";
    status_.mode = DriverMode::Fault;
    return false;
#endif
}

void ScrewDriverBackend::shutdown_openarm_locked() {
#ifdef USE_OPENARM_CAN
    if (openarm_) {
        try {
            disable_motor_locked();
        } catch (...) {
        }
        openarm_.reset();
    }
#endif
}

bool ScrewDriverBackend::enable_motor_locked() {
#ifdef USE_OPENARM_CAN
    if (config_.use_openarm && openarm_ && !motor_enabled_) {
        try {
            openarm_->enable_all();
            openarm_->recv_all(2000);
            motor_enabled_ = true;
            last_feedback_ = std::chrono::steady_clock::now();
        } catch (const std::exception& e) {
            status_.fault = true;
            status_.fault_message = std::string("failed to enable motor: ") + e.what();
            status_.mode = DriverMode::Fault;
            return false;
        }
    }
#endif
    return true;
}

void ScrewDriverBackend::disable_motor_locked() {
#ifdef USE_OPENARM_CAN
    if (config_.use_openarm && openarm_ && motor_enabled_) {
        try {
            openarm_->get_arm().vel_control_all({0.0f});
            openarm_->recv_all(1000);
            openarm_->disable_all();
            openarm_->recv_all(1000);
        } catch (...) {
            // Best effort during a safety stop; keep the target speed at zero.
        }
        motor_enabled_ = false;
    }
#endif
}

bool ScrewDriverBackend::set_speed(double rpm) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!can_accept_motion()) {
        return false;
    }
    if (rpm != 0.0 && status_.torque_stop_nm > 0.0 &&
        std::abs(status_.torque_nm) >= status_.torque_stop_nm) {
        status_.target_speed_rpm = 0.0;
        status_.mode = DriverMode::Holding;
        return false;
    }
    if (rpm != 0.0 && !enable_motor_locked()) {
        return false;
    }
    mark_motion_command_locked();
    status_.fault_message.clear();
    status_.target_speed_rpm = clamp_speed(rpm, config_.max_abs_speed_rpm);
    if (status_.target_speed_rpm > 0) {
        status_.mode = DriverMode::Forward;
    } else if (status_.target_speed_rpm < 0) {
        status_.mode = DriverMode::Reverse;
    } else {
        status_.mode = DriverMode::Idle;
    }
    return true;
}

bool ScrewDriverBackend::set_torque_stop(double torque_nm) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(torque_nm) || torque_nm < 0.0) {
        return false;
    }
    config_.torque_stop_nm = torque_nm;
    status_.torque_stop_nm = torque_nm;
    if (torque_nm > 0.0 && std::abs(status_.torque_nm) >= torque_nm) {
        status_.target_speed_rpm = 0.0;
        status_.mode = DriverMode::Holding;
    }
    return true;
}

bool ScrewDriverBackend::forward(double rpm) {
    return set_speed(std::abs(rpm));
}

bool ScrewDriverBackend::reverse(double rpm) {
    return set_speed(-std::abs(rpm));
}

bool ScrewDriverBackend::hold() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!can_accept_motion()) {
        return false;
    }
    mark_motion_command_locked();
    status_.fault_message.clear();
    status_.target_speed_rpm = 0.0;
    status_.mode = DriverMode::Holding;
    disable_motor_locked();
    return true;
}

bool ScrewDriverBackend::heartbeat() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!can_accept_motion()) {
        return false;
    }
    mark_motion_command_locked();
    status_.fault_message.clear();
    return true;
}

bool ScrewDriverBackend::clear_fault() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_.emergency_stop) {
        return false;
    }
    status_.fault = false;
    status_.fault_message.clear();
    status_.mode = DriverMode::Idle;
    return true;
}

bool ScrewDriverBackend::set_emergency_stop(bool active) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.emergency_stop = active;
    mark_motion_command_locked();
    status_.target_speed_rpm = 0.0;
    if (active) {
        status_.measured_speed_rpm = 0.0;
        status_.mode = DriverMode::EmergencyStop;
    } else if (!status_.fault) {
        status_.mode = DriverMode::Idle;
    }
    return true;
}

ScrewStatus ScrewDriverBackend::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

std::string ScrewDriverBackend::status_json() const {
    const ScrewStatus s = status();
    nlohmann::json out = {
        {"ok", true},
        {"mode", mode_to_string(s.mode)},
        {"target_speed_rpm", s.target_speed_rpm},
        {"measured_speed_rpm", s.measured_speed_rpm},
        {"motor_position_rad", s.motor_position_rad},
        {"motor_velocity_rad_s", s.motor_velocity_rad_s},
        {"current_amp", s.current_amp},
        {"torque_nm", s.torque_nm},
        {"torque_stop_nm", s.torque_stop_nm},
        {"mos_temperature_c", s.mos_temperature_c},
        {"rotor_temperature_c", s.rotor_temperature_c},
        {"emergency_stop", s.emergency_stop},
        {"fault", s.fault},
        {"fault_message", s.fault_message},
        {"tick", s.tick},
        {"feedback_count", s.feedback_count},
        {"feedback_age_ms", s.feedback_age_ms},
    };
    return out.dump();
}

void ScrewDriverBackend::control_loop() {
    while (running_) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            status_.tick += 1;
            const auto now = std::chrono::steady_clock::now();
            if (motion_command_timed_out_locked(now)) {
                status_.target_speed_rpm = 0.0;
                status_.mode = DriverMode::Holding;
                status_.fault_message = "motion command timeout, auto hold";
                disable_motor_locked();
            }
            if (status_.emergency_stop || status_.fault) {
                status_.measured_speed_rpm = 0.0;
                status_.motor_velocity_rad_s = 0.0;
                status_.current_amp = 0.0;
                status_.torque_nm = 0.0;
            } else {
#ifdef USE_OPENARM_CAN
                if (config_.use_openarm && openarm_) {
                    if (!motor_enabled_) {
                        status_.measured_speed_rpm = 0.0;
                        status_.motor_velocity_rad_s = 0.0;
                        status_.feedback_age_ms = 0.0;
                    } else {
                    const float vel_rad_s = static_cast<float>(
                        status_.target_speed_rpm * config_.rpm_to_rad_per_sec
                    );
                    openarm_->get_arm().vel_control_all({vel_rad_s});
                    openarm_->recv_all(1000);
                    const auto motor = openarm_->get_arm().get_motor(0);
                    const auto feedback_count = motor.get_state_update_count();
                    if (feedback_count > status_.feedback_count) {
                        status_.feedback_count = feedback_count;
                        last_feedback_ = now;
                    }
                    status_.feedback_age_ms =
                        std::chrono::duration<double, std::milli>(now - last_feedback_).count();
                    if (config_.feedback_timeout_sec > 0.0 &&
                        status_.feedback_age_ms > config_.feedback_timeout_sec * 1000.0) {
                        status_.target_speed_rpm = 0.0;
                        status_.fault = true;
                        status_.fault_message = "CAN motor feedback timeout";
                        status_.mode = DriverMode::Fault;
                        continue;
                    }
                    status_.motor_position_rad = motor.get_position();
                    status_.motor_velocity_rad_s = motor.get_velocity();
                    status_.measured_speed_rpm =
                        motor.get_velocity() / config_.rpm_to_rad_per_sec;
                    status_.torque_nm = motor.get_torque();
                    status_.mos_temperature_c = motor.get_state_tmos();
                    status_.rotor_temperature_c = motor.get_state_trotor();
                    status_.current_amp = 0.0;
                    if (status_.torque_stop_nm > 0.0 &&
                        std::abs(status_.torque_nm) >= status_.torque_stop_nm) {
                        status_.target_speed_rpm = 0.0;
                        status_.mode = DriverMode::Holding;
                        disable_motor_locked();
                    }
                    }
                } else
#endif
                {
                const double error = status_.target_speed_rpm - status_.measured_speed_rpm;
                status_.measured_speed_rpm += error * 0.12;
                status_.motor_velocity_rad_s =
                    status_.measured_speed_rpm * config_.rpm_to_rad_per_sec;
                status_.current_amp = 0.2 + std::abs(status_.measured_speed_rpm) / 1200.0;
                status_.torque_nm = std::abs(status_.measured_speed_rpm) / 3000.0;
                }
            }
        }
        std::this_thread::sleep_for(kControlPeriod);
    }
}
