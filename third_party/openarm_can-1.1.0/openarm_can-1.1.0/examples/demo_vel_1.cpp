// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>

const std::vector<openarm::damiao_motor::MotorType> DEFAULT_MOTOR_TYPES = {
    openarm::damiao_motor::MotorType::DM8009,  // Joint 1
    openarm::damiao_motor::MotorType::DM8009,  // Joint 2
    openarm::damiao_motor::MotorType::DM4340,  // Joint 3
    openarm::damiao_motor::MotorType::DM4340,  // Joint 4
    openarm::damiao_motor::MotorType::DM4310,  // Joint 5
    openarm::damiao_motor::MotorType::DM4310,  // Joint 6
    openarm::damiao_motor::MotorType::DM4310   // Joint 7
};

// const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
//                                                     0x05, 0x06, 0x07};
// const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14,
//                                                     0x15, 0x16, 0x17};

int main() {
    try {
        std::cout << "=== OpenArm CAN Example ===" << std::endl;
        std::cout << "This example demonstrates the OpenArm API functionality" << std::endl;

        // Initialize OpenArm with CAN interface and enable CAN-FD
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        // openarm::can::socket::OpenArm openarm("can1", true);  // Use CAN-FD on can0 interface
        auto openarm_ =
    //   std::make_unique<openarm::can::socket::OpenArm>("can1", true);
      std::make_unique<openarm::can::socket::OpenArm>("can0", true);

        // Initialize arm motors
        std::vector<openarm::damiao_motor::MotorType> motor_types = {
            DEFAULT_MOTOR_TYPES[5]};
        std::vector<uint32_t> send_can_ids = {0x06};
        std::vector<uint32_t> recv_can_ids = {0x16};
        openarm_->init_arm_motors(motor_types, send_can_ids, recv_can_ids);
        // openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
        //     DEFAULT_RECV_CAN_IDS);
        // openarm_->init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);


        openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
        openarm_->switch_control_mode_all(openarm::damiao_motor::Control_Mode_Code::VEL);
        openarm_->recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        openarm_->enable_all();
        openarm_->recv_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));


       for (size_t i = 0; i < 500; i++)
       {
           std::vector<float> arm_params;
           double vel = 1 + i / 10.0;
           if(vel > 4.0)
           {
               arm_params.push_back(4);
           }
           else
           {
                arm_params.push_back(vel);
           }
           std::cout << "\n=== Controlling Motors ===" << std::endl;
           openarm_->get_arm().vel_control_all(arm_params);
           openarm_->recv_all(1000);
           std::cout << "vel:" << openarm_->get_arm().get_motor(0).get_velocity() << std::endl;
           double torque = openarm_->get_arm().get_motor(0).get_torque();
           std::cout << "torque:" << torque << std::endl;
           if (torque > 0.3)
           {
                break;
           }
           
           std::this_thread::sleep_for(std::chrono::milliseconds(10));
       }
            

        openarm_->disable_all();
        openarm_->recv_all();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
