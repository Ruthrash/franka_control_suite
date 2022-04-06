#include "controllers/joint_space/joint_position_controller.hpp"
#include <iostream>

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

JointPositionController::JointPositionController(){}

JointPositionController::~JointPositionController(){}

namespace torqueComms {
    // JointListener jointListener("tcp://192.168.1.2:2069");
    // JointPublisher jointPublisher("tcp://192.168.1.3:2096");
    ReferenceSubscriber referenceSubscriber("tcp://192.168.1.2:2069");
    // TorqueCommandPublisher torqueCommandPublisher("tcp://192.168.1.2:2069");
}

franka::Torques JointPositionController::operator() (const franka::RobotState&, franka::Duration){
    std::array<double, DOF> tau_d_rate_limited;
    return tau_d_rate_limited;
} 

