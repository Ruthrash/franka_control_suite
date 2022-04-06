#pragma once

#include <array>
#include <string>

#include <eigen3/Eigen/Core>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>

#include "joint_listener.h"
#include "joint_publisher.h"

#include <memory>

#define DOF 7 

namespace torqueComms {
    extern JointListener jointListener;
    extern JointPublisher jointPublisher;
    
}

namespace robotContext {
    extern franka::Robot robot;
    extern franka::Model model;
}

class TorqueGenerator {
public:
    
    TorqueGenerator(int start);
    
    
    franka::Torques operator()(const franka::RobotState& robot_state, franka::Duration period);

private:
    // stiffness gain
    // const std::array<double, DOF> k_s = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    const std::array<double, DOF> k_s = {{700.0, 700.0, 700.0, 700.0, 291.67, 175.0, 58.33}};
    
    // damping gain
    const std::array<double, DOF> k_d = {{100.0, 100.0, 100.0, 100.0, 60.0, 50.0, 30.0}};
    // const std::array<double, DOF> k_d = {{100.0, 100.0, 100.0, 100.0, 30.0, 25.0, 15.0}};
    // integral gain
    const std::array<double, DOF> k_i = {{100.0, 100.0, 100.0, 200.0, 120.0, 100.0, 30.0}};
    // integral
    std::array<double, DOF> integral = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    // const std::array<double, DOF> k_d = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
    // joint limits
    const std::array<double, DOF> joint_min = {{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
    const std::array<double, DOF> joint_max = {{2.8973, 1.7628 	, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};
    // torque limits
    const std::array<double, DOF> torque_max = {{87, 87, 87, 87, 12, 12, 12}};
    // calculated torque output
    std::array<double, DOF> torque_output = {{0, 0, 0, 0, 0, 0, 0}};
    // count for receiving data 
    size_t count;
    // desired joint configuration
    std::array<double, 9> q_goal;
};