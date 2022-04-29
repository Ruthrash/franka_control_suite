#pragma once

#include <array>
#include <string>

#include <eigen3/Eigen/Core>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot_state.h>

#include <memory>

#define DOF 7 


class TorqueGenerator {
public:
    
    TorqueGenerator(int start, bool useGripper);
    TorqueGenerator(const std::vector<double>& q_goal);
    // desired joint configuration
    std::vector<double> q_goal;
    // count for receiving data 
    size_t count;
    
    franka::Torques operator()(const franka::RobotState& robot_state, franka::Duration period);

private:
    // stiffness gain
    const std::array<double, DOF> k_s = {{266.0, 266.0, 266.0, 266.0, 110.83, 68.5, 22.16}};
    // damping gain
    const std::array<double, DOF> k_d = {{40.0, 40.0, 40.0, 40.0, 24.0, 20.0, 12.0}};
    // integral gain
    const std::array<double, DOF> k_i = {{100.0, 100.0, 100.0, 200.0, 120.0, 100.0, 30.0}};
    // integral
    std::array<double, DOF> integral = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    // joint limits
    const std::array<double, DOF> joint_min = {{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
    const std::array<double, DOF> joint_max = {{2.8973, 1.7628 	, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};
    // torque limits
    const std::array<double, DOF> torque_max = {{87, 87, 87, 87, 12, 12, 12}};
    // calculated torque output
    std::array<double, DOF> torque_output = {{0, 0, 0, 0, 0, 0, 0}};

    bool useGripper;
};