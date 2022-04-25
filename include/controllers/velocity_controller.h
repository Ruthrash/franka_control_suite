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


class VelocityGenerator {
public:
    
    VelocityGenerator(int start, bool useGripper);
    VelocityGenerator(const std::vector<double>& q_goal);
    // desired joint configuration
    std::vector<double> q_dot_goal;
    // count for receiving data 
    size_t count;
    
    franka::Torques operator()(const franka::RobotState& robot_state, franka::Duration period);

private:
    // stiffness gain
    // const std::array<double, DOF> k_s = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    const std::array<double, DOF> k_s = {{700.0, 700.0, 700.0, 700.0, 291.67, 175.0, 58.33}};
    
    // damping gain
    const std::array<double, DOF> k_d = {{100.0, 100.0, 100.0, 100.0, 60.0, 50.0, 30.0}};
    // feedforward
    const std::array<double, DOF> k_ff = {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}};
    // const std::array<double, DOF> k_d = {{100.0, 100.0, 100.0, 100.0, 30.0, 25.0, 15.0}};
    // integral gain
    const std::array<double, DOF> k_i = {{100.0, 100.0, 100.0, 200.0, 120.0, 100.0, 30.0}};
    // integral
    std::array<double, DOF> integral = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    // const std::array<double, DOF> k_d = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
    // torque limits
    const std::array<double, DOF> torque_max = {{87, 87, 87, 87, 12, 12, 12}};
    // calculated torque output
    std::array<double, DOF> torque_output = {{0, 0, 0, 0, 0, 0, 0}};
    // prev velocity
    std::array<double, DOF> prevVel = {{0, 0, 0, 0, 0, 0, 0}};

    bool useGripper;
};