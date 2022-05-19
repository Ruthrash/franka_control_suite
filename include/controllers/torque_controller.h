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

#include "interpolator/interpolator.h"
#include "interpolator/min_jerk_interpolator.h"

#include <memory>
#include <thread>
#include <mutex>

#define DOF 7 
#define DT 0.001


class TorqueGenerator {
public:
    
    TorqueGenerator(int start, bool useGripper);
    TorqueGenerator(const std::vector<double>& q_goal);
    // desired joint configuration
    std::vector<double> q_goal; // count for receiving data 
    size_t count;
    
    franka::Torques operator()(const franka::RobotState& robot_state, franka::Duration period);

private:
    // stiffness gain
    const std::array<double, DOF> propGains = {{800.0, 800.0, 800.0, 800.0, 330, 205, 65}};
    // damping gain
    const std::array<double, DOF> derivGains = {{30.0, 60.0, 50.0, 35.0, 20.0, 15.0, 10.0}};
    // acceleration damping
    const std::array<double, DOF> velDamping = {{0.6, 0.6, 0.6, 0.45, 0.72, 0.6, 0.36}};
    // acceleration 
    std::array<double, DOF> prevVel = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    // joint limits
    const std::array<double, DOF> joint_min = {{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
    const std::array<double, DOF> joint_max = {{2.8973, 1.7628 	, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};
    // torque limits
    const std::array<double, DOF> torque_max = {{87, 87, 87, 87, 12, 12, 12}};
    // calculated torque output
    std::array<double, DOF> torque_output = {{0, 0, 0, 0, 0, 0, 0}};

    QuinticInterpolator interpolator;
    MinJerkInterpolator min_jerk_interpolator;

    std::vector<double> interPos;
    std::vector<double> interVel = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> interAccel = {0, 0, 0, 0, 0, 0, 0};

    bool waitingForCommand = true;

    bool useGripper = false;

    std::thread* gripperThread;
    double gripperWidth = 0;
    double maxWidth = 0;

    void gripperThreadProc();
};
