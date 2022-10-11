#pragma once
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <Poco/DateTimeFormatter.h>
#include <Poco/File.h>
#include <Poco/Path.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include <thread>
#include "common.h"
#include "tests/test_common.h"
#include <atomic>
#include <eigen3/Eigen/Dense>
namespace test_params{
    double vel_current = 0.0;
    const double acceleration_time = 2.0;
    const double runtime = 20.0;
}
franka::JointPositions test_joint_pos_motion_generator(const franka::RobotState&robot_state, franka::Duration period, const std::array<double, 7>& initial_position, double& index){
    index += period.toSec();
    double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * index));
    franka::JointPositions output = {{initial_position[0], initial_position[1],
                                    initial_position[2], initial_position[3] + delta_angle,
                                    initial_position[4] + delta_angle, initial_position[5],
                                    initial_position[6] + delta_angle}};    
    return output;  
}

franka::CartesianPose test_cartesian_pose_motion_generator(const franka::RobotState&robot_state, franka::Duration period,const std::array<double, 16>& initial_pose, double& index){
    index += period.toSec();
    
    double angle = 0.0;
    const double radius = 0.05;
    const double vel_max = 0.25;

    // Compute Cartesian velocity.
    if (test_params::vel_current < vel_max && index < test_params::runtime) {
        test_params::vel_current += period.toSec() * std::fabs(vel_max / test_params::acceleration_time);
    }
    if (test_params::vel_current > 0.0 && index > test_params::runtime) {
        test_params::vel_current -= period.toSec() * std::fabs(vel_max / test_params::acceleration_time);
    }
    test_params::vel_current = std::fmax(test_params::vel_current, 0.0);
    test_params::vel_current = std::fmin(test_params::vel_current, vel_max);

    // Compute new angle for our circular trajectory.
    angle += period.toSec() * test_params::vel_current / std::fabs(radius);
    if (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
    }        
    // Compute relative y and z positions of desired pose.
    double delta_y = radius * (1 - std::cos(angle));
    double delta_z = radius * std::sin(angle);
    franka::CartesianPose pose_desired = initial_pose;
    pose_desired.O_T_EE[13] += delta_y;
    pose_desired.O_T_EE[14] += delta_z;   
    return pose_desired;

}
// joint_velocity_generator

// cartesian_velocity_generator
