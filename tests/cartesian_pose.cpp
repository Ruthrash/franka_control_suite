// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
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

#include "controllers/cartesian_impedance.h"

namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;

}
namespace Comms {
    ActionSubscriber *actionSubscriber; 
    StatePublisher *statePublisher; 
}

// Compliance parameters
const double translational_stiffness{150.0};
const double rotational_stiffness{10.0};
Eigen::MatrixXd stiffness(6, 6), damping(6, 6);

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  // Parameters
  const size_t joint_number{3};
  const size_t filter_size{5};
  const double print_rate = 10.0;
  std::atomic_bool running{true};
  double vel_current = 0.0;
  double angle = 0.0;
  const double radius = 0.1;
  const double vel_max = 0.25;
  const double acceleration_time = 2.0;
  const double run_time = 20.0;

    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                        Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                            Eigen::MatrixXd::Identity(3, 3);

  // NOLINTNEXTLINE(readability-identifier-naming)
  const std::array<double, 7> K_P{{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}};
  // NOLINTNEXTLINE(readability-identifier-naming)
  const std::array<double, 7> K_D{{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
  const double max_acceleration{1.0};

  CartesianImpedance controller(1, false);
  std::string file_name = "/home/ruthrash/log.txt";
  bool joint_space=true;
  std::thread print_thread = std::thread(log_data, std::cref(print_rate), std::ref(print_data), std::ref(running), std::ref(file_name), std::cref(joint_space));
  try {
    franka::Robot robot(argv[1]);
    franka::Model model_ = robot.loadModel();
    robotContext::model = &model_;
    setDefaultBehavior(robot);
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    double index = 0.0;
    // std::vector<double> trajectory = generateTrajectory(max_acceleration);
    std::array<double, 16> initial_pose;
    robot.control(
        [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
          return controller(robot_state, period);
        },
        [&](const franka::RobotState&robot_state, franka::Duration period) -> franka::CartesianPose {
        index += period.toSec();
        if (index == 0.0) {
            initial_pose = robot_state.O_T_EE_c;
        }          
        // Compute Cartesian velocity.
        if (vel_current < vel_max && index < run_time) {
            vel_current += period.toSec() * std::fabs(vel_max / acceleration_time);
        }
        if (vel_current > 0.0 && index > run_time) {
            vel_current -= period.toSec() * std::fabs(vel_max / acceleration_time);
        }
        vel_current = std::fmax(vel_current, 0.0);
        vel_current = std::fmin(vel_current, vel_max);

        // Compute new angle for our circular trajectory.
        angle += period.toSec() * vel_current / std::fabs(radius);
        if (angle > 2 * M_PI) {
            angle -= 2 * M_PI;
        }        

        // Compute relative y and z positions of desired pose.
        double delta_y = radius * (1 - std::cos(angle));
        double delta_z = radius * std::sin(angle);
        franka::CartesianPose pose_desired = initial_pose;
        pose_desired.O_T_EE[13] += delta_y;
        pose_desired.O_T_EE[14] += delta_z;

        if (index >= run_time + acceleration_time) {
            running = false;
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(pose_desired);
        }                                        

        return pose_desired;
        });
  } catch (const franka::ControlException& e) {
    std::cout << e.what() << std::endl;
    return -1;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  if (print_thread.joinable()) {
    print_thread.join();
  }  
  return 0;
}  