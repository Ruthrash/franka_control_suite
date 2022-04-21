// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <eigen3/Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>

 #include <franka/model.h>
// #include <franka/robot.h>
#include "zmq/dyn_model_subscriber.hpp"
#include "sim_layer.hpp"
#include "examples_common.h"

#include "zmq/reference_subscriber.hpp"
#include "zmq/dyn_model_subscriber.hpp"

/**
 * @example cartesian_impedance_control.cpp
 * An example showing a simple cartesian impedance controller without inertia shaping
 * that renders a spring damper system where the equilibrium is the initial configuration.
 * After starting the controller try to push the robot around and try different stiffness levels.
 *
 * @warning collision thresholds are set to high values. Make sure you have the user stop at hand!
 */


namespace torqueComms {
    //if REAL 
    // JointListener jointListener("tcp://192.168.1.2:2069");
    // JointPublisher jointPublisher("tcp://192.168.1.3:2096");
    //if sim
    //ReferenceSubscriber referenceSubscriber("tcp://127.0.0.1:2069");
    TorqueCommandPublisher torqueCommandPublisher("tcp://127.0.0.1:5069");
}

namespace robotContext {
    //franka::Robot robot;
    //franka::Model model = robotContext::robot.loadModel();
  DynamicsModelSubscriber model("tcp://127.0.0.1:3069");
  SimLayer robot; 
}

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  // if (argc != 2) {
  //   std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  //   return -1;
  // }

  // Compliance parameters
  const double translational_stiffness{1500.0};
  const double rotational_stiffness{100.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  try {
    // connect to robot
    //franka::Robot robot(argv[1]);
    
    
    //setDefaultBehavior(robot);

    // load the kinematics and dynamics model
    //franka::Model model = robotContext::robot.loadModel();

    franka::RobotState initial_state = robotContext::robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    // set collision behavior
    // robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques {
      // get state variables
      std::array<double, 7> coriolis_array = robotContext::model.coriolis();//robotContext::model.coriolis(robot_state);
       std::array<double, 7> gravity_array = robotContext::model.gravity();
      std::array<double, 42> jacobian_array = robotContext::model.zeroJacobian();
          //robotContext::model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d;

      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.linear() * error.tail(3);

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      // Spring damper system with damping ratio=1
      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      tau_d << tau_task + coriolis + gravity;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      //if((count-1)%4==0){
      std::array<double, 7> jointBroadcast = {tau_d_array[0], tau_d_array[1], tau_d_array[2], tau_d_array[3],  
                                              tau_d_array[4], tau_d_array[5], tau_d_array[6]};
      
      torqueComms::torqueCommandPublisher.writeMessage(jointBroadcast);
      //}      
      return tau_d_array;
    };

    // start real-time control loop
    // std::cout << "WARNING: Collision thresholds are set to high values. "
    //           << "Make sure you have the user stop at hand!" << std::endl
    //           << "After starting try to push the robot and see how it reacts." << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();
    robotContext::robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
