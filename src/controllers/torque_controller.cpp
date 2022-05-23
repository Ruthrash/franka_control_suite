#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

#include "controllers/torque_controller.h"
#include "context/context.h"
#include "pinocchio/algorithm/rnea.hpp"


TorqueGenerator::TorqueGenerator(int start, bool useGripper) : 
    useGripper(useGripper), interpolator(0.001), min_jerk_interpolator(0.001, 1. / 120.) {
    count = start;
    q_goal = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4, 0.04, 0.04}; 
    for(size_t i = 0; i < 7; i++)
        interPos.push_back(q_goal[i]);
    interpolator.setInterpolationTimeWindow(1 / 60.);

    if(commsContext::subscriber.type == CommsDataType::JOINT_ANGLES_VEL_GRIPPER) {
        useGripper = true;
        franka::GripperState gripperState = robotContext::gripper.readOnce();
        maxWidth = gripperState.max_width;
        gripperThread = new std::thread(&TorqueGenerator::gripperThreadProc, this);
    }
}

TorqueGenerator::TorqueGenerator(const std::vector<double>& q_goal) : 
    q_goal(q_goal), interpolator(0.001), min_jerk_interpolator(0.001, 1 / 60) {
    count = 1;
    interpolator.setInterpolationTimeWindow(1 / 120);
}

franka::Torques TorqueGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
    if((count-1)%3==0){
        std::vector<double> jointBroadcast = {
            robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3], robot_state.q[4], robot_state.q[5], robot_state.q[6],
            robot_state.dq[0], robot_state.dq[1], robot_state.dq[2], robot_state.dq[3], robot_state.dq[4], robot_state.dq[5], robot_state.dq[6],
        };
        
        commsContext::publisher.writeMessage(jointBroadcast);
    }

    if(count % 8 == 0) {
        commsContext::subscriber.readValues(q_goal);
        Eigen::VectorXd start_position(7);
        Eigen::VectorXd end_position(7);
        Eigen::VectorXd start_velocity(7);
        Eigen::VectorXd end_velocity(7);

        start_position << robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3], robot_state.q[4], robot_state.q[5], robot_state.q[6];
        start_velocity << robot_state.dq[0], robot_state.dq[1], robot_state.dq[2], robot_state.dq[3], robot_state.dq[4], robot_state.dq[5], robot_state.dq[6];
        end_position << q_goal[0], q_goal[1], q_goal[2], q_goal[3], q_goal[4], q_goal[5], q_goal[6];
        end_velocity << q_goal[7], q_goal[8], q_goal[9], q_goal[10], q_goal[11], q_goal[12], q_goal[13];
       
        interpolator.setTargetEndpoints(start_position, end_position, start_velocity, end_velocity);
        waitingForCommand = false;
    }
    if(!waitingForCommand) {
        interpolator.step(interPos, interVel, interAccel);
    }

    count++;
    // read current coriolis terms from model
    std::array<double, DOF> coriolis = robotContext::model.coriolis(robot_state);
    // read gravity term from model (maybe we might want this?)
    std::array<double, DOF> gravity = robotContext::model.gravity(robot_state);
    // mass
    auto mass = robotContext::model.mass(robot_state);
    std::array<double, DOF> tau_d_calculated;

    for (size_t i = 0; i < DOF; i++) {
        // clamp goal joint position
        if(q_goal[i] < joint_min[i])
            q_goal[i] = joint_min[i];
        else if(q_goal[i] > joint_max[i])
            q_goal[i] = joint_max[i];

    auto q_error = interPos[i] - robot_state.q[i];
	auto q_dot_error = interVel[i] - robot_state.dq[i];
	double acceleration = (robot_state.dq[i] - prevVel[i]) / DT;
	prevVel[i] = robot_state.dq[i];
    
    // calculate torque
    tau_d_calculated[i] = 
        propGains[i] * (q_error)
        + derivGains[i] * q_dot_error
        - velDamping[i] * acceleration;

    // clamp torque
    if(tau_d_calculated[i] > 0.8 * torque_max[i])
        tau_d_calculated[i] = 0.8 *  torque_max[i];
    else if(tau_d_calculated[i] < -0.8 * torque_max[i])
        tau_d_calculated[i] = -0.8 *  torque_max[i];
    }

    std::array<double, DOF> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, robot_state.tau_J_d);

    franka::Torques output(tau_d_rate_limited);

    return output;
}

void TorqueGenerator::gripperThreadProc() {
    while(useGripper) {
        double width = commsContext::subscriber.readGripperCommands();
        franka::GripperState gripper_state = robotContext::gripper.readOnce();
        if(width >= maxWidth)
            width = maxWidth;
        if(std::abs(width - gripperWidth) >= 0.001) {
            gripperWidth = width;
            if(width > 0.005)
                robotContext::gripper.move(gripperWidth, 0.2);
            else {
                if(!robotContext::gripper.grasp(0.04, 0.1, 60)) {
                    std::cout << "FAILED GRASP!" << std::endl;
                }
            }
        }

    } 
}