#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

#include "controllers/torque_controller.h"
#include "context/context.h"



TorqueGenerator::TorqueGenerator(int start, bool useGripper) : 
    useGripper(useGripper), interpolator(0.001), min_jerk_interpolator(0.001, 1 / 60) {
    count = start;
    q_goal = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4, 0.04, 0.04}; 
    for(size_t i = 0; i < 7; i++)
        interPos.push_back(q_goal[i]);
    interpolator.setInterpolationTimeWindow(1 / 60.);
}

TorqueGenerator::TorqueGenerator(const std::vector<double>& q_goal) : 
    q_goal(q_goal), interpolator(0.001), min_jerk_interpolator(0.001, 1 / 60) {
    count = 1;
    interpolator.setInterpolationTimeWindow(1 / 60.);
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
    if(count % 16 == 0) {
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
        min_jerk_interpolator.setTargetEndpoints(start_position, end_position);
        waitingForCommand = false;
    }
    std::cout << interPos.size() << " " << interVel.size() << " " << interAccel.size() << std::endl;
    if(!waitingForCommand) {
        interpolator.step(interPos, interVel, interAccel);
        // min_jerk_interpolator.step(interPos, interVel);
    }

    count++;
    // read current coriolis terms from model
    std::array<double, DOF> coriolis = robotContext::model.coriolis(robot_state);
    // read gravity term from model (maybe we might want this?)
    std::array<double, DOF> gravity = robotContext::model.gravity(robot_state);
    // mass
    auto mass = robotContext::model.mass(robot_state);
    std::array<double, DOF> tau_d_calculated;

    double k_i_term = 0;
    for (size_t i = 0; i < DOF; i++) {
        // clamp goal joint position
        if(q_goal[i] < joint_min[i])
            q_goal[i] = joint_min[i];
        else if(q_goal[i] > joint_max[i])
            q_goal[i] = joint_max[i];
    auto q_error = interPos[i] - robot_state.q[i];
	auto q_dot_error = interVel[i] - robot_state.dq[i];
	double acceleration = (robot_state.dq[i] - prevVel[i]) / DT;
	std::cout << "acceleration " << acceleration << std::endl;
	std::cout << "period: " << DT << std::endl;
	prevVel[i] = robot_state.dq[i];
	// reset integral term if sign changes
        if(integral[i] * (integral[i] + DT * q_dot_error) < 0)
	    integral[i] = 0;
	// only add if error is within range
        if(q_error <= 0.05) {
            integral[i] += DT * q_dot_error;
            k_i_term = k_i[i];
        }
        else {
            integral[i] = 0;
            k_i_term = 0;
        }

        float derror;
        if (error_initialised) {
            derror = (q_error - last_error[i]) / DT;
        } else {
            derror = 0.0;
            error_initialised = true;
        }
        last_error[i] = q_error;

        std::cout << derror << std::endl;

        tau_d_calculated[i] = 
            1.5 * k_s[i] * (q_error)
            + 0.3 * k_d[i] * q_dot_error
            - 0.0 * velDamping[i] * acceleration
            - 0.00 * k_dError[i] * derror;
            // - 0.1 * k_dError[i] * derror;

        // clamp torque
        if(tau_d_calculated[i] > 0.7 * torque_max[i])
            tau_d_calculated[i] = 0.7 *  torque_max[i];
    }


    std::array<double, DOF> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, robot_state.tau_J_d);

    franka::Torques output(tau_d_rate_limited);

    return output;
}
