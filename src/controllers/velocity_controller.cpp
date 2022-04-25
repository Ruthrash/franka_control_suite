#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

#include "controllers/velocity_controller.h"
#include "context/context.h"



VelocityGenerator::VelocityGenerator(int start, bool useGripper) : useGripper(useGripper) {
    count = start;
    q_dot_goal = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4, 0.04, 0.04}; 
}

VelocityGenerator::VelocityGenerator(const std::vector<double>& q_dot_goal) : q_dot_goal(q_dot_goal) {
    count = 1;
}

franka::Torques VelocityGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
    if((count-1)%3==0){
        std::vector<double> jointBroadcast = {
            robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3], robot_state.q[4], robot_state.q[5], robot_state.q[6],
            robot_state.dq[0], robot_state.dq[1], robot_state.dq[2], robot_state.dq[3], robot_state.dq[4], robot_state.dq[5], robot_state.dq[6],
        };
        
        commsContext::publisher.writeMessage(jointBroadcast);
    }
    // coriolis compensation
    // if(count % 3 == 0) {
        // commsContext::subscriber.readMessage();
        // q_goal = commsContext::subscriber.values;
    // }
    commsContext::subscriber.readValues(q_dot_goal);

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
        auto q_dot_error = q_dot_goal[i] - robot_state.dq[i];
        auto acceleration = (robot_state.dq[i] - prevVel[i]) / period.toSec();
        prevVel[i] = robot_state.dq[i];
        tau_d_calculated[i] = k_ff[i] * q_dot_goal[i] + k_s[i] * (q_dot_error) - k_d[i] * acceleration;

        // clamp torque
        if(tau_d_calculated[i] > 0.7 * torque_max[i])
            tau_d_calculated[i] = 0.7 *  torque_max[i];
    }


    std::array<double, DOF> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, robot_state.tau_J_d);

    franka::Torques output(tau_d_rate_limited);

    return output;
}
