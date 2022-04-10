#include "controllers/joint_space/joint_position_controller.hpp"
#include <iostream>

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

/*
Constructor must have atleast one parameter. Because when robot.cotrol()is run, 
it tries to pass the () operator with this class names, which will conflict with the base constructor
*/
JointPositionController::JointPositionController(int start){count =start;}

JointPositionController::~JointPositionController(){}

namespace torqueComms {
    // JointListener jointListener("tcp://192.168.1.2:2069");
    // JointPublisher jointPublisher("tcp://192.168.1.3:2096");
    ReferenceSubscriber referenceSubscriber("tcp://127.0.0.1:2069");
    TorqueCommandPublisher torqueCommandPublisher("tcp://127.0.0.1:5069");
}

namespace robotContext {
    // franka::Robot robot("192.168.0.1");
    // franka::Model model = robot.loadModel();
    DynamicsModelSubscriber model("tcp://127.0.0.1:3069");

}

//robot state, period, model 

franka::Torques JointPositionController::operator() (const franka::RobotState& robot_state, franka::Duration period){
    // coriolis compensation

    //reads reference signal every 10ms 
    if(count % 10 == 0) {
        torqueComms::referenceSubscriber.readMessage();
        q_goal = torqueComms::referenceSubscriber.jointAngles;
    }

    count++;
    // read current coriolis terms from model
//    std::array<double, DOF> coriolis = robotContext::model.coriolis(robot_state);
    // read gravity term from model (maybe we might want this?)
//    std::array<double, DOF> gravity = robotContext::model.gravity(robot_state);
    // mass
//    auto mass = robotContext::model.mass(robot_state);
    std::array<double, DOF> tau_d_calculated;

    double k_i_term = 0;

    for (size_t i = 0; i < DOF; i++) {
        // clamp goal joint position
        if(q_goal[i] < joint_min[i])
            q_goal[i] = joint_min[i];
        else if(q_goal[i] > joint_max[i])
            q_goal[i] = joint_max[i];

        auto q_error = q_goal[i] - robot_state.q[i];
        
        if(q_error <= 0.05) {
            integral[i] += period.toSec() * q_error;
            k_i_term = k_i[i];
        }
        else
            integral[i] = 0;
            k_i_term = 0;

        // tau_d_calculated[i] = k_s[i] * q_error - k_d[i] * robot_state.dq[i];

        tau_d_calculated[i] = 2* (
            1.9 * k_s[i] * (q_error) 
            - k_d[i] * robot_state.dq[i]); 
        // const std::array<double, DOF> k_s = {{700.0, 700.0, 700.0, 900.0, 450.0, 450.0, 300.0}};
        // const std::array<double, DOF> k_d = {{100.0, 100.0, 100.0, 100.0, 60.0, 50.0, 30.0}};

        // clamp torque
        if(tau_d_calculated[i] > torque_max[i])
            tau_d_calculated[i] = torque_max[i];
    }

    //for(int i = 0; i < 7; i++) 
        //std::cout << integral[i] << " " << std::endl;
    //std::cout << "\n" << std::endl;

    if((count-1)%4==0){
    std::array<double, DOF> jointBroadcast = {tau_d_calculated[0], tau_d_calculated[1], tau_d_calculated[2], tau_d_calculated[3],  
                                            tau_d_calculated[4], tau_d_calculated[5], tau_d_calculated[6]};
    
    torqueComms::torqueCommandPublisher.writeMessage(jointBroadcast);
    }
   
    // std::array<double, DOF> tau_d_rate_limited =
    //       franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, robot_state.tau_J_d);
    return tau_d_calculated;
} 

