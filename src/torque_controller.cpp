// #include "torque_controller.h"

// #include <algorithm>
// #include <array>
// #include <vector>
// #include <cmath>
// #include <iostream>
// #include <chrono>

// #include <franka/exception.h>
// #include <franka/robot.h>
// #include <franka/rate_limiting.h>

// namespace torqueComms {
//     JointListener jointListener("tcp://192.168.1.2:2069");
//     JointPublisher jointPublisher("tcp://192.168.1.3:2096");
    
// }

// namespace robotContext {
//     franka::Robot robot("192.168.0.1");
//     franka::Gripper gripper("192.168.0.1");
//     franka::Model model = robot.loadModel();
// }

// TorqueGenerator::TorqueGenerator(int start, bool useGripper) : useGripper(useGripper) {
//     count = start;
//     q_goal = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4, 0.04, 0.04}; 
// }

// TorqueGenerator::TorqueGenerator(const std::array<double, 9>& q_goal) : q_goal(q_goal) {
//     count = 1;
// }

// franka::Torques TorqueGenerator::operator()(const franka::RobotState& robot_state,
//                                                    franka::Duration period) {
//     // coriolis compensation
//     if(count % 17 == 0) {
//         // std::cout << "receiving message" << std::endl;
//         torqueComms::jointListener.readMessage();
//         q_goal = torqueComms::jointListener.jointAngles;

//         if(useGripper) {
//             double width = q_goal[7]+q_goal[8];
//             if(width > robotContext::gripper.readOnce().max_width)
//                 width = robotContext::gripper.readOnce().max_width;

//             robotContext::gripper.move(width, 0.1);

//             // if(q_goal[7] == 0.0) {
//             //     robotContext::gripper.move(0.0, 0.1);
//             // }
//             // else {
//             //     robotContext::gripper.move(robotContext::gripper.readOnce().max_width, 0.1);
//             // }
//         }
//     }
//     // std::cout << "COUNT " << count << std::endl;
//     // std::cout << "received goal config: " << std::endl;
//     // for(int i = 0 ; i < 9; i++)
//     //     std::cout << q_goal[i] << " ";
//     // std::cout << std::endl;

//     count++;
//     // read current coriolis terms from model
//     std::array<double, DOF> coriolis = robotContext::model.coriolis(robot_state);
//     // read gravity term from model (maybe we might want this?)
//     std::array<double, DOF> gravity = robotContext::model.gravity(robot_state);
//     // mass
//     auto mass = robotContext::model.mass(robot_state);
//     std::array<double, DOF> tau_d_calculated;

//     double k_i_term = 0;
//     for (size_t i = 0; i < DOF; i++) {
//         // clamp goal joint position
//         if(q_goal[i] < joint_min[i])
//             q_goal[i] = joint_min[i];
//         else if(q_goal[i] > joint_max[i])
//             q_goal[i] = joint_max[i];

//         auto q_error = q_goal[i] - robot_state.q[i];
        
//         if(q_error <= 0.05) {
//             integral[i] += period.toSec() * q_error;
//             k_i_term = k_i[i];
//         }
//         else
//             integral[i] = 0;
//             k_i_term = 0;

//         // tau_d_calculated[i] = k_s[i] * q_error - k_d[i] * robot_state.dq[i];

//         tau_d_calculated[i] = 2*0.1 * (
//             1.9 * k_s[i] * (q_error) 
//             - 2 * k_d[i] * robot_state.dq[i]); 
//         // const std::array<double, DOF> k_s = {{700.0, 700.0, 700.0, 900.0, 450.0, 450.0, 300.0}};
//         // const std::array<double, DOF> k_d = {{100.0, 100.0, 100.0, 100.0, 60.0, 50.0, 30.0}};

//         // clamp torque
//         if(tau_d_calculated[i] > 0.7 * torque_max[i])
//             tau_d_calculated[i] = 0.7 *  torque_max[i];
//     }
//     // std::cout << "Integral" << std::endl;
//     // for(int i = 0; i < 7; i++) 
//     //     std::cout << integral[i] << " " << std::endl;
//     // std::cout << "\n" << std::endl;

//     if((count-1)%4==0){
//         std::array<double, 14> jointBroadcast = {
//             robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3], robot_state.q[4], robot_state.q[5], robot_state.q[6],
//             robot_state.dq[0], robot_state.dq[1], robot_state.dq[2], robot_state.dq[3], robot_state.dq[4], robot_state.dq[5], robot_state.dq[6]

//             //0.42, 0.42
//             //robotContext::gripper.readOnce().width / 2, robotContext::gripper.readOnce().width / 2
//         };
        
//         torqueComms::jointPublisher.writeMessage(jointBroadcast);
//     }


//     std::array<double, DOF> tau_d_rate_limited =
//           franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, robot_state.tau_J_d);

//     franka::Torques output(tau_d_rate_limited);

//     return output;
// }
