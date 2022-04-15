#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>

#include "motion_controller.h"

// int main(int argc, char** argv) {
//     try {
//         franka::Robot robot("192.168.0.1");
//         franka::Gripper gripper("192.168.0.1");
//         franka::Model model = robot.loadModel();
//         td::cout << "moving robot to default position..." << std::endl;
//         std::array<double, 7> qRest = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
//         MotionGenerator motionGenerator(0.5, qRest, true);
//         robot.control(motionGenerator);
//         std::cout << "finished moving robot to default position" << std::endl;

//         auto robotState = robot.readOnce();
//         std::cout << "jacobian " 
//                 << model.zeroJacobian(franka::Frame::kEndEffector, robotState)
//                 << std::endl;
//         std::cout << "mass matrix "
//                 << model.mass(robotState) 
//                 <<std::endl;

//     } catch (const franka::Exception& e) {
//         std::cout << e.what() << std::endl;
//         return -1;
//     }
// }