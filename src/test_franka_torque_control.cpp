#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>

#include "torque_controller.h"
#include "motion_controller.h"

// int main(int argc, char** argv) {
//     try {
//         std::cout << "moving robot to default position..." << std::endl;
//         std::array<double, 7> qRest = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
//         MotionGenerator motionGenerator(0.5, qRest);
//         robotContext::robot.control(motionGenerator);
//         std::cout << "finished moving robot to default position" << std::endl;

//         TorqueGenerator torqueController(1);
//         while(true) {
//             std::cout << "inside loop" << std::endl;
//             robotContext::robot.control(torqueController);
//         }

//     } catch (const franka::Exception& e) {
//         std::cout << e.what() << std::endl;
//         return -1;
//     }
// }