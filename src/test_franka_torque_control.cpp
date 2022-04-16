// #include <franka/exception.h>
// #include <franka/robot.h>
// #include <iostream>

// #include "torque_controller.h"
// #include "motion_controller.h"

// int main(int argc, char** argv) {
//     try {
//         std::cout << "moving robot to default position..." << std::endl;
//         std::array<double, 7> qRest = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
//         MotionGenerator motionGenerator(0.5, qRest);
//         robotContext::robot.control(motionGenerator);
//         std::cout << "finished moving robot to default position" << std::endl;

//         TorqueGenerator torqueController(1, false);
//         while(true) {
//             robotContext::robot.control(torqueController);
//         }
//         // while(true) {
//         //     // std::cout << "inside loop" << std::endl;
//         //     // broadcast current state
//         //     franka::RobotState robotState = robotContext::robot.readOnce();
//         //     franka::GripperState gripperState = robotContext::gripper.readOnce();
//         //     double width = gripperState.width;
//         //     std::array<double, 9> jointBroadcast = {
//         //         robotState.q[0], robotState.q[1], robotState.q[2], robotState.q[3],  
//         //         robotState.q[4], robotState.q[5], robotState.q[6],
//         //         width / 2, width / 2
//         //     };
//         //     torqueComms::jointPublisher.writeMessage(jointBroadcast);
//         //     // move arm
//         //     // move gripper
//         //     robotContext::gripper.move(
//         //         torqueController.q_goal[7] + torqueController.q_goal[8], 0.1
//         //     );
//         //     // get goal
//         //     torqueController.count = 1;
//         //     torqueComms::jointListener.readMessage();
//         //     torqueController.q_goal = torqueComms::jointListener.jointAngles;
//         //     // std::cout << "received goal config: " << std::endl;
//         //     // for(int i = 0 ; i < 9; i++)
//         //     //     std::cout << torqueController.q_goal[i] << " " << std::endl;
//         //     std::cout << std::endl;
//         // }

//     } catch (const franka::Exception& e) {
//         std::cout << e.what() << std::endl;
//         return -1;
//     }
// }