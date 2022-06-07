#include <franka/exception.h>
#include <iostream>
#include <thread>

#include "controllers/osc.h"
#include "controllers/torque_controller.h"
#include "controllers/motion_controller.h"
#include "controllers/velocity_controller.h"
#include "context/context.h"


int main(int argc, char** argv) {
    try {
        std::string controlMode = "posCtrl";
        // bool useOSC = false;

        std::cout << "moving robot to default position..." << std::endl;
        std::array<double, 7> qRest = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motionGenerator(0.5, qRest);
        robotContext::robot.control(motionGenerator);

        franka::GripperState gripperState = robotContext::gripper.readOnce();
        double maxWidth = gripperState.max_width;
        robotContext::gripper.move(maxWidth, 0.2);

        std::cout << "finished moving robot to default position" << std::endl;

        // robotContext::robot.setCollisionBehavior(
        //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{120.0, 120.0, 118.0, 118.0, 116.0, 114.0, 112.0}},
        //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{120.0, 120.0, 118.0, 118.0, 116.0, 114.0, 112.0}},
        //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{120.0, 120.0, 120.0, 125.0, 125.0, 125.0}},
        //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{120.0, 120.0, 120.0, 125.0, 125.0, 125.0}}
        // );

        if(controlMode == "osc") {
            commsContext::subscriber.setDataType(CommsDataType::DELTA_POSE_NULL_POSE);
            Osc osc(1, true, false, false);
            // initialize subscriber thread
            std::thread subscribeThread([]() {
                while(true) {
                    commsContext::subscriber.readMessage();
                }
            });
            // run control loop
            while(true) {
                robotContext::robot.control(osc);
            }
        }
        else if(controlMode == "posCtrl") {
            // commsContext::subscriber.setDataType(CommsDataType::JOINT_ANGLES_VEL);
            commsContext::subscriber.setDataType(CommsDataType::JOINT_ANGLES_VEL_GRIPPER);
            std::cout << "before initialization" << std::endl;
            TorqueGenerator torqueController(1, true);
            std::cout << "before assignment" << std::endl;
            commsContext::subscriber.values[0] = 0;
            commsContext::subscriber.values[1] = -M_PI_4;
            commsContext::subscriber.values[2] = 0;
            commsContext::subscriber.values[3] = -3 * M_PI_4;
            commsContext::subscriber.values[4] = 0;
            commsContext::subscriber.values[5] = M_PI_2;
            commsContext::subscriber.values[6] = M_PI_4;
            std::cout << "after sub assignment" << std::endl;
            // initialize subscriber thread
            std::thread subscribeThread([]() {
                while(true) {
                    commsContext::subscriber.readMessage();
                }
            });
            // run control loop
            while(true) {
                robotContext::robot.control(torqueController);
            }
        }
        else if(controlMode == "velCtrl") {
            commsContext::subscriber.setDataType(CommsDataType::JOINT_ANGLES);
            VelocityGenerator velocityController(1, false);
            // initialize subscriber thread
            std::thread subscribeThread([]() {
                while(true) {
                    commsContext::subscriber.readMessage();
                }
            });
            // run control loop
            while(true) {
                robotContext::robot.control(velocityController);
            }
        }


    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

}
