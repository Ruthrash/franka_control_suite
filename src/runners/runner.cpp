#include <franka/exception.h>
#include <iostream>
#include <thread>

#include "controllers/osc.h"
#include "controllers/torque_controller.h"
#include "controllers/motion_controller.h"
#include "context/context.h"


int main(int argc, char** argv) {
    try {
        bool useOSC = false;

        std::cout << "moving robot to default position..." << std::endl;
        std::array<double, 7> qRest = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motionGenerator(0.5, qRest);
        robotContext::robot.control(motionGenerator);
        std::cout << "finished moving robot to default position" << std::endl;

        if(useOSC) {
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
        else {
            commsContext::subscriber.setDataType(CommsDataType::JOINT_ANGLES);
            TorqueGenerator torqueController(1, false);
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


    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

}