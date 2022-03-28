#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>

#include "torque_controller.h"

int main(int argc, char** argv) {
    try {
        std::cout<<"Before object"<<std::endl;
        TorqueGenerator torqueController(1);
        std::cout << "before loop" << std::endl;
        while(true) {
            std::cout << "inside loop" << std::endl;
            robotContext::robot.control(torqueController);
        }

    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
}