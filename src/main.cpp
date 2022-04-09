#include "sim_layer.hpp"
#include <iostream>
#include <franka/exception.h>
#include "controllers/joint_space/joint_position_controller.hpp"


int main(int argc, char** argv) {
    try {
        std::cout<<"Before object"<<std::endl;
        SimLayer simLayer; 
        JointPositionController torqueController(1);
        std::cout << "before loop" << std::endl;

            std::cout << "inside loop" << std::endl;
            simLayer.control(torqueController);


    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
}