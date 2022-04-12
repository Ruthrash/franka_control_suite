#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>

#include "osc.h"

int main(int argc, char** argv) {
    try {
        Osc osc(2);
        while(true) {
            robotContext::robot.control(osc);
        }

    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
}