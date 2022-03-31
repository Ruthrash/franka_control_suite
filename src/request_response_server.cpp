#include "request_response_server.hpp"
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <iostream>
ReqRepServer::ReqRepServer(){}
ReqRepServer::~ReqRepServer(){}

void ReqRepServer::SendEEPose(const std::array<double, 7>& EEPose){}

int main()
{
    return 0; 
    franka::Robot robot("192.168.0.1");
    while (true)
    {
        franka::RobotState current_state = robot.readOnce();
        std::cout<<"Current State: " << current_state.O_T_EE[0]<<std::endl;
    }
    
    

}
