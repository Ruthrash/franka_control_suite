#include "request_response_server.hpp"
#include ""

ReqRepServer::ReqRepServer(){}
ReqRepServer::~ReqRepServer(){}

void ReqRepServer::SendEEPose(const std::array<double, 7>& EEPose){}
#include <franka/robot.h>
#include <robot_state.h>

int main()
{
    return 0; 
    franka::Robot robot("192.168.0.1");
    while (true)
    {
        franka::RobotState current_state = robot.readOnce()
        std::cout<<
    }
    
    

}
