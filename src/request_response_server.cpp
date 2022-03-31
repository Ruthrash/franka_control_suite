#include "request_response_server.hpp"
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <iostream>



ReqRepServer::ReqRepServer(){}

ReqRepServer::~ReqRepServer(){}

ReqRepServer::ReqRepServer(std::string port) : socket_(ctx_, zmq::socket_type::rep){
    socket_.bind(port);
    port_ = port;
    server_thread = std::thread(&ReqRepServer::SendEEPose, this);
}

void ReqRepServer::SendEEPose(){
    while(true){
        zmq::message_t requestMessage;
        std::cout<<"Waiting 4 Request\n";
        socket_.recv(&requestMessage);
        std::cout<<"Message is: "<<requestMessage.to_string()<<std::endl;
        std::cout<<"Received Request\n";
        zmq::message_t currentEEPoseMsg(current_EE_pose);
        std::cout<<"Current State: " <<current_EE_pose[12]<<", "
                                    <<current_EE_pose[13]<<", "
                                    <<current_EE_pose[14]<<", "<<std::endl; 
        socket_.send(currentEEPoseMsg, zmq::send_flags::dontwait);
        std::cout<<"Sent response\n";
    }
    std::cout<<"Came out of infinite loop\n";
}

void ReqRepServer::SetCurrrentEEPose(const std::array<double, 16>& EE_pose){
    current_EE_pose = EE_pose;
}

namespace zqmComms {
    ReqRepServer reqrepServer("tcp://127.0.0.1:2000");
}

int main(int argc, char** argv)
{
    std::cout<<"Trying to connect\n";
    franka::Robot robot("192.168.0.1");
    while (true)
    {
        franka::RobotState current_state = robot.readOnce();
        zqmComms::reqrepServer.SetCurrrentEEPose(current_state.O_T_EE);
          
    }
    return 0;
}
