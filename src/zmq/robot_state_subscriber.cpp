#include "zmq/robot_state_subscriber.hpp"
#include <iostream>
#include <zmq_addon.hpp>


RobotStateSubscriber::RobotStateSubscriber(std::string port) : socket_(ctx_, zmq::socket_type::sub) {
    int confl = 1;
    // socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(port);
    // socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = port;
    readMessage();
}

RobotStateSubscriber::RobotStateSubscriber(const RobotStateSubscriber& joint_listener) : socket_(ctx_, zmq::socket_type::sub){
    int confl = 1;
    // socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(joint_listener.port_);
    // socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = joint_listener.port_;
    readMessage();
}

void RobotStateSubscriber::readMessage() {
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    std::vector<zmq::message_t>  msgs;
    auto res = recv_multipart(socket_,std::back_inserter(msgs));
    int numValues = msgs[0].size() / sizeof(double);//joint angles
    for(int i = 0; i < numValues; i++){
        state_.q[i] = *(reinterpret_cast<double*>(msgs[0].data()) + i);
        std::cout<<state_.q[i]<<",";
    }std::cout<<" anlges\n";

    numValues = msgs[1].size() / sizeof(double);//joint velocities
    for(int i = 0; i < numValues; i++){
        state_.dq[i] = *(reinterpret_cast<double*>(msgs[1].data()) + i);
        std::cout<<state_.dq[i]<<","; 
    }std::cout<<" veloc\n";    

    numValues = msgs[2].size() / sizeof(double);//ee_frame
    for(int i = 0; i < numValues; i++){
        state_.O_T_EE[i] = *(reinterpret_cast<double*>(msgs[2].data()) + i);
        std::cout<<state_.O_T_EE[i]<<","; 
    }std::cout<<" veloc\n";    

    socket_.setsockopt(ZMQ_UNSUBSCRIBE, "", 0);

}
