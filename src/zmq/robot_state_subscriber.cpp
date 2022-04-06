#include "zmq/robot_state_subscriber.hpp"
#include <iostream>

RobotStateSubscriber::RobotStateSubscriber(std::string port) : socket_(ctx_, zmq::socket_type::sub) {
    int confl = 1;
    socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(port);
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = port;
    readMessage();
}

RobotStateSubscriber::RobotStateSubscriber(const RobotStateSubscriber& joint_listener) : socket_(ctx_, zmq::socket_type::sub){
    int confl = 1;
    socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(joint_listener.port_);
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = joint_listener.port_;
    readMessage();
}

void RobotStateSubscriber::readMessage() {
    zmq::message_t jointAnglesMessage;
    socket_.recv(&jointAnglesMessage);
 
    int numValues = jointAnglesMessage.size() / sizeof(double);
    assert(numValues == 9);

    for(int i = 0; i < numValues; i++) 
        jointAngles[i] = *(reinterpret_cast<double*>(jointAnglesMessage.data()) + i);
}
