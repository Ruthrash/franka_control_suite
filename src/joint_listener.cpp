#include "joint_listener.h"
#include <iostream>
#include <chrono>

JointListener::JointListener(std::string port) : socket_(ctx_, zmq::socket_type::sub) {
    int confl = 1;
    socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(port);
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = port;
    jointAngles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    readMessage();
}

JointListener::JointListener(const JointListener& joint_listener) : socket_(ctx_, zmq::socket_type::sub){
    socket_.connect(joint_listener.port_);
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = joint_listener.port_;
    jointAngles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    readMessage();
}

void JointListener::readMessage() {
    zmq::message_t jointAnglesMessage;
    socket_.recv(&jointAnglesMessage);
 
    int numValues = jointAnglesMessage.size() / sizeof(double);

    for(int i = 0; i < numValues; i++) {
        double val = *reinterpret_cast<double*>(jointAnglesMessage.data()+i*sizeof(double));
        jointAngles[i] = val;
    }
}
