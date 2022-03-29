#pragma once

#include "joint_listener.h"
#include <iostream>
#include <chrono>

JointListener::JointListener(std::string port) : socket_(ctx_, zmq::socket_type::sub) {
    int confl = 1;
    socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(port);
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = port;
    readMessage();
}

JointListener::JointListener(const JointListener& joint_listener) : socket_(ctx_, zmq::socket_type::sub){
    int confl = 1;
    socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(joint_listener.port_);
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = joint_listener.port_;
    readMessage();
}

void JointListener::readMessage() {
    zmq::message_t jointAnglesMessage;
    socket_.recv(&jointAnglesMessage);
 
    int numValues = jointAnglesMessage.size() / sizeof(double);
    assert(numValues == 9);

    for(int i = 0; i < numValues; i++) 
        jointAngles[i] = *(reinterpret_cast<double*>(jointAnglesMessage.data()) + i);
}
