#include "zmq/reference_subscriber.hpp"
#include <iostream>

ReferenceSubscriber::ReferenceSubscriber(std::string port) : socket_(ctx_, zmq::socket_type::sub) {
    int confl = 1;
    socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(port);
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = port;
    readMessage();
}

ReferenceSubscriber::ReferenceSubscriber(const ReferenceSubscriber& joint_listener) : socket_(ctx_, zmq::socket_type::sub){
    int confl = 1;
    socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(joint_listener.port_);
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = joint_listener.port_;
    readMessage();
}

void ReferenceSubscriber::readMessage() {
    std::cout<<"waiting for message reference joint angles\n";
    zmq::message_t jointAnglesMessage;
    socket_.recv(&jointAnglesMessage);
 
    int numValues = jointAnglesMessage.size() / sizeof(double);
    assert(numValues == 9);

    for(int i = 0; i < numValues; i++){
        jointAngles[i] = *(reinterpret_cast<double*>(jointAnglesMessage.data()) + i);
        std::cout<<jointAngles[i]<<" ";
    }std::cout<<"reference\n";
}
