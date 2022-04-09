#include "zmq/dyn_model_subscriber.hpp"
#include <iostream>
#include <zmq_addon.hpp>
/*
to do: 
Make abstract classes for pub, sub,  
*/



DynamicsModelSubscriber::DynamicsModelSubscriber(std::string port) : socket_(ctx_, zmq::socket_type::sub) {
    int confl = 1;
    //socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(port);
    //socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = port;
    readMessage();
}

DynamicsModelSubscriber::DynamicsModelSubscriber(const DynamicsModelSubscriber& joint_listener) : socket_(ctx_, zmq::socket_type::sub){
    int confl = 1;
    //socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(joint_listener.port_);
    //socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = joint_listener.port_;
    readMessage();
}

/*
Reads 3 msg parts of dynamics parameters
*/
void DynamicsModelSubscriber::readMessage() {
    //subscribe and unsubscribe to keep only the last multipart message 
    //conflate option lets you do this for single part messages
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    std::vector<zmq::message_t> msgs;
    auto res = recv_multipart(socket_,std::back_inserter(msgs));
    int numValues = msgs[0].size() / sizeof(double);//coriolis
    for(int i = 0; i < numValues; i++)
        coriolis_[i] = *(reinterpret_cast<double*>(msgs[0].data()) + i);

    numValues = msgs[1].size() / sizeof(double);//gravity
    for(int i = 0; i < numValues; i++)
        gravity_comp_[i] = *(reinterpret_cast<double*>(msgs[1].data()) + i);

    numValues = msgs[2].size() / sizeof(double);//inertia
    for(int i = 0; i < numValues; i++)
         inertia_matrix_[i] = *(reinterpret_cast<double*>(msgs[2].data()) + i);

    socket_.setsockopt(ZMQ_UNSUBSCRIBE, "", 0);
}


std::array<double, 7> DynamicsModelSubscriber::coriolis(){
    return coriolis_; 
}
std::array<double, 7> DynamicsModelSubscriber::gravity(){
    return gravity_comp_;
}
std::array<double, 49> DynamicsModelSubscriber::mass(){
    return inertia_matrix_;
}
