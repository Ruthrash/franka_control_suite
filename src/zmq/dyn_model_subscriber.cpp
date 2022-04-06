#include "zmq/dyn_model_subscriber.hpp"
#include <iostream>

DynamicsModelSubscriber::DynamicsModelSubscriber(std::string port) : socket_(ctx_, zmq::socket_type::sub) {
    int confl = 1;
    socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(port);
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = port;
    readMessage();
}

DynamicsModelSubscriber::DynamicsModelSubscriber(const DynamicsModelSubscriber& joint_listener) : socket_(ctx_, zmq::socket_type::sub){
    int confl = 1;
    socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket_.connect(joint_listener.port_);
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port_ = joint_listener.port_;
    readMessage();
}

/*
Reads 3 msg parts of dynamics parameters
*/
void DynamicsModelSubscriber::readMessage() {
    zmq::message_t coriolis_msg, gravity_msg, inertia_msg, j_angle_msg;
    socket_.recv(coriolis_msg, zmq::recv_flags::none);

    if(!coriolis_msg.more()){
        std::cout<<"Something wrong\n";
    }
    else{
        int numValues = coriolis_msg.size() / sizeof(double);
        assert(numValues == 7);
        
        for(int i = 0; i < numValues; i++) 
            coriolis_[i] = *(reinterpret_cast<double*>(coriolis_msg.data()) + i);
        
        socket_.recv(gravity_msg, zmq::recv_flags::none); 
        if(!gravity_msg.more()){
            std::cout<<"Something went wrong\n"; 
        }
        else{
            int numValues = gravity_msg.size() / sizeof(double);
            assert(numValues == 7);
            for(int i = 0; i < numValues; i++) 
                gravity_comp_[i] = *(reinterpret_cast<double*>(gravity_msg.data()) + i);
            socket_.recv(inertia_msg, zmq::recv_flags::none); 
            numValues = inertia_msg.size() / sizeof(double);
            assert(numValues == 49);
            for(int i = 0; i < numValues; i++) 
                inertia_matrix_[i] = *(reinterpret_cast<double*>(inertia_msg.data()) + i);
        }
    }
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
