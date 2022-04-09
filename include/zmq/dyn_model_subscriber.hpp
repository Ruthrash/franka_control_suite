#pragma once

#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <array>

class DynamicsModelSubscriber {
public:
    DynamicsModelSubscriber(std::string port);
    DynamicsModelSubscriber(const DynamicsModelSubscriber& listener);
    void readMessage();

    std::array<double, 9> jointAngles = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    zmq::context_t ctx_;
    zmq::socket_t socket_;
    bool messageChanged_;
    std::string port_;


protected:
    std::array<double, 7> coriolis(); 
    std::array<double, 7> gravity(); 
    std::array<double, 49> mass();


private:
    std::array<double, 7> coriolis_; 
    std::array<double, 7> gravity_comp_; 
    std::array<double, 49> inertia_matrix_; //column major 
    std::array<double, 7> joint_angles_;
};