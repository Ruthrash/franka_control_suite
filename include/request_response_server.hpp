# pragma once

#include <zmq.hpp>
#include <array>
#include <franka/robot_state.h>
#include <thread>

class ReqRepServer {

public:
    ReqRepServer();
    ReqRepServer(std::string port_); 
    ~ReqRepServer(); 
    /* sends column-major EE transformation matrix w.r.t base frame */
    void SendEEPose();
    void SetCurrrentEEPose(const std::array<double, 16>& EE_pose_);

protected:

private: 
    zmq::context_t ctx_;
    zmq::socket_t socket_;
    std::string port_;
    std::array<double, 16> current_EE_pose;
    std::thread server_thread;  
};