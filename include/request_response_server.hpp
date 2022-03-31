# pragma once

#include <zmq.hpp>
#include <array>

class ReqRepServer {

public:
    ReqRepServer(); 
    ~ReqRepServer(); 
    /* sends 3D position + 4D quaternion */
    void SendEEPose(const std::array<double, 7>& EEPose);

protected:

private: 
    zmq::context_t ctx_;
    zmq::socket_t socket_;
    std::string port_;
};