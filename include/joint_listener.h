#pragma once

#include <zmq.hpp>
#include <array>
#include <cmath>

class JointListener {
public:
    JointListener(std::string port);
    JointListener(const JointListener& listener);
    void readMessage();

    std::array<double, 9> jointAngles = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4, 0, 0}};
    //{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    zmq::context_t ctx_;
    zmq::socket_t socket_;
    bool messageChanged_;
    std::string port_;
};
