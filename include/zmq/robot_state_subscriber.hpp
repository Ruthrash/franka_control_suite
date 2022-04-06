#pragma once

#include <zmq.hpp>
#include <array>

class RobotStateSubscriber {
public:
    RobotStateSubscriber(std::string port);
    RobotStateSubscriber(const RobotStateSubscriber& listener);
    void readMessage();

    std::array<double, 9> jointAngles = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    zmq::context_t ctx_;
    zmq::socket_t socket_;
    bool messageChanged_;
    std::string port_;
};
