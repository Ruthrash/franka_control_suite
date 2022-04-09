#pragma once

#include <zmq.hpp>
#include <array>
#include <franka/robot.h>

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
public:
    franka::RobotState state_;
    int count;

};
