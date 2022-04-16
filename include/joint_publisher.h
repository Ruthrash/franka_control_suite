#pragma once

#include <zmq.hpp>
#include <array>

class JointPublisher {
public:
    JointPublisher(std::string port);
    JointPublisher(const JointPublisher& publisher);
    void writeMessage(const std::array<double, 14>& jointAngles);

    zmq::context_t ctx_;
    zmq::socket_t socket_;
    std::string port_;
};