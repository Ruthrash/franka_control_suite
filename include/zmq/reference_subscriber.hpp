#pragma once

#include <zmq.hpp>
#include <array>

class ReferenceSubscriber {
public:
    ReferenceSubscriber(std::string port);
    ReferenceSubscriber(const ReferenceSubscriber& listener);
    void readMessage();

    std::array<double, 9> jointAngles = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    zmq::context_t ctx_;
    zmq::socket_t socket_;
    bool messageChanged_;
    std::string port_;
};
