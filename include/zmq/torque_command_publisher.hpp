#pragma once

#include <zmq.hpp>
#include <array>

class TorqueCommandPublisher {
public:
    TorqueCommandPublisher(std::string port);
    TorqueCommandPublisher(const TorqueCommandPublisher& publisher);
    void writeMessage(const std::array<double, 7>& torqueCommands);

    zmq::context_t ctx_;
    zmq::socket_t socket_;
    std::string port_;
};