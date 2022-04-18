#pragma once

#include <zmq.hpp>
#include <vector>

class Publisher {
public:
    Publisher(std::string port);
    Publisher(const Publisher& publisher);
    void writeMessage(const std::vector<double>& data);

    zmq::context_t ctx;
    zmq::socket_t socket;
    std::string port;
};