#pragma once

#include <zmq.hpp>
#include <vector>
#include <map>

enum class CommsDataType {
    JOINT_ANGLES,
    DELTA_POSE
};

const std::map<CommsDataType, int> typeLengths = {
    {CommsDataType::JOINT_ANGLES, 7},
    {CommsDataType::DELTA_POSE, 6}
}

class Listener {
public:
    const CommsDataType type;

    Listener(CommdsDataType type, std::string port);
    Listener(const Listener& listener);
    void readMessage();

    std::vector<double> values;
    zmq::context_t ctx;
    zmq::socket_t socket;
    std::string port;
};
