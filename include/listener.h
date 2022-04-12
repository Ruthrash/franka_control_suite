#pragma once

#include <zmq.hpp>
#include <vector>
#include <map>

enum class CommsDataType {
    JOINT_ANGLES,
    DELTA_POSE,
    POSE,
    JOINT_ANGLES_GRIPPER,
    DELTA_POSE_GRIPPER,
    POSE_GRIPPER
};

inline std::map<CommsDataType, int> typeLengths = {
    {CommsDataType::JOINT_ANGLES, 7},
    {CommsDataType::DELTA_POSE, 6},
    {CommsDataType::POSE, 6},
    {CommsDataType::JOINT_ANGLES_GRIPPER, 7 + 2},
    {CommsDataType::DELTA_POSE_GRIPPER, 6 + 2},
    {CommsDataType::POSE_GRIPPER, 6 + 2},
};

class Listener {
public:
    CommsDataType type;

    Listener(CommsDataType dataType, std::string portId);
    Listener(const Listener& listener);
    void readMessage();

    std::vector<double> values;
    zmq::context_t ctx;
    zmq::socket_t socket;
    std::string port;
};
