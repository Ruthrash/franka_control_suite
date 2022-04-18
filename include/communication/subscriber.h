#pragma once

#include <zmq.hpp>
#include <vector>
#include <map>


enum class CommsDataType {
    JOINT_ANGLES,
    DELTA_POSE,
    DELTA_POSE_NULL_POSE,
    POSE,
    JOINT_ANGLES_GRIPPER,
    DELTA_POSE_GRIPPER,
    POSE_GRIPPER
};

inline std::map<CommsDataType, int> typeLengths = {
    {CommsDataType::JOINT_ANGLES, 7},
    {CommsDataType::DELTA_POSE, 6},
    {CommsDataType::DELTA_POSE_NULL_POSE, 6 + 7},
    {CommsDataType::POSE, 6},
    {CommsDataType::JOINT_ANGLES_GRIPPER, 7 + 1},
    {CommsDataType::DELTA_POSE_GRIPPER, 6 + 1},
    {CommsDataType::POSE_GRIPPER, 6 + 1},
};

class Subscriber {
public:
    CommsDataType type;

    Subscriber(CommsDataType dataType, std::string portId);
    Subscriber(const Subscriber& Subscriber);
    void readMessage();
    void setDataType(CommsDataType dataType);

    std::vector<double> values;
    zmq::context_t ctx;
    zmq::socket_t socket;
    std::string port;
};
