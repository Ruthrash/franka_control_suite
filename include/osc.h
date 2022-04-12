#pragma once

#include <vector>
#include <array>
#include <string>

#include <eigen3/Eigen/Core>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>

#include "listener.h"
#include "publisher.h"

#include <memory>

namespace oscComms {
    extern Listener listener;
    extern Publisher publisher;
}

namespace oscRobotContext {
    extern franka::Robot robot;
    extern franka::Gripper grippper;
    extern franka::Model model;
}

class Osc {
public:
    Osc(int start, bool sendJoints);
    
    franka::Torques operator()(const franka::RobotState& robotState, franka::Duration period);

private:
    // stiffness
    const std::array<double, 6> k_s = {{40, 40, 40, 40, 40, 40}};
    // damping gain
    const std::array<double, 6> k_d = {{8, 8, 8, 8, 8, 8}};
    // torque limits
    const std::array<double, 7> torqueMax = {{87, 87, 87, 87, 12, 12, 12}};
    // count for receiving data
    size_t count;
    // pose delta
    std::array<double, 6> deltaPose;
    // whether joints angles or ee pose should be sent
    bool jointMessage;
    // gripper command
    std::array<double, 2> gripperCommand;
};