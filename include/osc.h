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
    extern franka::Model model;
}

class Osc {
public:
    Osc(int start);
    
    franka::Torques operator()(const franka::RobotState& robotState, franka::Duration period);

private:
    // stiffness
    const std::array<double, 6> k_s = {{700.0, 700.0, 700.0, 700.0, 291.67, 175.0}};
    // damping gain
    const std::array<double, 6> k_d = {{100.0, 100.0, 100.0, 100.0, 60.0, 50.0}};
    // torque limits
    const std::array<double, 7> torque_max = {{87, 87, 87, 87, 12, 12, 12}};
    // count for receiving data
    size_t count;
    // pose delta
    std::array<double, 6> deltaPose;

    
};