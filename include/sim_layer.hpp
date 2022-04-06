#pragma once

#include <functional>
#include <franka/robot.h>
#include "zmq/torque_command_publisher.hpp"
#include "zmq/dyn_model_subscriber.hpp"
#include "zmq/robot_state_subscriber.hpp"

namespace zmqComms{
    extern TorqueCommandPublisher torqueCommandPublisher;
    extern DynamicsModelSubscriber dynamicsModelSubscriber;
    extern RobotStateSubscriber robotStateSubscriber;
}

class SimLayer {

public:
    SimLayer();
    ~SimLayer(); 

protected:

private: 
    franka::Torques command; 
    void control(std::function<franka::Torques(const franka::RobotState&, franka::Duration)> control_callback,
                bool limit_rate,
                double cutoff_frequency);

    // bool SimLayer::spinControl(const RobotState& robot_state,
    //                             franka::Duration time_step,
    //                             research_interface::robot::ControllerCommand* command)                


};