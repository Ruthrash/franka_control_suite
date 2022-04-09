#pragma once

#include <functional>
#include <franka/robot.h>
#include "zmq/torque_command_publisher.hpp"
#include "zmq/robot_state_subscriber.hpp"

#include <chrono>
#include <thread>
namespace zmqComms{
    // extern TorqueCommandPublisher torqueCommandPublisher;
    extern RobotStateSubscriber robotStateSubscriber;
}

class SimLayer {

public:
    SimLayer();
    ~SimLayer();
    void loop(); 
    void control(std::function<franka::Torques(const franka::RobotState&, franka::Duration)> control_callback);
protected:

private: 

    franka::Torques command{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};


    // bool SimLayer::spinControl(const RobotState& robot_state,
    //                             franka::Duration time_step,
    //                             research_interface::robot::ControllerCommand* command)                


};