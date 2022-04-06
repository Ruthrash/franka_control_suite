#pragma once

#include "controllers/controllers_base.hpp"

#include "zmq/reference_subscriber.hpp"

namespace torqueComms {
    extern ReferenceSubscriber referenceSubscriber;
}


namespace robotContext {
    extern franka::Robot robot;
    extern franka::Model model;
}

class JointPositionController: public ControllersBase{

    JointPositionController(); 
    ~JointPositionController();

    franka::Torques operator() (const franka::RobotState&, franka::Duration); 
};

