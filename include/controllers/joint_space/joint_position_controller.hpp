#pragma once

#include "controllers/controllers_base.hpp"

#include "zmq/reference_subscriber.hpp"
#include "zmq/dyn_model_subscriber.hpp"

namespace torqueComms {
    extern ReferenceSubscriber referenceSubscriber;
}


namespace robotContext {
    // extern franka::Robot robot;
    // extern franka::Model model;
    extern DynamicsModelSubscriber model;
}

class JointPositionController: public ControllersBase{


public:
    JointPositionController(int start); 
    ~JointPositionController();
    franka::Torques operator()(const franka::RobotState&, franka::Duration); 

private:
    std::array<double, 9> q_goal;
};

