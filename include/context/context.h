#pragma once

#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>

#include "communication/subscriber.h"
#include "communication/publisher.h"

namespace robotContext {
    extern franka::Robot robot;
    extern franka::Gripper gripper;
    extern franka::Model model;
}

namespace commsContext {
    extern Subscriber subscriber;
    extern Publisher publisher;
}