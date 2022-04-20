#include "context/context.h"


namespace robotContext {
    franka::Robot robot("192.168.0.1");
    franka::Gripper gripper("192.168.0.1");
    franka::Model model = robot.loadModel();
}

namespace commsContext {
    Subscriber subscriber(CommsDataType::DELTA_POSE_NULL_POSE, "tcp://192.168.1.2:2069");
    Publisher publisher("tcp://192.168.1.3:2096");
}
