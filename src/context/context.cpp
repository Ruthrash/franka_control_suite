#include "context/context.h"


namespace robotContext {
    franka::Robot robot("172.16.0.2");
    // franka::Gripper gripper("172.16.0.2");
    franka::Model model = robot.loadModel();
}

namespace commsContext {
    //Subscriber subscriber(CommsDataType::DELTA_POSE_NULL_POSE, "tcp://192.168.1.2:2069");
    //Publisher publisher("tcp://192.168.1.3:2096");
    Subscriber subscriber(CommsDataType::DELTA_POSE_NULL_POSE, "tcp://192.168.2.42:2069");
    Publisher publisher("tcp://192.168.2.56:2096");
}
