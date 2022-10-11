// https://frankaemika.github.io/libfranka/motion_with_control_8cpp-example.html
#include "tests/test_common.h"
#include "context.h"

namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;

}
namespace Comms {
    ActionSubscriber *actionSubscriber; 
    StatePublisher *statePublisher; 
}
int main(int argc, char *argv[]){
    return 0;
}