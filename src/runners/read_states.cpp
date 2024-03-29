// #include <iostream>
// #include <iterator>
// #include <string>

// #include <franka/exception.h>
// #include <franka/model.h>

// #include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/rnea.hpp"
// #include "pinocchio/multibody/model.hpp"


// #ifndef PINOCCHIO_MODEL_DIR
//   #define PINOCCHIO_MODEL_DIR "/home/snl/franka_dev/franka_control_suite/models/"
// #endif


// template <class T, size_t N>
// std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
//     ostream << "[";
//     std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
//     std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
//     ostream << "]";
//     return ostream;
// }

// int main(int argc, char** argv) {
//     if (argc != 2) {
//     std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
//     return -1;
//     }

//     try {
//     franka::Robot robot(argv[1]);

//     franka::RobotState state = robot.readOnce();

//     franka::Model model(robot.loadModel());
//     // for (franka::Frame frame = franka::Frame::kJoint1; frame <= franka::Frame::kEndEffector;
//     //      frame++) {
//     //   std::cout << model.pose(frame, state) << std::endl;
//     // }

//     auto gravityLibFranka = model.gravity(state);
//     std::cout << "Gravity from libfranka: " << gravityLibFranka << std::endl;

//     std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("franka_panda_with_gripper.urdf");
//     pinocchio::Model modelPin;
//     pinocchio::urdf::buildModel(urdf_filename, modelPin);

//     pinocchio::Data data(modelPin);
//     Eigen::VectorXd q(9);
//     q << state.q[0], state.q[1], state.q[2], state.q[3], state.q[4], state.q[5], state.q[6], 0, 0;
//     Eigen::VectorXd v = Eigen::VectorXd::Zero(modelPin.nv);
//     Eigen::VectorXd a = Eigen::VectorXd::Zero(modelPin.nv);
//     std::cout << pinocchio::rnea(modelPin, data, q, v, a) << std::endl;
    
//     // auto pin_gravity = data.g;
//     // std::cout << pin_gravity[0] << pin_gravity[1] << pin_gravity[2] << pin_gravity[3] << pin_gravity[4] << pin_gravity[5] << pin_gravity[6] << std::endl;

//     // pinocchio::forwardKinematics(modelPin, data, q);
//     } catch (franka::Exception const& e) {
//     std::cout << e.what() << std::endl;
//     return -1;
//     }


//     return 0;
// }