#include "osc.h"

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

namespace oscComms {
    Listener listener(CommsDataType::DELTA_POSE, "tcp://192.168.1.2:2069");
    Publisher publisher("tcp://192.168.1.3:2096");
}

namespace oscRobotContext {
    franka::Robot robot("192.168.0.1");
    franka::Model model = robot.loadModel();
}

Osc::Osc(int start) {
    count = start;
    deltaPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

franka::Torques Osc::operator()(const franka::RobotState& robotState,
                                franka::Duration period) {
    // read command
    if(count % 10 == 0) {
        oscComms::listener.readMessage();
        for(size_t i = 0; i < 6; i++)
            deltaPose[i] = oscComms::listener.values[i];
    }
    // write state
    if((count - 1) % 4 == 0) {
        std::vector<double> jointBroadcast = {
            robotState.q[0], robotState.q[1], robotState.q[2], robotState.q[3],  
            robotState.q[4], robotState.q[5], robotState.q[6]
        };
    
        oscComms::publisher.writeMessage(jointBroadcast);
    }
    count++;

    // joint space mass matrix
    auto massArray = oscRobotContext::model.mass(robotState);
    // geometrix jacobian
    auto jacobianArray = oscRobotContex::model.zeroJacobian(
        franka::Frame::kEndEffector, robotState
    );
    // ee velocity
    auto eeVelocityArray = robotState.O_dP_EE_d;

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(massArray.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobianArray.data());

    // task space gains
    Eigen::Array<double, 6, 1> taskWrenchMotion;
    for(size_t i = 0; i < 6; i++)
        taskWrenchMotion[i] = k_s[i] * deltaPose[i] - k_d * eeVelocityArray[i];

    // task space mass 
    Eigen::Matrix<double, 6, 6> armMassMatrixTask = (
        jacobian * mass.inverse() * jacobian.transpose()
    ).inverse();

    // computed torque
    taskWrenchMotion = armMassMatrixTask * taskWrenchMotion.matrix();
    Eigen::VectorXd tau_d(7) = jacobian.transpose() * taskWrenchMotion.matrix();

    // convert back to std::array
    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
    
    return tau_d_array;
}
