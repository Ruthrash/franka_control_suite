#include "osc.h"

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>


namespace oscComms {
    Listener listener(CommsDataType::DELTA_POSE, "tcp://192.168.1.2:2069");
    Publisher publisher("tcp://192.168.1.3:2096");
}

namespace oscRobotContext {
    franka::Robot robot("192.168.0.1");
    franka::Gripper gripper("192.168.0.1");
    franka::Model model = robot.loadModel();
}

Osc::Osc(int start, bool sendJoints, bool nullspace) {
    count = start;
    jointMessage = sendJoints;
    deltaPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    gripperCommand = -100.0;
    useNullspace = nullspace;
    restPose << 0.0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;
    nullGain << 1, 1, 1, 1, 1, 1, 1;
    nullWeight << 1, 1, 1, 1, 1, 1, 1;
}

franka::Torques Osc::operator()(const franka::RobotState& robotState,
                                franka::Duration period) {
    // read command
    if(count % 10 == 0) {
        oscComms::listener.readMessage();
        for(size_t i = 0; i < 6; i++)
            deltaPose[i] = oscComms::listener.values[i];
        // check for gripper command
        if(
            oscComms::listener.type == CommsDataType::DELTA_POSE_GRIPPER || 
            oscComms::listener.type == CommsDataType::POSE_GRIPPER) {
                if(
                    fabs(oscComms::listener.values[6] - gripperCommand) > 0.01 
                ) {
                    gripperCommand = oscComms::listener.values[6];
                    // stop the current gripper movement
                    oscRobotContext::gripper.stop();
                }
            }
    }
    // write state
    if((count - 1) % 4 == 0) {
        if(jointMessage) {
            std::vector<double> jointBroadcast = {
                robotState.q[0], robotState.q[1], robotState.q[2], robotState.q[3],  
                robotState.q[4], robotState.q[5], robotState.q[6]
            };
        
            oscComms::publisher.writeMessage(jointBroadcast);
        }
        else {
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robotState.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());      
            Eigen::Quaterniond orientation(transform.linear());
            std::vector<double> poseBroadcast(9);
            // Eigen::Map<const Eigen::Vector3d>(poseBroadcast.data(), position.rows(), position.cols()) = position;
            poseBroadcast[0] = position(0);
            poseBroadcast[1] = position(1);
            poseBroadcast[2] = position(2);
            poseBroadcast[3] = orientation.x();
            poseBroadcast[4] = orientation.y();
            poseBroadcast[5] = orientation.z();
            poseBroadcast[6] = orientation.w();
            ranka::GripperState gripperState = oscRobotContext::gripper.readOnce();
            double gripperWidth = gripperState.width;
            poseBroadcast[7] = width / 2;
            poseBroadcast[8] = width / 2;
            oscComms::publisher.writeMessage(poseBroadcast);
        }
    }
    count++;

    // joint space mass matrix
    auto massArray = oscRobotContext::model.mass(robotState);
    // geometrix jacobian
    auto jacobianArray = oscRobotContext::model.zeroJacobian(
        franka::Frame::kEndEffector, robotState
    );
    // joint velocity
    auto jointVelocityArray = robotState.dq;

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(massArray.data());
    Eigen::Matrix<double, 7, 7> massInv = mass.inverse();
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobianArray.data());
    Eigen::Matrix<double, 7, 6> jacobianT = jacobian.transpose();
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> jointVelocity(jointVelocityArray.data());

    // add extra mass to the mass matrix
    // mass(4, 4) += 0.10;
    // mass(5, 5) += 0.10;
    // mass(6, 6) += 0.10;
    
    // ee velocity
    Eigen::Array<double, 6, 1> eeVelocityArray = (jacobian * jointVelocity).array();
    // auto eeVelocityArray = robotState.O_dP_EE_d;

    // task space gains
    Eigen::Array<double, 6, 1> taskWrenchMotion;
    for(size_t i = 0; i < 6; i++)
        taskWrenchMotion[i] = k_s[i] * deltaPose[i] - k_d[i] * eeVelocityArray[i];

    // task space mass 
    Eigen::Matrix<double, 6, 6> armMassMatrixTask = (
        jacobian * mass.inverse() * jacobianT
    ).inverse();

    // computed torque
    taskWrenchMotion = (armMassMatrixTask * taskWrenchMotion.matrix()).array();
    Eigen::VectorXd tau_d = jacobianT * taskWrenchMotion.matrix();

    // nullspace control
    if(useNullspace) {
        // inertia weighted pseudoinverse
        EigenMatrix<double, 7, 6> inertiaWeightedPInv = (
            massInv * jacobianT * armMassMatrixTask
        );
        // nullspace projector
        Eigen::Matrix<double, 7, 7> projector = (
            Eigen::MatrixXd::Identity(7, 7) - jacobianT * inertiaWeightedPInv.transpose()
        );
        // robot state
        Eigen::Map<const Eigen::Array<double, 7, 1>> jointPos(robotState.q);
        Eigen::Map<const Eigen::Array<double, 7, 1>> jointVel(robotState.dq);
        // nullspace torque action
        Eigen::Array<double, 7, 1> tauNull = (
            -1 * nullGain * jointVel - alpha * nullWeight * (jointPos - jointVel);
        );
        // apply nullspace action
        taskWrenchMotion += (
            projector * tauNull.matrix()
        ).array();
    }

    // convert back to std::array
    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

    // gripper command
    if(gripperCommand[0] != -100)
        oscRobotContext::gripper.move(gripperCommand, 0.1);
    
    return tau_d_array;
}
