#include "controllers/osc.h"
#include "context/context.h"

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>


// namespace oscComms {
//     Listener listener(CommsDataType::DELTA_POSE_NULL_POSE, "tcp://192.168.1.2:2069");
//     Publisher publisher("tcp://192.168.1.3:2096");
// }

// namespace oscRobotContext {
//     franka::Robot robot("192.168.0.1");
//     franka::Gripper gripper("192.168.0.1");
//     franka::Model model = robot.loadModel();
// }

Osc::Osc(int start, bool sendJoints, bool nullspace, bool coriolis) {
    count = start;
    jointMessage = sendJoints;
    jointMessage = true;
    deltaPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    gripperCommand = -100.0;

    useNullspace = nullspace;
    restPose << 0.0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;
    nullGain << 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02;
    nullWeight << 0.7, 0.7, 0.7, 0.7, 0.3, 0.18, 0.01;

    coriolisCompensation = coriolis;
    auto robotState = robotContext::robot.readOnce();
    prevJacobian = Eigen::Map<Eigen::Matrix<double, 6, 7>>(
        robotContext::model.zeroJacobian(
            franka::Frame::kEndEffector, robotState
        ).data()
    );
}

franka::Torques Osc::operator()(const franka::RobotState& robotState,
                                franka::Duration period) {
    // read command
    if(count % 10 == 0) {
        commsContext::subscriber.readMessage();
        for(size_t i = 0; i < 6; i++)
            deltaPose[i] = commsContext::subscriber.values[i];
        if(commsContext::subscriber.type == CommsDataType::DELTA_POSE_NULL_POSE) {
            for(size_t i = 0; i < 7; i++)
                restPose[i] = commsContext::subscriber.values[6+i];
        }

        // std::cout << "nullspace target ";
        // for(size_t i = 0; i < 7; i++)
        //     std::cout << restPose[i];
        // std::cout << std::endl;

        // check for gripper command
        // if(
        //     commsContext::listener.type == CommsDataType::DELTA_POSE_GRIPPER || 
        //     commsContext::listener.type == CommsDataType::POSE_GRIPPER) {
        //         if(
        //             fabs(commsContext::listener.values[6] - gripperCommand) > 0.01 
        //         ) {
        //             gripperCommand = commsContext::listener.values[6];
        //             // stop the current gripper movement
        //             robotContext::gripper.stop();
        //         }
        //     }
    }
    // write state
    if((count - 1) % 4 == 0) {
        if(jointMessage) {
            std::vector<double> jointBroadcast = {
                robotState.q[0], robotState.q[1], robotState.q[2], robotState.q[3],  
                robotState.q[4], robotState.q[5], robotState.q[6]
            };
            // std::cout << "sending message" << std::endl;
            commsContext::publisher.writeMessage(jointBroadcast);
        }
        else {
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robotState.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());      
            Eigen::Quaterniond orientation(transform.linear());
            std::vector<double> poseBroadcast(7);
            // Eigen::Map<const Eigen::Vector3d>(poseBroadcast.data(), position.rows(), position.cols()) = position;
            poseBroadcast[0] = position(0);
            poseBroadcast[1] = position(1);
            poseBroadcast[2] = position(2);
            poseBroadcast[3] = orientation.x();
            poseBroadcast[4] = orientation.y();
            poseBroadcast[5] = orientation.z();
            poseBroadcast[6] = orientation.w();
            // franka::GripperState gripperState = robotContext::gripper.readOnce();
            // double gripperWidth = gripperState.width;
            // poseBroadcast[7] = gripperWidth / 2;
            // poseBroadcast[8] = gripperWidth / 2;
            commsContext::publisher.writeMessage(poseBroadcast);
        }
    }
    count++;

    // joint space mass matrix
    auto massArray = robotContext::model.mass(robotState);
    // geometrix jacobian
    auto jacobianArray = robotContext::model.zeroJacobian(
        franka::Frame::kEndEffector, robotState
    );
    // joint velocity
    auto jointVelocityArray = robotState.dq;

    // robot state
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> jointPos(robotState.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> jointVel(jointVelocityArray.data());
    
    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(massArray.data());
    Eigen::Matrix<double, 7, 7> massInv = mass.inverse();
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobianArray.data());
    Eigen::Matrix<double, 7, 6> jacobianT = jacobian.transpose();

    // add extra mass to the mass matrix
    mass(4, 4) += 0.20;
    mass(5, 5) += 0.20;
    mass(6, 6) += 0.20;
    
    // ee velocity
    Eigen::Array<double, 6, 1> eeVelocityArray = (jacobian * jointVel).array();
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

    // // nullspace control
    if(useNullspace) {
        // inertia weighted pseudoinverse
        Eigen::Matrix<double, 7, 6> inertiaWeightedPInv = (
            massInv * jacobianT * armMassMatrixTask
        );
        // nullspace projector
        Eigen::Matrix<double, 7, 7> projector = (
            Eigen::MatrixXd::Identity(7, 7) - jacobianT * inertiaWeightedPInv.transpose()
        );
        // nullspace torque action
        Eigen::Array<double, 7, 1> tauNull = (
            0 * -1 * nullGain * jointVel.array() - alpha * nullWeight * (jointPos.array() - restPose)
        );
        
        auto addOn = projector * tauNull.matrix();
        std::cout << "=====================================================" << std::endl;
        std::cout << "Null space target: ";
        for(int i = 0; i < 7; i++) 
            std::cout << restPose[i] << " ";
        std::cout << std::endl;
        std::cout << "Null space torques: ";
        for(int i = 0; i < 7; i++) 
            std::cout << addOn[i] << " ";
        std::cout << std::endl;
        std::cout << "Joint position: ";
        for(int i = 0; i < 7; i++) 
            std::cout << robotState.q[i] << " ";
        std::cout << std::endl;

        // apply nullspace action
        tau_d += (
            projector * tauNull.matrix()
        );
    }

    // // coriolis compensation
    // if(coriolisCompensation) {
    //     // calculate jacobian using finite difference
    //     Eigen::Matrix<double, 6, 7> jacobianDerivative = (jacobian - prevJacobian) / 0.001; 
    //     prevJacobian = jacobian;
    //     // joint space coriolis
    //     Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
    //         robotContext::model.coriolis(robotState).data()
    //     );
    //     // apply coriolis compensation
    //     tau_d += (
    //         coriolis - (jacobianT * armMassMatrixTask * jacobianDerivative * jointVel.matrix())
    //     );
    // }

    // convert back to std::array
    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

    for(size_t i = 0; i < 7; i++) {
        if(tau_d_array[i] > 0.7 * torqueMax[i])
            tau_d_array[i] = 0.7 *  torqueMax[i];
    }

    // gripper command
    // if(gripperCommand != -100)
    //     robotContext::gripper.move(gripperCommand, 0.1);
    
    return tau_d_array;
}
