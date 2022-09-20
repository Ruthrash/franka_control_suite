#pragma once
#include "base_controller.h"

#include <eigen3/Eigen/Dense>
#include <franka/rate_limiting.h>


class JointPosition: public ControllersBase{

public:
    JointPosition(int start); 
    ~JointPosition();
    franka::Torques operator()(const franka::RobotState&, franka::Duration); 

private: 
    Eigen::VectorXd joint_goal_position;
    void clamp_joint_angles(std::vector<double> &joint_pos_goal);
    void clamp_torque(std::array<double, 7> &torque_array);
};