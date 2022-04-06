#pragma once


#include <franka/robot.h>
#define DOF  7



class ControllersBase{

public:
    /* virtual destructor to prevent memory leaks */
    virtual ~ControllersBase();

protected: 
    //virtual franka::Torques control_callback(const franka::RobotState&, franka::Duration) = 0; 
    // stiffness gain
    // const std::array<double, DOF> k_s = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    const std::array<double, DOF> k_s = {{700.0, 700.0, 700.0, 700.0, 291.67, 175.0, 58.33}};
    
    // damping gain
    const std::array<double, DOF> k_d = {{100.0, 100.0, 100.0, 100.0, 60.0, 50.0, 30.0}};
    // const std::array<double, DOF> k_d = {{100.0, 100.0, 100.0, 100.0, 30.0, 25.0, 15.0}};
    // integral gain
    const std::array<double, DOF> k_i = {{100.0, 100.0, 100.0, 200.0, 120.0, 100.0, 30.0}};
    // integral
    std::array<double, DOF> integral = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    // const std::array<double, DOF> k_d = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
    // joint limits
    const std::array<double, DOF> joint_min = {{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
    const std::array<double, DOF> joint_max = {{2.8973, 1.7628 	, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};
    // torque limits
    const std::array<double, DOF> torque_max = {{87, 87, 87, 87, 12, 12, 12}};
    // calculated torque output
    std::array<double, DOF> torque_output = {{0, 0, 0, 0, 0, 0, 0}};
    // count for receiving data 
    size_t count;
};



ControllersBase::~ControllersBase(){}