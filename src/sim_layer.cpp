#include "sim_layer.hpp"
#include <chrono>
#include <iostream>

SimLayer::SimLayer(){}

SimLayer::~SimLayer(){}

namespace zmqComms{
    // TorqueCommandPublisher torqueCommandPublisher("tcp://127.0.0.1:5069");
    RobotStateSubscriber robotStateSubscriber("tcp://127.0.0.1:4069");
}

void SimLayer::control (std::function<franka::Torques(const franka::RobotState&, franka::Duration)> control_callback){
    while(1){
    auto t_start = std::chrono::high_resolution_clock::now();
    zmqComms::robotStateSubscriber.readMessage();
    franka::Duration dur; 
    auto res = control_callback(zmqComms::robotStateSubscriber.state_, dur);
    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
    std::cout<<elapsed_time_ms<< " run time\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    }
                            
                            /*state_.dq[i], state_.q[i], state_.tau_J_d*/
}

franka::RobotState SimLayer::readOnce(){
    return zmqComms::robotStateSubscriber.state_;
} 

void SimLayer::loop()
{
    zmqComms::robotStateSubscriber.readMessage();
}

    //someone will set this
    // while(!command.motion_finished){
    //     //call control callback at 1Khz, get torques and publish it to Orbit
    //     command = control_callback(current_state, current_time_step);
    //     //publish torque to orbit
    //     send();
    // }




// bool SimLayer::spinControl(const RobotState& robot_state,
//                                  franka::Duration time_step,
//                                  research_interface::robot::ControllerCommand* command) {
//   Torques control_output = control_callback_(robot_state, time_step);
//   if (cutoff_frequency_ < kMaxCutoffFrequency) {
//     for (size_t i = 0; i < 7; i++) {
//       control_output.tau_J[i] = lowpassFilter(kDeltaT, control_output.tau_J[i],
//                                               robot_state.tau_J_d[i], cutoff_frequency_);
//     }
//   }
//   if (limit_rate_) {
//     control_output.tau_J = limitRate(kMaxTorqueRate, control_output.tau_J, robot_state.tau_J_d);
//   }
//   command->tau_J_d = control_output.tau_J;
//   checkFinite(command->tau_J_d);
//   return !control_output.motion_finished;
// }






