#include "sim_layer.hpp"


SimLayer::SimLayer(){}

SimLayer::~SimLayer(){}

namespace zmqComms{
    TorqueCommandPublisher torqueCommandPublisher("tcp://192.168.1.2:2069");
    DynamicsModelSubscriber dynamicsModelSubscriber("tcp://192.168.1.2:2069");
    RobotStateSubscriber robotStateSubscriber("tcp://192.168.1.2:2069");
}

void SimLayer::control (std::function<franka::Torques(const franka::RobotState&, franka::Duration)> control_callback,
                        bool limit_rate,
                        double cutoff_frequency){

    //someone will set this
    while(!command.motion_finished){
        //call control callback at 1Khz, get torques and publish it to Orbit
        command = control_callback(current_state, current_time_step);
        //publish torque to orbit
        send();
    }
}



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






