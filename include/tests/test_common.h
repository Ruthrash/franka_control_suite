#pragma once


#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <iterator>
#include <thread>
#include <atomic>
#include <fstream>


// Initialize data fields for the print thread.
struct {
    std::mutex mutex;
    bool has_data;
    std::array<double, 7> tau_d_last;
    franka::RobotState robot_state;
    std::array<double, 7> gravity;
} print_data{};

using StructType = decltype(print_data);
void log_data(const double& print_rate, StructType& print_data, std::atomic_bool& running, std::string& file_name, const bool&joint_space=true)
{
  std::fstream log_file(file_name);
  log_file.open(file_name);
  while (running) {
  
  // Sleep to achieve the desired print rate.
  std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        
        if (print_data.has_data) {  
    
          if(joint_space){
            for(size_t i =0; i < print_data.robot_state.q.size(); i++)
              log_file<<print_data.robot_state.q[i]<<",";
            for(size_t i =0; i < print_data.robot_state.dq.size(); i++)
              log_file<<print_data.robot_state.dq[i]<<",";           
            for(size_t i =0; i < print_data.robot_state.q_d.size(); i++)
              log_file<<print_data.robot_state.q_d[i]<<",";
            for(size_t i =0; i < print_data.robot_state.dq_d.size(); i++)
              log_file<<print_data.robot_state.dq_d[i]<<",";    
            log_file<<"\n"; 
          }
          else{

          }
          print_data.has_data = false;
        }
        print_data.mutex.unlock();
      }      
    
  }
  
  std::cout<<"closing\n";
  log_file.close();

}