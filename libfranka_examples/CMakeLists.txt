cmake_minimum_required(VERSION 3.4)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

find_package(ZeroMQ QUIET)
find_package(cppzmq REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka)

set(EXAMPLES
  cartesian_impedance_control
  # communication_test
  # echo_robot_state
  # force_control
  # generate_cartesian_pose_motion
  # generate_cartesian_velocity_motion
  # generate_consecutive_motions
  # generate_elbow_motion
  # generate_joint_position_motion
  # generate_joint_velocity_motion
  # grasp_object
  # joint_impedance_control
  # joint_point_to_point_motion
  # motion_with_control
  # print_joint_poses
  # vacuum_object
)

set(SIMLAYER /home/ruthrash/utm/git/franka_control_suite/src/sim_layer.cpp)
file (GLOB_RECURSE ZMQ /home/ruthrash/utm/git/franka_control_suite/src/zmq/*.cpp)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp ${ZMQ} ${SIMLAYER}  )
  target_link_libraries(${example} Franka::Franka examples_common libzmq ${cppzmq} )
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads ${Franka})
target_link_libraries(motion_with_control Poco::Foundation ${Franka})

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
