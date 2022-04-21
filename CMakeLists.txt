cmake_minimum_required(VERSION 3.10)
project(franka_control)

# minimum C++ version
set(CMAKE_CXX_STANDARD 17)

find_package(Franka 0.7.0 REQUIRED)
find_package(ZeroMQ QUIET)
find_package(cppzmq REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(${Franka_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${Franka_INCLUDE_DIRS})

# link libraries 


# define source files
file(GLOB_RECURSE FRANKA_CONTROL_SRC ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp ${CMAKE_CURRENT_LIST_DIR}/src/controllers/joint_space/joint_position_controller.cpp 
 ${CMAKE_CURRENT_LIST_DIR}/src/controllers/zmq/*.cpp
)
add_executable(franka_control ${FRANKA_CONTROL_SRC})



target_link_libraries(franka_control PUBLIC Franka::Franka libzmq ${Franka} ${cppzmq} )


####build libfranka examples
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(EXAMPLES_DIR ${CMAKE_CURRENT_LIST_DIR}/libfranka_examples)
find_package(Threads REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)


add_library(examples_common STATIC
  ${EXAMPLES_DIR}/examples_common.cpp
)
add_library(simlayer ${CMAKE_CURRENT_LIST_DIR}/src/sim_layer.cpp)
file (GLOB_RECURSE ZMQ ${CMAKE_CURRENT_LIST_DIR}/src/zmq/*.cpp)


target_link_libraries(examples_common PUBLIC Franka::Franka)
set(EXAMPLES
  cartesian_impedance_control
  #joint_impedance_control  
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
  # joint_point_to_point_motion
  # motion_with_control
  # print_joint_poses
  # vacuum_object
)
file (GLOB_RECURSE ZMQ ${CMAKE_CURRENT_LIST_DIR}/src/zmq/*.cpp)

foreach(example ${EXAMPLES})
  add_executable(${example} ${EXAMPLES_DIR}/${example}.cpp ${ZMQ} ${Franka}   )
  target_link_libraries(${example} Franka::Franka examples_common simlayer libzmq ${cppzmq} )
endforeach()
# target_link_libraries(joint_impedance_control Threads::Threads)
# target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)