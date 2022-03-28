cmake_minimum_required(VERSION 3.10)
project(franka_control)

# minimum C++ version
set(CMAKE_CXX_STANDARD 17)

find_package(Franka 0.7.0 REQUIRED)
find_package(ZeroMQ QUIET)
find_package(cppzmq REQUIRED)

include_directories(${Franka_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${Franka_INCLUDE_DIRS})

# link libraries 


# define source files
file(GLOB_RECURSE FRANKA_CONTROL_SRC ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
add_executable(franka_control ${FRANKA_CONTROL_SRC})


target_link_libraries(franka_control PUBLIC Franka::Franka libzmq ${Franka} ${cppzmq} )


