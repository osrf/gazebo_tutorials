cmake_minimum_required(VERSION 2.6)
project(SetVelPlugin)

find_package(gazebo REQUIRED)

include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS ${GAZEBO_CXX_FLAGS})

add_library(SetLinkVelocityPlugin SHARED src/SetLinkVelocityPlugin.cpp)
target_link_libraries(SetLinkVelocityPlugin ${GAZEBO_LIBRARIES})

add_library(SetJointVelocityPlugin SHARED src/SetJointVelocityPlugin.cpp)
target_link_libraries(SetJointVelocityPlugin ${GAZEBO_LIBRARIES})

