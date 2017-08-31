cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("use_logical_camera")

find_package(gazebo REQUIRED)
# Add gazebo directories/flags to the entire project
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Create a gazebo plugin that gets data directly
add_library(DataViaPlugin SHARED DataViaPlugin.cc)
target_link_libraries(DataViaPlugin ${GAZEBO_LIBRARIES})

# Create a standalone executable that gets data using gazebo transport
add_executable(DataViaTransport DataViaTransport.cc)
target_link_libraries(DataViaTransport ${GAZEBO_LIBRARIES})
