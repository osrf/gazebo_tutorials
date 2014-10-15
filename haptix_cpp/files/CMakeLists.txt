cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find the Ignition_Transport library
find_package(ignition-transport QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")

include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

# Find the haptix_comm library
find_package(haptix_comm QUIET REQUIRED)
if (NOT haptix_comm_FOUND)
  BUILD_ERROR ("Missing: Haptix Comm (libhaptix-comm-dev)")
endif()
include_directories(${HAPTIX_COMM_INCLUDE_DIRS})
link_directories(${HAPTIX_COMM_LIBRARY_DIRS})

########################################
add_executable(hx_controller hx_controller.cpp)
target_link_libraries(hx_controller ${IGNITION-TRANSPORT_LIBRARIES} ${HAPTIX_COMM_LIBRARIES})
