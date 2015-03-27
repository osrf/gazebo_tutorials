cmake_minimum_required(VERSION 2.8.3)
project(atlas_robot_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  atlas_msgs
  osrf_msgs
  rosgraph_msgs
  sensor_msgs
  std_msgs
)

find_package(orocos_kdl REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

if(NOT DEFINED ENV{ATLAS_ROBOT_INTERFACE_ROOT})
  message(FATAL_ERROR "You must set ATLAS_ROBOT_INTERFACE_ROOT to the directory
where you unpacked the AtlasRobotInterface distribution.")
endif()

# What version of the SDK do we have?
set(ATLAS_VERSION_MAJOR 3)
set(ATLAS_VERSION_MINOR 0)
set(ATLAS_VERSION_PATCH 0)
file(GLOB ATLAS_VERSION_FILE RELATIVE $ENV{ATLAS_ROBOT_INTERFACE_ROOT} $ENV{ATLAS_ROBOT_INTERFACE_ROOT}/AtlasRobotInterface-version-*)
if(NOT ${ATLAS_ROBOT_INTERFACE_ROOT})
  message(WARNING "Couldn't find AtlasRobotInterface-version-* file; assuming version is ${ATLAS_VERSION_MAJOR}.${ATLAS_VERSION_MINOR}.${ATLAS_VERSION_PATCH}")
else()
  string(REGEX REPLACE "AtlasRobotInterface-version-([0-9]*)\\.([0-9]*)\\.([0-9]*)" "\\1;\\2;\\3" ATLAS_VERSION ${ATLAS_VERSION_FILE})
  list(LENGTH ATLAS_VERSION ATLAS_VERSION_LENGTH)
  if(NOT ${ATLAS_VERSION_LENGTH} EQUAL 3)
    message(WARNING "Couldn't parse AtlasRobotInterface-version-* file name; assuming version is ${ATLAS_VERSION_MAJOR}.${ATLAS_VERSION_MINOR}.${ATLAS_VERSION_PATCH}")
  else()
    list(GET ATLAS_VERSION 0 ATLAS_VERSION_MAJOR)
    list(GET ATLAS_VERSION 1 ATLAS_VERSION_MINOR)
    list(GET ATLAS_VERSION 2 ATLAS_VERSION_PATCH)
    message(STATUS "Detected AtlasRobotInterface version: ${ATLAS_VERSION_MAJOR}.${ATLAS_VERSION_MINOR}.${ATLAS_VERSION_PATCH}")
  endif()
endif()

add_definitions(-DATLAS_VERSION_MAJOR=${ATLAS_VERSION_MAJOR} -DATLAS_VERSION_MINOR=${ATLAS_VERSION_MINOR} -DATLAS_VERSION_PATCH=${ATLAS_VERSION_PATCH})

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES atlas_interface
  CATKIN_DEPENDS roscpp atlas_msgs osrf_msgs rosgraph_msgs sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########
# find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  $ENV{ATLAS_ROBOT_INTERFACE_ROOT}/include
)

link_directories(
  $ENV{ATLAS_ROBOT_INTERFACE_ROOT}/lib64
)

add_library(atlas_interface src/atlas_interface.cpp)
target_link_libraries(atlas_interface ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} AtlasRobotInterface)
# AtlasRobotInterface apparently depends on zlib
target_link_libraries(atlas_interface z)

add_executable(example examples/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES} atlas_interface)

#############
## Install ##
#############
install(TARGETS atlas_interface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

# Install Gazebo Scripts
install(TARGETS example
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
