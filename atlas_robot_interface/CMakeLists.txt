cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

if(NOT DEFINED ENV{ATLAS_ROBOT_INTERFACE_ROOT})
  message(FATAL_ERROR "You must set ATLAS_ROBOT_INTERFACE_ROOT to the directory
where you unpacked the AtlasRobotInterface distribution.")
endif()

# What version of the SDK do we have?
set(ATLAS_VERSION_MAJOR 2)
set(ATLAS_VERSION_MINOR 0)
set(ATLAS_VERSION_PATCH 2)
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

include_directories($ENV{ATLAS_ROBOT_INTERFACE_ROOT}/include)
link_directories($ENV{ATLAS_ROBOT_INTERFACE_ROOT}/lib64)

set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(atlas_interface src/atlas_interface.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(atlas_interface thread)
target_link_libraries(atlas_interface AtlasRobotInterface)
# AtlasRobotInterface apparently depends on zlib
target_link_libraries(atlas_interface z)

rosbuild_add_executable(example examples/example.cpp)
target_link_libraries(example atlas_interface)
