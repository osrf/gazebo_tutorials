# Tutorial: Ros Plugins

In this tutorial we'll walk through creating a very basic Gazebo plugin that is ROS-aware.

## Create a ROS Package

Create a new ROS package in your catkin workspace:

~~~
cd ~/catkin_ws/src
catkin_create_pkg gazebo_tutorials gazebo_ros roscpp
~~~

## Create the Plugin

Create a very simple plugin as described [here](/tutorials?tut=plugins_hello_world&cat=write_plugin) and save the file as `gazebo_tutorials/src/simple_world_plugin.cpp`:

~~~
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO("Hello World!");
  }

};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
~~~

## Update CMakeLists.txt

Open `gazebo_tutorials/CMakeLists.txt` and replace it with the following:

~~~
cmake_minimum_required(VERSION 2.8.3)
project(gazebo_tutorials)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

add_library(${PROJECT_NAME} src/simple_world_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
~~~

## Update package.xml

Update `gazebo_tutorials/package.xml` by adding the following line within the `<export></export>` tags (or add the `<export></export>` tags also).

~~~
  <gazebo_ros plugin_path="$(dirname $(catkin_find --first-only libgazebo_tutorials.so))" gazebo_media_path="${prefix}" />
~~~

## Compiling the Plugin

Build the plugin by going to the base of your work space and running catkin:

~~~
cd ~/catkin_ws
catkin_make
~~~

## Creating a World file

Save the following file as `gazebo_tutorials/worlds/hello.world`:

~~~
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- reference to your plugin -->
    <plugin name="gazebo_tutorials" filename="libgazebo_tutorials.so"/>
  </world>
</sdf>
~~~


## Create a Launch File

Create the following launch file gazebo_tutorials/launch/hello.launch:

~~~
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find gazebo_tutorials)/worlds/hello.world"/>
    <!-- more default parameters can be changed here -->
  </include>
</launch>
~~~

Before continuing source your new setup.*sh file:

~~~
source devel/setup.bash
~~~

## Run the Plugin

~~~
roslaunch gazebo_tutorials hello.launch
~~~

An empty Gazebo should open and in the terminal you should see it print out something like:

~~~
 INFO ros.gazebo_tutorials: Hello World!
~~~

## Starting from a Template

A template is available to help you quickly get a Gazebo-ROS plugin working:

[gazebo_ros_template.cpp](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_template.cpp)

## Adding Functionality

To make your plugin do something useful with Gazebo and ROS, we suggest you read the ROS-agnostic tutorials on [Plugins](/tutorials/?cat=write_plugin).

## ROS Node Note

All gazebo-ros plugins should check if the ROS node has already been initialized in their `Load()` function, as discussed in this [issue](http://answers.gazebosim.org/question/1493/rosinit-needed-for-ros-gazebo-plugin/). The initialization of the ROS node is performed automatically when you run

~~~
rosrun gazebo_ros gazebo
~~~

or use the generic empty.world launch file. The `gazebo_ros/src/gazebo_ros_api_plugin.cpp` should be the only place in Gazebo that calls `ros::init()`.

## Next Steps

For miscellaneous Gazebo-ROS tricks see [Advanced ROS integration](/tutorials/?tut=ros_advanced)
