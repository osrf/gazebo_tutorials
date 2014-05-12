# Tutorial: Ros Plugins

In this tutorial we'll walk through creating a very basic Gazebo plugin that is ROS-aware.

## Create a ROS Package

Create a new ROS package in your catkin workspace:

<pre>
cd ~/catkin_ws
catkin_create-pkg gazebo_tutorials gazebo_ros roscpp
</pre>

## Create the Plugin

Create a very simple plugin as described [http://gazebosim.org/wiki/Tutorials/1.9/plugins/overview here] and save the file as <tt>gazebo_tutorials/src/simple_world_plugin.cpp</tt>:

<pre><nowiki>
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
</nowiki></pre>

## Update CMakeLists.txt

Open <tt>gazebo_tutorials/CMakeLists.txt</tt> and replace it with the following:

<pre><nowiki>
cmake_minimum_required(VERSION 2.8.3)
project(gazebo_tutorials)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/simple_world_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)
</nowiki></pre>

## Update package.xml

Update <tt>gazebo_tutorials/package.xml</tt> by adding the following line within the <tt><export></export></tt> tags (or add the <tt><export></export></tt> tags also).

<pre><nowiki>
  <gazebo_ros plugin_path="${prefix}/lib" gazebo_media_path="${prefix}" />
</nowiki></pre>

## Compiling the Plugin

Build the plugin by going to the base of your work space and running catkin:

<pre><nowiki>
cd ~/catkin_ws
catkin_make
</nowiki></pre>

## Creating a World file

Save the following file as <tt>gazebo_tutorials/worlds/hello.world</tt>:

<pre><nowiki>
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
</nowiki></pre>


## Create a Launch File

Create the following launch file gazebo_tutorials/launch/hello.launch:

<pre><nowiki>
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find gazebo_tutorials)/worlds/hello.world"/>
    <!-- more default parameters can be changed here -->
  </include>
</launch>
</nowiki></pre>

## Run the Plugin

<pre><nowiki>
roslaunch gazebo_tutorials hello.launch
</nowiki></pre>

An empty Gazebo should open and in the terminal you should see it print out something like:
<pre>
 INFO ros.gazebo_tutorials: Hello World!
</pre>

## Starting from a Template

A template is available to help you quickly get a Gazebo-ROS plugin working:

[gazebo_ros_template.cpp](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/hydro-devel/gazebo_plugins/src/gazebo_ros_template.cpp)

## Adding Functionality

To make your plugin do something useful with Gazebo and ROS, we suggest you read the ROS-agnostic tutorials on [Plugins](http://gazebosim.org/tutorials/?cat=write_plugin).

## ROS Node Note

All gazebo-ros plugins should check if the ROS node has already been initialized in their <tt>Load()</tt> function, as discussed in this [issue](http://answers.gazebosim.org/question/1493/rosinit-needed-for-ros-gazebo-plugin/). The initialization of the ROS node is performed automatically when you run <pre>rosrun gazebo_ros gazebo</pre> or use the generic empty.world launch file. The <pre>gazebo_ros/src/gazebo_ros_api_plugin.cpp</pre> should be the only place in Gazebo that calls <pre>ros::init()</pre>.

## Next Steps

For miscellaneous Gazebo-ROS tricks see [Advanced ROS integration](http://gazebosim.org/tutorials/?tut=ros_advanced)
