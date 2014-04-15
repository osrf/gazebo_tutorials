# Tutorial: ROS integration overview

As of Gazebo 1.9 and [ROS Hydro](http://www.ros.org/wiki/hydro/), Gazebo no longer has any direct ROS dependencies and is now installed as an Ubuntu stand-alone package. Historically using Gazebo with ROS required a specific version of Gazebo be built with the legacy 'simulator_gazebo' stack.

To achieve ROS integration with stand-alone Gazebo, a new set of ROS packages named [`gazebo_ros_pkgs`](http://ros.org/wiki/gazebo_ros_pkgs) has been created to provide wrappers around the stand-alone Gazebo. They provide the necessary interfaces to simulate a robot in Gazebo using ROS messages, services and dynamic reconfigure. Among the primary differences from 'simulator_gazebo', 'gazebo_ros_pkgs' now:

- Supports the latest stand alone system dependency of Gazebo, that has no ROS bindings on its own
- Builds with [catkin](http://www.ros.org/wiki/catkin)
- Treats URDF and [SDF](http://gazebosim.org/sdf.html) as equally as possible
- Reduces code duplication with Gazebo
- Improves out of the box support for controllers using <tt>ros_control</tt>
- Integrates real time controller efficiency improvements from the DARPA Robotics Challenge
- Cleans up old code from previous versions of ROS and Gazebo

An overview of the new interface is in the following diagram:

[[file:figs/775px-Gazebo_ros_api.png|775px]]

## Upgrading from simulator_gazebo

The following guidelines will help you upgrade your Gazebo-dependent packages from <tt>simulator_gazebo</tt> for use in your ROS packages:

### Catkin

Your previous packages for interfacing with Gazebo with the old <tt>simulator_gazebo</tt> stack are likely still using the [rosbuild](http://www.ros.org/wiki/rosbuild) build system. With gazebo_ros_pkgs in ROS Hydro, you will first need to "catkinize" your packages to begin migration. See the [Catkin Tutorials](http://www.ros.org/wiki/catkin/Tutorials).

### Launch Files

Some changes are required in previously created roslaunch files for starting Gazebo. The best way to update these packages is to review the [Using roslaunch files to spawn models in Gazebo](http://gazebosim.org/wiki/Tutorials/1.9/Using_roslaunch_Files_to_Spawn_Models) tutorial. In a nutshell:

- Within roslaunch files, <tt>pkg="gazebo"</tt> needs to be now renamed to <tt>pkg="gazebo'''_ros'''"</tt>
- <tt>gazebo_worlds</tt> package has been removed. Most of the world files were rarely used and were not maintained with changes in SDF XML formats. Thus, all worlds have been centralized within the Gazebo project itself, including <tt>empty.world</tt>.
- The best way to use Gazebo launch files is to simply inherent/include the master "empty world" launch file located in the <tt>gazebo_ros</tt> package.

### CMakeLists.txt

- Because Gazebo is no longer a ROS package but instead a system dependency, your CMake file might need to be reconfigured. The following is an example CMakeLists.txt:

<pre><nowiki>
cmake_minimum_required(VERSION 2.8.3)
project(YOURROBOT_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

# Build whatever you need here
add_library(...) # TODO

catkin_package(
    DEPENDS
      gazebo_ros
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)
</nowiki></pre>

### package.xml

This is the replacement for the rosbuild "manifest.xml":

- Add dependency on the new <tt>gazebo_ros</tt> package:
<pre>
<build_depend>gazebo_ros</build_depend>
<run_depend>gazebo_ros</run_depend>
</pre>

### Running Gazebo

The names of the ROS nodes to launch Gazebo have changes slightly to coincide with the Gazebo executable names:

 - <tt>rosrun gazebo_ros gazebo</tt> now launch both the Gazebo server and GUI.
 - <tt>rosrun gazebo_ros gui</tt> has been renamed to <tt>rosrun gazebo_ros gzclient</tt>
 - <tt>rosrun gazebo_ros gzserver</tt> has been added

Available nodes to run:

<pre>
rosrun gazebo_ros gazebo
rosrun gazebo_ros gzserver
rosrun gazebo_ros gzclient
rosrun gazebo_ros spawn_model
rosrun gazebo_ros perf
rosrun gazebo_ros debug
</pre>

These nodes are better documented in the tutorial [ Using roslaunch files to spawn models in Gazebo](http://gazebosim.org/wiki/Tutorials/1.9/Using_roslaunch_Files_to_Spawn_Models).

### More

''Add your upgrade issues here, please''

## Tutorials

Tutorials from ros.org have been almost entirely removed and re-written from scratch on this website to reflect the many changes that have occured over the course of Gazebo's history. We've done our best to thoroughly document how to get your URDF-based robot running smoothly in Gazebo. If you have any question please see [answers.ros.org](http://answers.ros.org/).

Continue to [Installing gazebo_ros Packages](http://gazebosim.org/wiki/Tutorials/1.9/Installing_gazebo_ros_Packages).
