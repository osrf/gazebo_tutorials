# Tutorial: Installing gazebo\_ros\_pkgs

The set of ROS packages for interfacing with Gazebo are contained within a new meta package (catkin's version of stacks) named <tt>gazebo\_ros\_pkgs</tt>. See [Overview of new ROS integration](http://gazebosim.org/tutorials/?tut=ros_overview) for background information before continuing here.

These instructions are for using the latest stand-alone version of Gazebo (version 1.9) with ROS [Hydro](http://www.ros.org/wiki/hydro) using [catkin](http://www.ros.org/wiki/catkin). ROS [Groovy](http://www.ros.org/wiki/groovy) is somewhat compatible with gazebo\_ros\_pkgs but requires more of the components be installed from source and is not an official target of gazebo\_ros\_pkgs.

## Prerequisites

You should understand the basic concepts of ROS and have gone through the [ROS Tutorials](http://www.ros.org/wiki/ROS/Tutorials).

## Install ROS

We recommend for these ROS integration tutorials you install '''ros-hydro-desktop-full''' so that you have all the necessary packages.

See the [ROS installation page](http://www.ros.org/wiki/ROS/Installation) for more details. Be sure to source your ROS setup.bash script by following the instructions on the ROS installation page.

## Remove ROS's Old Version of Gazebo ROS Integration

If you have previously installed ROS's version of Gazebo through the ROS debians, remove them now by running either of these:

<pre>
sudo apt-get remove ros-fuerte-simulator-gazebo ros-groovy-simulator-gazebo
</pre>

## Install Gazebo

You can install Gazebo either from source or from pre-build Ubuntu debians.

See [Install Gazebo 1.9](http://gazebosim.org/tutorials?tut=install&ver=1.9&cat=get_started). If installing from source, be sure to build the '''gazebo_1.9''' branch.

## Test that stand-alone Gazebo works

Before attempting to install the gazebo\_ros\_pkgs, make sure the stand-alone Gazebo works by running in terminal:

<pre>
gazebo
</pre>

You should see the GUI open with an empty world. Also, test adding a model by clicking on the "Insert" tab on the left and selecting a model to add (then clicking on the simulation to select where to place the model).

## Test that you have the right version of Gazebo

To see where you install Gazebo, and if it is in the correct location, run:

<pre>
which gzserver
which gzclient
</pre>

If you installed from source to the default location it should say:
<pre>
/usr/local/bin/gzserver
/usr/local/bin/gzclient
</pre>

If you installed from debian it should say:
<pre>
/usr/bin/gzserver
/usr/bin/gzclient
</pre>

## Install gazebo\_ros\_pkgs

Choose the method you would prefer. Installing from source means you can more easily debug and submit bug patches ;-)

### Install Pre-Built Debians

The gazebo\_ros\_pkgs are available in [ROS Hydro](http://ros.org/wiki/hydro) as debians for Ubuntu. To install, first ensure you have ROS Hydro properly [setup](http://ros.org/wiki/hydro/Installation) with ROS's package repository and key. Then:

<pre>
sudo apt-get install ros-hydro-gazebo-ros-pkgs ros-hydro-gazebo-ros-control
</pre>

If this installation method worked for you, jump to [[#Testing_Gazebo_with_ROS_Integration|the next step]].

### Install from Source (on Ubuntu)

If you are running an earlier version of ROS (Groovy, Fuerte, Electric) you will need to install gazebo\_ros\_pkgs from source. Installing from source is also useful if you want to develop new plugins or submit patches.

#### Setup A Catkin Workspace

These instructions require the use of the [catkin](http://www.ros.org/wiki/catkin) build system.

If you do not have a catkin workspace setup, try the following commands:

<pre>
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
</pre>

Then add to your <tt>.bashrc</tt> file a source to the setup scripts:

<pre>
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
</pre>

For more details see the [Create A Catkin Workspace](http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace) tutorial.

#### Clone the Github Repositories

Make sure <tt>git</tt> is installed on your Ubuntu machine

<pre>
sudo apt-get install git
</pre>

Choose the ROS distro you are targeting:

##### ROS Hydro

<pre>
sudo apt-get install -y gazebo
</pre>

Download the source code from the [gazebo_ros_pkgs Github repo](https://github.com/ros-simulation/gazebo_ros_pkgs):
<pre>
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
</pre>

You are highly encouraged to fork our code and submit a pull request if you find fixes or features that you'd like to add.

Check for any missing dependencies using rosdep:
<pre>
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro hydro
</pre>

You can automatically install the missing dependencies using rosdep via debian install:
<pre>
rosdep install --from-paths . --ignore-src --rosdistro hydro -y
</pre>

Now jump to the **Build_the_gazebo\_ros\_pkgs** section.

##### ROS Groovy

ROS Groovy and earlier distros are not officially supported but maintenance support is available for Groovy.

Download the source code from the [gazebo_ros_pkgs Github repo](https://github.com/ros-simulation/gazebo_ros_pkgs):
<pre>
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
</pre>

''Note: the groovy-devel branch has been deleted because the breaking API changes in PCL have been resolved. The hydro-devel branch now works with Groovy.''

You are highly encouraged to fork our code and submit a pull request if you find fixes or features that you'd like to add.

You will also need the [ros_control](http://ros.org/wiki/ros_control) and [ros_controllers](http://ros.org/wiki/ros_controllers) packages installed on your system from source - they are new in ROS Hydro. If you do not already have these, install them from source:

<pre>
git clone https://github.com/ros-controls/ros_control.git
git clone https://github.com/ros-controls/ros_controllers.git -b groovy-backported-hydro
git clone https://github.com/ros-controls/control_toolbox.git
git clone https://github.com/ros-controls/realtime_tools.git
</pre>

Check for any missing dependencies using rosdep:
<pre>
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro groovy
</pre>

You can automatically install the missing dependencies using rosdep via debian install:
<pre>
rosdep install --from-paths . --ignore-src --rosdistro groovy -y
</pre>

#### Build the gazebo\_ros\_pkgs

To build the Gazebo ROS integration packages, run the following commands:

<pre>
cd ~/catkin_ws/
catkin_make
</pre>

See [answers.gazebosim.org](http://answers.gazebosim.org/questions/) for issues or questions with building these packages.

## Testing Gazebo with ROS Integration
Be sure to always source the appropriate ROS setup file, which for Hydro is done like so:

<pre>
source /opt/ros/hydro/setup.bash
</pre>

You might want to add that line to your `~/.bashrc`.

Assuming your ROS and Gazebo environment have been properly setup and built, you should now be able to run Gazebo through a simple <tt>rosrun</tt> command, after launching <tt>roscore</tt> if needed:

Source the catkin setup.bash if it's not already in your .bashrc

<pre>
source ~/catkin_ws/devel/setup.bash
</pre>

<pre>
roscore &
rosrun gazebo_ros gazebo
</pre>

The Gazebo GUI should appear with nothing inside the viewing window.

[[file:figs/800px-EmptyGazebo.png|800px]]

To verify that the proper ROS connections are setup, view the available ROS topics:

<pre>
rostopic list
</pre>

You should see within the lists topics such as:

<pre>
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
</pre>

You can also verify the Gazebo services exist:

<pre>
rosservice list
</pre>

You should see within the list services such as:

<pre>
/gazebo/apply_body_wrench
/gazebo/apply_joint_effort
/gazebo/clear_body_wrenches
/gazebo/clear_joint_forces
/gazebo/delete_model
/gazebo/get_joint_properties
/gazebo/get_link_properties
/gazebo/get_link_state
/gazebo/get_loggers
/gazebo/get_model_properties
/gazebo/get_model_state
/gazebo/get_physics_properties
/gazebo/get_world_properties
/gazebo/pause_physics
/gazebo/reset_simulation
/gazebo/reset_world
/gazebo/set_joint_properties
/gazebo/set_link_properties
/gazebo/set_link_state
/gazebo/set_logger_level
/gazebo/set_model_configuration
/gazebo/set_model_state
/gazebo/set_parameters
/gazebo/set_physics_properties
/gazebo/spawn_gazebo_model
/gazebo/spawn_sdf_model
/gazebo/spawn_urdf_model
/gazebo/unpause_physics
/rosout/get_loggers
/rosout/set_logger_level
</pre>

## Other ROS Ways To Start Gazebo

There are several <tt>rosrun</tt> commands for starting Gazebo:

**rosrun gazebo_ros gazebo**

> Launched both the server and client together

**rosrun gazebo_ros gzserver**

> Launches the Gazebo server only

**rosrun gazebo_ros gzclient**

> Launches the Gazebo client only

**rosrun gazebo_ros debug**

> Launches the Gazebo server only, in debug mode using GDB

Additionally, you can start Gazebo using <tt>roslaunch</tt>

## Next Steps

You are now ready to launch robot models (URDFs) into Gazebo demo worlds in the tutorial [Using <tt>roslaunch</tt> Files to Spawn Models](http://gazebosim.org/tutorials/?tut=ros_roslaunch).
