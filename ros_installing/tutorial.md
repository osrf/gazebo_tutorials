# Introduction

The set of ROS packages for interfacing with Gazebo are contained within a
new meta package (catkin's version of stacks) named `gazebo_ros_pkgs`.
See [Overview of new ROS integration](/tutorials/?tut=ros_overview)
for background information before continuing here.

These instructions are for using the Gazebo versions that are fully integrated
with ROS [Noetic](http://www.ros.org/wiki/noetic),
[Melodic](http://www.ros.org/wiki/melodic) and ROS.  It is recommended to first
read [Which combination of ROS/Gazebo version to use](/tutorials/?tut=ros_wrapper_versions)
before going on with this tutorial. Depending on your needs, you could need an
alternative installation.

## Prerequisites

You should understand the basic concepts of ROS and have gone through the [ROS Tutorials](http://www.ros.org/wiki/ROS/Tutorials).

### Install ROS

We recommend for these ROS integration tutorials you install
(`ros-noetic-desktop-full` or `ros-melodic-desktop-full`) so
that you have all the necessary packages.

See the [ROS installation page](http://www.ros.org/wiki/ROS/Installation) for more details. Be sure to source your ROS setup.bash script by following the instructions on the ROS installation page.

### Install Gazebo

You can install Gazebo either from source or from pre-build Ubuntu debians.

See [Install Gazebo](/tutorials?cat=install). If installing from source, be sure to build the `gazebo_X.Y` (X.Y being your desired version) branch.

#### Test that stand-alone Gazebo works

Before attempting to install the `gazebo_ros_pkgs`, make sure the stand-alone Gazebo works by running in terminal:

~~~
gazebo
~~~

You should see the GUI open with an empty world. Also, test adding a model by clicking on the "Insert" tab on the left and selecting a model to add (then clicking on the simulation to select where to place the model).

#### Test that you have the right version of Gazebo

To check version of Gazebo run:

~~~
gazebo --version
~~~

To see where you installed Gazebo, and if it is in the correct location, run:

~~~
which gzserver
which gzclient
~~~

If you installed from source to the default location it should say:

%%%
/usr/local/bin/gzserver
/usr/local/bin/gzclient
%%%

If you installed from debian/ubuntu binary packages it should say:

%%%
/usr/bin/gzserver
/usr/bin/gzclient
%%%

## Install gazebo\_ros\_pkgs

Choose the method you would prefer. The easier and faster is installing it from
packages but installing from source means you can more easily debug and submit
bug patches ;-)

### A. Install Pre-Built Debian\Ubuntu binary packages

The `gazebo_ros_pkgs` packages are available in:

* [ROS Noetic](http://ros.org/wiki/noetic):

~~~
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
~~~

* [ROS Melodic](http://ros.org/wiki/melodic):

~~~
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
~~~

If this installation method ends successfully for you, jump to the Testing Gazebo with ROS Integration section below.

#### Setup A Catkin Workspace

These instructions require the use of the [catkin](http://www.ros.org/wiki/catkin) build system.

If you do not have a catkin workspace setup, try the following commands:

~~~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
~~~

Then add to your `.bashrc` file a source to the setup scripts:

~~~
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
~~~

For more details see the [Create A Catkin Workspace](http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace) tutorial.

#### Clone the Github Repositories

Make sure `git` is installed on your Ubuntu machine:

~~~
sudo apt-get install git
~~~

##### ROS Noetic

Noetic is using the gazebo 11.x series, start by installing it:

~~~
sudo apt-get install -y libgazebo11-dev
~~~

Download the source code from the [`gazebo_ros_pkgs` github repository](https://github.com/ros-simulation/gazebo_ros_pkgs):

~~~
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel
~~~

Check for any missing dependencies using rosdep:

~~~
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro noetic
~~~

You can automatically install the missing dependencies using rosdep via debian install:

~~~
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
~~~

Now jump to the [build the gazebo\_ros\_pkgs](#Buildthegazebo_ros_pkgs) section.

#### Build the gazebo\_ros\_pkgs

To build the Gazebo ROS integration packages, run the following commands:

~~~
cd ~/catkin_ws/
catkin_make
~~~

See [answers.gazebosim.org](http://answers.gazebosim.org/questions/) for issues or questions with building these packages.

## Testing Gazebo with ROS Integration

Be sure to always source the appropriate ROS setup file, which for Noetic is done like so:

~~~
source /opt/ros/noetic/setup.bash
~~~

You might want to add that line to your `~/.bashrc`.

Assuming your ROS and Gazebo environment have been properly setup and built, you should now be able to run Gazebo through a simple `rosrun` command, after launching `roscore` if needed:

Source the catkin setup.bash if it's not already in your .bashrc

~~~
source ~/catkin_ws/devel/setup.bash
~~~

~~~
roscore &
rosrun gazebo_ros gazebo
~~~

The Gazebo GUI should appear with nothing inside the viewing window.

[[file:figs/800px-EmptyGazebo.png|800px]]

To verify that the proper ROS connections are setup, view the available ROS topics:

~~~
rostopic list
~~~

You should see within the lists topics such as:

%%%
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
%%%

You can also verify the Gazebo services exist:

~~~
rosservice list
~~~

You should see within the list services such as:

%%%
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
%%%

## Other ROS Ways To Start Gazebo

There are several `rosrun` commands for starting Gazebo:

* Launch both the server and client together

    ~~~
    rosrun gazebo_ros gazebo
    ~~~

* Launch the Gazebo server only

    ~~~
    rosrun gazebo_ros gzserver
    ~~~

* Launch the Gazebo client only

    ~~~
    rosrun gazebo_ros gzclient
    ~~~

* Launches the Gazebo server only, in debug mode using GDB

    ~~~
    rosrun gazebo_ros debug
    ~~~

* Additionally, you can start Gazebo using `roslaunch`

    ~~~
    roslaunch gazebo_ros empty_world.launch
    ~~~
