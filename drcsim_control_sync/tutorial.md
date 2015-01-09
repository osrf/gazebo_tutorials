# Overview

This tutorial will explain how to setup synchronized controller updates over ROS topics by using the built-in synchronization mechanism within Atlas simulation interface [AtlasPlugin](https://bitbucket.org/osrf/drcsim/src/default/drcsim_gazebo_ros_plugins/src/AtlasPlugin.cpp?at=default).

## Background

As stated in [DRCSim](https://bitbucket.org/osrf/drcsim/wiki/DRC/UserGuide), joint level control of the Atlas robot can be performed using following ROS topics:

* Receive joint, imu and force torque sensor states from the robot.
 * ROS topic: `/atlas/atlas_state`
 * message type: [atlas_msgs/AtlasState](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/msg/AtlasState.msg?at=default)
* Send setpoints and gains to the PID controllers that are running on each joint.
 * ROS topic: `/atlas/atlas_command`
 * message type: [atlas_msgs/AtlasCommand](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/msg/AtlasCommand.msg?at=default)

In this tutorial, we'll construct a basic ROS node that listens to robot state (`AtlasState`) and publishes robot commands (`AtlasCommand`), while enforcing basic synchronization.

ROS topics are inherently asynchronous; however, with the addition of a built-in simulation delay mechanism, controller synchronization simulating real-time control can be achieved.  Synchronization is achieved with potential cost in reduced overall simulation performance, but the trade off can be tuned via exposed ROS parameters.  The synchronization mechanism has the following ROS parameters for controlling amount of delay to inject into simulation when expected controller command does not arrive on time:

* [`AtlasCommand/desired_controller_period_ms`](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/msg/AtlasCommand.msg?at=default#cl-36): This `uint8` parameter tells the simulated Atlas driver interface to expect a command at least once every N-milliseconds of simulation time.  If it's set to 0, no delay is enforced.  For example, if the controller update is expected to run at 200Hz, `desired_controller_period_ms` should be set to 5.
* ROS Parameter `/atlas/delay_window_size`: paging window size defined in real-time seconds. This parameter will allow `delay_max_per_window` seconds of delay with a single window.  Upon window paging, internally accumulated total delay per window period is reset to 0.  The default value for this parameter is 5 seconds real-time.
* ROS Parameter `/atlas/delay_max_per_window`: total cumulative delay in seconds allotted per `delay_window_size`.  The default value for this parameter is 250ms real-time.
* ROS Parameter `/atlas/delay_max_per_step`: maximum delay per simulation time step.   The default value for this parameter is 25ms real-time.

The default values for above parameters are chosen such that for near real-time simulation with `desired_controller_period_ms` of 5 simulation milliseconds, and a controller that takes ~2ms real time to compute a new command in response to a received state, can comfortably run synchronized at 200Hz simulation time (command age <= 5ms simulation time) without exhausting the delay budgets.

## Simulation Setup

### Creating ROS environment to launch demo
For running the demo script a minimal ROS setup is needed. Use a directory under `ROS_PACKAGE_PATH` and follow the instructions to create a package there:

~~~
. /usr/share/drcsim/setup.sh
cd ~/ros
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
roscreate-pkg control_synchronization_tutorial drcsim_gazebo
~~~

Copy the [launch file with control synchronization parameters](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_control_sync/files/atlas_sync.launch) into a file named `~/ros/control_synchronization_tutorial/atlas_sync.launch`:

<include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_control_sync/files/atlas_sync.launch' />

## Mock-Controller Node Setup


Download [`my_atlas_controller.cpp`](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_control_sync/files/my_atlas_controller.cpp) into  `~/ros/control_synchronization_tutorial/my_atlas_controller.cpp`. This file contains the following code:

<include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_control_sync/files/my_atlas_controller.cpp' />

In this example, robot states are received by `SetAtlasState()` callback when new messages arrive over the wire (ROS topic `/atlas/atlas_state`).  A separate worker thread `Work()` is expected to independently update and publish [`AtlasCommand`](https://bitbucket.org/osrf/drcsim/src/default/atlas_msgs/msg/AtlasCommand.msg?at=default). The published command contains an arbitrarily defined joint trajectory with `desired_controller_period_ms` set to 5ms (to be enforced in simulation-time).  If executed successfully, the robot will thrash around on the ground, and the age of received joint command should never exceed 5ms (simulation-time) old.

### Compiling the Tutorial
Edit `CMakeLists.txt` by typing:

~~~
gedit ~/ros/control_synchronization_tutorial/CMakeLists.txt
~~~

Append the following line to the end of `CMakeLists.txt`:

~~~
rosbuild_add_executable(my_atlas_controller my_atlas_controller.cpp)
~~~

To compile, type the following commands:

~~~
roscd control_synchronization_tutorial
make
~~~

## Running the Tutorial
To run this tutorial, you'll need at least 2 separate terminals.  Don't forget to execute following setup commands in each new terminal you open.

~~~
. /usr/share/drcsim/setup.sh
cd ~/ros
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
~~~

Start simulation by roslaunching the newly created launch file:

~~~
roslaunch control_synchronization_tutorial atlas_sync.launch
~~~

You should see an Atlas robot standing in an empty world:

[[file:files/Atlas_sync.png|640px]]

If running this in the cloud, make sure to set your `GAZEBO_IP` and `ROS_IP` in every open terminal, and roslaunch instead:

~~~
roslaunch control_synchronization_tutorial atlas_sync.launch gzname:=gzserver
~~~

In a separate terminal, start the mock-controller:

~~~
rosrun control_synchronization_tutorial my_atlas_controller
~~~

To view synchronization status, `rxplot`  or `rqt_plot` can be used.

To use `rxplot` (deprecated):

~~~
# depending on your ROS version
sudo apt-get install ros-hydro-rx
sudo apt-get install ros-indigo-rx
~~~

Then open a third terminal (don't forget to source `setup.sh` and export `ROS_PACKAGE_PATH`), and type the following:

~~~
rxplot -p 5 -b 10 /atlas/controller_statistics/command_age /atlas/controller_statistics/command_age_mean /atlas/synchronization_statistics/delay_in_step /atlas/synchronization_statistics/delay_in_window /atlas/synchronization_statistics/delay_window_remain
~~~

Or alternatively with `rqt_plot`, install by:

~~~
# depending on your ROS version
sudo apt-get install ros-hydro-rqt-plot
sudo apt-get install ros-indigo-rqt-plot
~~~

and plot by running `rqt` and setup custom perspectives; or by executing command below to see all the plots in one window:

~~~
rqt_plot /atlas/controller_statistics/command_age /atlas/controller_statistics/command_age_mean /atlas/synchronization_statistics/delay_in_step /atlas/synchronization_statistics/delay_in_window /atlas/synchronization_statistics/delay_window_remain
~~~

[[file:files/Sync_stats.png|640px]]

The sub-plots (from top down) are:

  * Instantaneous age of AtlasCommand.
  * Average of AtlasCommand age taken over a moving window of 1 second simulation time.
  * Instantaneous delay sleep needed to receive a new AtlasCommand within prescribed time period of 5 ms. simulation time.
  * Accumulated sleep used in every delay window.
  * Time until next delay window and delay budget reset.

### Important Note on Delay Budget
As demonstrated in this tutorial, synchronization at 200Hz(sim time rate) for a controller that takes roughly 2 ms. real-time to do its calculations will fit comfortably inside of the [existing synchronization delay budget](https://bitbucket.org/osrf/drcsim/src/default/drcsim_gazebo/launch/atlas_sandia_hands.launch?at=default#cl-20). However, at 200 Hz. (sim time synchronization rate), if your controller takes more than 2ms(real-time) to update, due to synchronization overheads, you will exhaust the [existing delay budget quite quickly](https://bitbucket.org/osrf/drcsim/src/default/drcsim_gazebo/launch/atlas_sandia_hands.launch?at=default#cl-20). Similar to the previous timing plot, below is what happens when a controller takes 3 ms. (real-time) to do its calculations:

[[file:files/Sync_exhaust_stats.png|640px]]

As shown in above figure, within each 5 second delay window, the delay budget is exhausted in about 1 second, and controller becomes unsynchronized.

Along the same logic, attempt to synchronize at 500 Hz. (sim time rate) and stay within the existing delay budget, the controller needs to be able to complete its update within 0.5 ms. (real-time) (For a sample implementation, see [pub\_atlas\_command_fast.cpp](https://bitbucket.org/osrf/drcsim/src/default/drcsim_gazebo_ros_plugins/src/pub_atlas_command.cpp?at=default).

## Known Issues
  * Pausing simulation has unexpected behaviors, given the delay budget control is done in real-time.
  * Large `delay_max_per_step` values can cause simulation to "freeze" on controller startup.  Keep these numbers small to avoid simulation stuttering.
  * The mock-controller in this example may not depict your controller setup.
