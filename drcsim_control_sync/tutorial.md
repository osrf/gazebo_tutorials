# Overview

This tutorial will explain how to setup synchronized controller updates over ROS topics by using the built-in synchronization mechanism within Atlas simulation interface [AtlasPlugin](https://bitbucket.org/osrf/drcsim/src/f31ae4bfec80e40deb8936a0c335e8e62edc3fb3/drcsim_gazebo_ros_plugins/src/AtlasPlugin.cpp?at=default).

## Background

As stated in [DRCSim](http://gazebosim.org/wiki/DRC/UserGuide), joint level control of the Atlas robot can be performed using following ROS topics:

* Receive joint, imu and force torque sensor states from the robot.
 * ROS topic: `/atlas/atlas_state`
 * message type: [atlas_msgs/AtlasState](https://bitbucket.org/osrf/drcsim/src/f31ae4bfec80e40deb8936a0c335e8e62edc3fb3/atlas_msgs/msg/AtlasState.msg?at=default) 
* Send setpoints and gains to the PID controllers that are running on each joint.
 * ROS topic: `/atlas/atlas_command`
 * message type: [atlas_msgs/AtlasCommand](https://bitbucket.org/osrf/drcsim/src/f31ae4bfec80e40deb8936a0c335e8e62edc3fb3/atlas_msgs/msg/AtlasCommand.msg?at=default)

In this tutorial, we'll construct a basic ROS node that listens to robot state (`AtlasState`) and publishes robot commands (`AtlasCommand`), while enforcing basic synchronization.

ROS topics are inherently asynchronous; however, with the addition of a built-in simulation delay mechanism, controller synchronization simulating real-time control can be achieved.  Synchronization is achieved with potential cost in reduced overall simulation performance, but the trade off can be tuned via exposed ROS parameters.  The synchronization mechanism has the following ROS parameters for controlling amount of delay to inject into simulation when expected controller command does not arrive on time:

* [`AtlasCommand/desired_controller_period_ms`](https://bitbucket.org/osrf/drcsim/src/f31ae4bfec80e40deb8936a0c335e8e62edc3fb3/atlas_msgs/msg/AtlasCommand.msg?at=default#cl-36): This `uint8` parameter tells the simulated Atlas driver interface to expect a command at least once every N-milliseconds of simulation time.  If it's set to 0, no delay is enforced.  For example, if the controller update is expected to run at 200Hz, `desired_controller_period_ms` should be set to 5.
* ROS Parameter `/atlas/delay_window_size`: paging window size defined in real-time seconds. This parameter will allow `delay_max_per_window`-seconds of delay with a single window.  Upon window paging, internally accumulated total delay per window period is reset to 0.  The default value for this parameter is 5 seconds real-time.
* ROS Parameter `/atlas/delay_max_per_window`: total cumulative delay in seconds allotted per `delay_window_size`.  The default value for this parameter is 250ms real-time.
* ROS Parameter `/atlas/delay_max_per_step`: maximum delay per simulation time step.   The default value for this parameter is 25ms real-time.

The default values for above parameters are chosen such that for near real-time simulation with `desired_controller_period_ms` of 5 simulation milliseconds, and a controller that takes ~2ms real time to compute a new command in response to a received state, can comfortably run synchronized at 200Hz simulation time (command age <= 5ms simulation time) without exhausting the delay budgets.

## Simulation Setup

### Creating ROS environment to launch demo
For running the demo script a minimal ROS setup is needed. Use a directory under ROS_PACKAGE_PATH and follow the instructions to create a package there:

~~~
. /usr/share/drcsim/setup.sh
cd ~/ros
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
roscreate-pkg control_synchronization_tutorial drcsim_gazebo
~~~

Create a launch file with control synchronization parameters by saving the following content into a file named `~/ros/control_synchronization_tutorial/atlas_sync.launch`:

~~~
<launch>
  <!-- Control Synchronization Parameters -->
  <!-- delay_window_size: paging window that will allow delay_max_per_window-seconds of delay. -->
  <!-- delay_max_per_window: total cumulative delay in seconds allotted per delay_window_size. -->
  <!-- delay_max_per_step: maximum delay per simulation time step. -->
  <param name="/atlas/delay_window_size" type="double" value="25.0"/>
  <param name="/atlas/delay_max_per_window" type="double" value="1.0"/>
  <param name="/atlas/delay_max_per_step" type="double" value="0.1"/>

  <arg name="gzname" default="gazebo"/>
  <!-- default launch file for starting an Atlas robot -->
  <include file="$(find drcsim_gazebo)/launch/atlas.launch">
    <arg name="gzname" value="$(arg gzname)"/>
    <arg name="gzworld" value="atlas.world"/>
  </include>
</launch>
~~~

## Mock-Controller Node Setup

Create a file named `my_atlas_controller.cpp`:

~~~
gedit ~/ros/control_synchronization_tutorial/my_atlas_controller.cpp
~~~

and populate it with following content:

~~~
#include <string>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>

ros::Publisher pubAtlasCommand;
atlas_msgs::AtlasCommand ac;
atlas_msgs::AtlasState as;
boost::mutex mutex;
ros::Time t0;
unsigned int numJoints = 28;

void SetAtlasState(const atlas_msgs::AtlasState::ConstPtr &_as)
{
  static ros::Time startTime = ros::Time::now();
  t0 = startTime;

  // lock to copy incoming AtlasState
  {
    boost::mutex::scoped_lock lock(mutex);
    as = *_as;
  }

  // uncomment to simulate state filtering
  // usleep(1000);
}

void Work()
{
  // simulated worker thread
  while(true)
  {
    // lock to get data from AtlasState
    {
      boost::mutex::scoped_lock lock(mutex);
      // for testing round trip time
      ac.header.stamp = as.header.stamp;
    }

    // simulate working
    usleep(2000);

    // assign arbitrary joint angle targets
    for (unsigned int i = 0; i < numJoints; i++)
    {
      ac.position[i] = 3.2* sin((ros::Time::now() - t0).toSec());
      ac.k_effort[i] = 255;
    }

    // Let AtlasPlugin driver know that a response over /atlas/atlas_command
    // is expected every 5ms; and to wait for AtlasCommand if none has been
    // received yet. Use up the delay budget if wait is needed.
    ac.desired_controller_period_ms = 5;

    pubAtlasCommand.publish(ac);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_atlas_commandt");

  ros::NodeHandle* rosnode = new ros::NodeHandle();

  // this wait is needed to ensure this ros node has gotten
  // simulation published /clock message, containing
  // simulation time.
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }

  ac.position.resize(numJoints);
  ac.k_effort.resize(numJoints);

  // default values for AtlasCommand
  for (unsigned int i = 0; i < numJoints; i++)
    ac.k_effort[i]     = 255;

  // ros topic subscribtions
  ros::SubscribeOptions atlasStateSo =
    ros::SubscribeOptions::create<atlas_msgs::AtlasState>(
    "/atlas/atlas_state", 100, SetAtlasState,
    ros::VoidPtr(), rosnode->getCallbackQueue());
  atlasStateSo.transport_hints =
    ros::TransportHints().reliable().tcpNoDelay(true);
  ros::Subscriber subAtlasState = rosnode->subscribe(atlasStateSo);

  // ros topic publisher
  pubAtlasCommand = rosnode->advertise<atlas_msgs::AtlasCommand>(
    "/atlas/atlas_command", 100, true);

  // simulated worker thread
  boost::thread work = boost::thread(&Work);

  ros::spin();

  return 0;
}
~~~

In this example, robot states are received by `SetAtlasState()`-callback when new messages arrive over the wire (ROS topic `/atlas/atlas_state`).  A separate worker thread `Work()` is expected to independently update and publish [`AtlasCommand`](https://bitbucket.org/osrf/drcsim/src/f31ae4bfec80e40deb8936a0c335e8e62edc3fb3/atlas_msgs/msg/AtlasCommand.msg?at=default). The published command contains an arbitrarily defined joint trajectory with `desired_controller_period_ms` set to 5ms (to be enforced in simulation-time).  If executed successfully, the robot will thrash around on the ground, and the age of received joint command should never exceed 5ms (simulation-time) old.

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

[[file:fles/atlas_sync.png|640px]]

If running this in the cloud, make sure to set your `GAZEBO_IP` and `ROS_IP` in every open terminal, and roslaunch instead:

~~~
roslaunch control_synchronization_tutorial atlas_sync.launch gzname:=gzserver
~~~

In a separate terminal, start the mock-controller:

~~~
rosrun control_synchronization_tutorial my_atlas_controller
~~~

To view synchronization status, rxplot will be used. To install rxplot:

~~~
# depending on your ROS version
sudo apt-get install ros-fuerte-rx
sudo apt-get install ros-groovy-rx
~~~

Then open a third terminal (don't forget to source `setup.sh` and export `ROS\_PACKAGE\_PATH`), and type the following:

~~~
rxplot -p 5 -b 10 /atlas/controller_statistics/command_age /atlas/controller_statistics/command_age_mean /atlas/synchronization_statistics/delay_in_step /atlas/synchronization_statistics/delay_in_window /atlas/synchronization_statistics/delay_window_remain
~~~

[[file:files/sync_stats.png|640px]]

The sub-plots (from top down) are:

  * Instantaneous age of AtlasCommand.
  * Average of AtlasCommand age taken over a moving window of 1 second simulation time.
  * Instantaneous delay sleep needed to receive a new AtlasCommand within prescribed time period of 5ms simulation time.
  * Accumulated sleep used in every delay window.
  * Time until next delay window and delay budget reset.

### Important Note on Delay Budget
As demonstrated in this tutorial, synchronization at 200Hz(sim time rate) for a controller that takes roughly 2ms real-time to do its calculations will fit comfortably inside of the [existing synchronization delay budget](https://bitbucket.org/osrf/drcsim/src/f31ae4bfec80e40deb8936a0c335e8e62edc3fb3/drcsim_gazebo/launch/atlas_sandia_hands.launch?at=default#cl-20). However, at 200Hz(sim time synchronization rate), if your controller takes more than 2ms(real-time) to update, due to synchronization overheads, you will exhaust the [existing delay budget quite quickly](https://bitbucket.org/osrf/drcsim/src/826552d800ba1fd84554b7f46f9757249f663565/ros/atlas_utils/launch/atlas_sandia_hands.launch?at=default#cl-16).  Similar to the previous timing plot, below is what happens when a controller takes 3ms(real-time) to do its calculations:

[[file:files/sync_exhaust_stats.png|640px]]

As shown in above figure, within each 5 second delay window, the delay budget is exhausted in about 1 second, and controller becomes unsynchronized.

Along the same logic, attempt to synchronize at 500Hz(sim time rate) and stay within the existing delay budget, the controller needs to be able to complete its update within 0.5ms(real-time) (For a sample implementation, see [pub\_atlas\_command_fast.cpp](https://bitbucket.org/osrf/drcsim/src/f31ae4bfec80e40deb8936a0c335e8e62edc3fb3/drcsim_gazebo/test/pub_atlas_command.launch?at=default).

## Known Issues
  * Pausing simulation has unexpected behaviors, given the delay budget control is done in real-time.
  * Large `delay_max_per_step` values can cause simulation to "freeze" on controller startup.  Keep these numbers small to avoid simulation stuttering.
  * The mock-controller in this example may not depict your controller setup.
