# Overview

This tutorial will explain how to use a music mixer, to teleoperate the Atlas model.  It's a handy way to try things out.

**Note: This is a tutorial, not a polished teleoperation tool. It's meant to be a demo to get you started, nothing more.**

# Hardware requirements

The tools described in this tutorial assume the use of a KORG nanoKONTROL device. We have tested only with version 1 of this device, which looks like this:

[[file:files/Nanokonotrol.jpg|640px|KORG nanoKontrol mixer board]]

This version of the device has been discontinued by the manufacturer, but can still be ordered online, e.g., via [Amazon](http://www.amazon.com/Korg-nanoKONTROL-USB-Controller-White/dp/B001H2P294).  Newer versions of the device can probably be made to work, but some modifications to the driver code (nanokontrol.py) might be required.

## Driver installation

Install the `pygame` library, which we'll use to access the mixer:

~~~
sudo apt-get install python-pygame
~~~

# Simulation setup

Start Gazebo with Atlas in a world where there's something to manipulate (be sure to do the usual `source /usr/share/drcsim/setup.sh` first):

~~~
roslaunch drcsim_gazebo qual_task_2.launch
~~~

You'll see the robot at a table with a drill on it:

[[file:files/Qual_2_start.png|800px|Robot with drill]]

# Teleop package setup

**Note: the ROS package being created here is also available in the drcsim 2.7.x source release, in the directory `tutorials/atlas_teleop`.**

1. Create a ROS package to contain the code for this tutorial. If you haven't already, create a ros directory in your home directory and add it to your `$ROS_PACKAGE_PATH`. From the command line

~~~
mkdir ~/ros
echo "export ROS_PACKAGE_PATH=\$HOME/ros:\$ROS_PACKAGE_PATH" >> ~/.bashrc
source ~/.bashrc
~~~


    ~~~
    cd ~/ros
    roscreate-pkg atlas_teleop osrf_msgs rospy
    roscd atlas_teleop
    mkdir scripts
    cd scripts
    ~~~

1. Download the [nanoKONTROL driver](https://bitbucket.org/osrf/drcsim/raw/default/drcsim_tutorials/atlas_teleop/nanokontrol.py) in your `atlas_teleop` package, and make it executable:

    ~~~
    wget https://bitbucket.org/osrf/drcsim/raw/default/drcsim_tutorials/atlas_teleop/nanokontrol.py
    chmod a+x nanokontrol.py
    ~~~

    This driver receives events from the mixer and publishes ROS [sensor_msgs/Joy](http://ros.org/doc/api/sensor_msgs/html/msg/Joy.html) messages on the `/joy` topic.  I.e., it makes the mixer look like a big joystick, with many axes and many buttons.

1. Download the Atlas teleop controller, [`atlas_teleop.py`](https://bitbucket.org/osrf/drcsim/raw/default/drcsim_tutorials/atlas_teleop/atlas_teleop.py) in your `atlas_teleop` package, and make it executable:

    ~~~
    wget https://bitbucket.org/osrf/drcsim/raw/default/drcsim_tutorials/atlas_teleop/atlas_teleop.py
    chmod a+x atlas_teleop.py
    ~~~

    This controller subscribes to ROS [sensor_msgs/Joy](http://ros.org/doc/api/sensor_msgs/html/msg/Joy.html) messages on the `/joy` topic and commands Atlas and the Sandia hands by publishing [osrf_msgs/JointCommands](https://bitbucket.org/osrf/osrf-common/src/default/osrf_msgs/msg/JointCommands.msg) messages on the `/atlas/joint_commands`, `/sandia_hands/l_hand/joint_commands`, and `/sandia_hands/r_hand/joint_commands` topics.  It requires as a command line argument a YAML configuration file that tells it how to map incoming `/joy` messages into commands for the robot and hands (more on this below).

# Finding your mixer device

1. Plug your KORG nanoKONTROL device into a free USB port.  Depending on the details of your computer, the mixer might show up with any of a number of integer IDs.  You need to find it.  One way to do this is to walk through the possible IDs until it works, starting with:

    ~~~
   # Start a listener on the /joy topic, which the driver will publish to when it's working
    rostopic echo /joy &
    # Try the driver with ID 0
    rosrun atlas_teleop nanokontrol.py 0
    ~~~

If `nanokontrol.py` exits with an error about the wrong ID, start it again (kill `nanokontrol.py` with Ctrl-C first, if needed), with the next ID:

        # It wasn't ID 0.  Try the driver with ID 1
        rosrun atlas_teleop nanokontrol.py 1

 Repeat this procedure until you find the right ID.  Remember that ID.  For the rest of this tutorial, we'll assume that the correct ID is 3 (it seems to often be 3).

1. Move the sliders around; you're looking for a stream of output from `rostopic` similar to:

        ---
        header:
          seq: 7
          stamp:
            secs: 3073
            nsecs: 235000000
          frame_id: ''
        axes: [0.06299212574958801, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ---

1. If you see that output, you've found the right ID.  If you see nothing, continue trying with the next ID.

# Pick up the drill

The `atlas_teleop.py` controller is configured with a YAML file.  Here's an example:

<include src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_mixer/files/drill.yaml' />

Download [`drill.yaml`](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_mixer/files/drill.yaml).

~~~
wget http://bitbucket.org/osrf/gazebo_tutorials/raw/default/drcsim_atlas_mixer/files/drill.yaml
~~~

We'll explain the format below. First, let's try it:

~~~
# The integer argument should be the ID for your device, which you discovered earlier; it might not be 3
rosrun atlas_teleop nanokontrol.py 3 &
rosrun atlas_teleop atlas_teleop.py drill.yaml
~~~

Now start moving the sliders around.  You'll notice that sliders 1-6 make the right arm do various things.  Sliders 7 and 8 make the right hand open and close in different ways.

With a bit of practice, you should be able to pick up the drill and drop it in the bin:

<iframe width="420" height="315" src="//www.youtube.com/embed/ywacltEGnDA" frameborder="0" allowfullscreen></iframe>

# Configuration file format

The configuration file for `atlas_teleop.py` is written in [YAML](http://www.yaml.org/), with the following structure:

* Each line defines a robot pose:
    * The first 28 numbers are joint values for the Atlas robot, given in the [usual order](https://bitbucket.org/osrf/drcsim/raw/default/atlas_msgs/msg/AtlasState.msg).
    * The next two elements specify a grasp posture for the left hand (the following two elements do the same for the right hand), in two parts:
        * The string specifies the grasp type, which should be one of: `cyl` (cylindrical) or `sph` (spherical).
        * The number specifies a grasp position between fully open (0.0) and fully closed (1.0).
* The `0:` line is special, in that it defines the origin pose.
* Each of the other (non-zero) lines defines the target pose for a given slider (1, 2, etc.).
    * When a slider is at 0.0 (all the way down), it commands the origin pose.
    * When a slider is at 1.0 (all the way up), it commands its target pose.
    * When a slider is between 0.0 and 1.0, it commands the pose that results from linearly interpolating between the origin and its target pose.
* Slider commands are *added*  together.  As a result, you can easily blend different postures (blending doesn't always make sense, of course).

# Known issues / caveats

* The mixer produces events only when a slider (or knob, or button) is changed.  So joystick messages and the resulting robot/hand command messages are not published continuously, but rather only when you're moving the sliders.
* After the `nanokontrol.py` driver starts, the initial position of each slider is not known until that slider is moved.  Unexpected things (e.g., sudden jumps) can occur on the first movement of a slider.
