# Overview

This tutorial introduces basic sensor stream visualization from simulated
[MultiSense SL](http://files.carnegierobotics.com/products/MultiSense_SL/MultiSense_SL_brochure.pdf) sensor head using [rviz](http://ros.org/wiki/rviz).

## Setup
If not done previously, [install DRCSim first](http://gazebosim.org/tutorials/?tut=drcsim_install).

## Starting Atlas Robot Simulation
Source the environment setup script:

    source /opt/ros/[Your ROS Distro Name]/setup.bash   # this is necessary for drcsim 3.1.0 and later
    source /usr/share/drcsim/setup.sh

Start the Atlas robot simulation:

    roslaunch drcsim_gazebo atlas.launch

**For drcsim < 3.1.0**: The package and launch file had a different name:

    roslaunch atlas_utils atlas.launch

the Gazebo gui should appear:

[[file:files/Atlas_launch.png|640px]]

## Using RVIZ for Visualization of Sensor Data

Next, start [rviz](http://ros.org/wiki/rviz)

    source /usr/share/drcsim/setup.sh
    rosrun rviz rviz

and follow steps in [this tutorial](http://gazebosim.org/tutorials?tut=drcsim_visualization&cat=drcsim) to configure rviz properly.

More importantly, repeat the steps to setup camera and laser visualization but substitute ROS topics in the original instructions with following ROS topics:

  - camera image streams: `/multisense_sl/camera/left/image_raw` and `/multisense_sl/camera/right/image_raw` for left and right eye.
  - LIDAR data stream: `/multisense_sl/laser/scan`

**For drcsim >= 4.1.0**: If you are using atlas versions >= v3 (e.g. atlas\_v3.launch), the camera and laser topics have changed:

  - camera image streams: `/multisense/camera/left/image_raw` and `/multisense/camera/right/image_raw` for left and right eye.
  - LIDAR data stream: `/multisense/lidar_scan`

In addition to visualizing camera images and laser data, to visualize stereo point cloud, click `Add` button under the rviz *Displays* panel, select `PointCloud2` then click `OK`.  Set the Topic for the newly added PointCloud2 visual as `/multisense_sl/camera/points2`.  Now drop a box in front of the robot to see the point cloud visualization in rviz.

[[file:files/Atlas_rviz.png|640px]]

## Controlling the Head LIDAR spindle ##

To set rotation speed commands to the LIDAR spindle, open a new terminal and type

    rostopic pub --once /multisense_sl/set_spindle_speed std_msgs/Float64 '{data: 3.0}'

**For drcsim >= 4.1.0**: If you are using atlas versions >= v3 (e.g. atlas\_v3.launch), use the following command instead:

    rostopic pub --once /multisense/set_spindle_speed std_msgs/Float64 '{data: 3.0}'

Setting `Decay Time` under `LaserScan` in the rviz display panel to 5 (seconds), and change the laser scan data to green in rviz yields:

[[file:files/Atlas_rviz_spindle.png|640px]]
