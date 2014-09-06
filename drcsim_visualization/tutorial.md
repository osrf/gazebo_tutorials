#  Overview

Visualizing and logging sensor information is an important part in developing and debugging controllers. This tutorial will show you how to visualize the simulated Atlas robot in rviz, log sensor information and replay that logged sensor information through rviz.

See the documentation pages of [rviz](http://www.ros.org/wiki/rviz) and [rosbag](http://www.ros.org/wiki/rosbag) for more information.

## Setup

We assume that you've already done the [installation step](http://gazebosim.org/tutorials?tut=drcsim_install&cat=drcsim).

If you haven't done so, add the environment setup.sh files to your .bashrc.

~~~
echo 'source /usr/share/drcsim/setup.sh' >> ~/.bashrc
~~~

~~~
source ~/.bashrc
~~~

Install the ROS visualization tools if you do not yet have them. From the command line:

  For ROS Groovy:

~~~
sudo apt-get install ros-groovy-rviz
~~~

  For ROS Hydro:

~~~
sudo apt-get install ros-hydro-rviz
~~~

  For ROS Indigo:

~~~
sudo apt-get install ros-indigo-viz
~~~

Bring up the Atlas robot in Gazebo

        roslaunch drcsim_gazebo atlas.launch

  **For drcsim < 3.1.0**: The package and launch file had a different name:

        roslaunch atlas_utils atlas.launch

When it comes time to bring down this process, press Control + C in the same terminal window. Closing Gazebo from the GUI will not kill the additional processes spawned with roslaunch.

## Visualizing Sensor Data with rviz

[rviz](http://www.ros.org/wiki/rviz) is a powerful robot visualization tool. It provides a convenient GUI to visualize sensor data, robot models, environment maps, which is useful for developing and debugging your robot controllers.

### Running Rviz ###
While your robot is running, start rviz from the command line

~~~
source /usr/share/drcsim/setup.sh
rosrun rviz rviz
~~~

[[file:files/Blank_rviz_2.7.png|640px]]

### Visualizing the Robot model

Now instead of the black screen, we want to actually visualize information. At the bottom of the 'Displays' group, you'll find buttons 'Add', 'Remove', 'Rename' for adding items to visualize, removing items and renaming them. Let's first add the robot model, so click 'Add' and scroll to 'rviz > RobotModel' and click 'OK'. It should now look like the display below.


[[file:files/robot_model_2.7.png|640px]]


We now need to tell rviz which fixed frame we want to use. In the 'Displays' group, under the 'Global Options' item, click the frame label next to 'Fixed Frame'. Type in or select '/pelvis'. You should now see your robot model from a distance.

[[file:files/Rviz_atlas_in_pelvis_frame_new_2.7.png|640px]]


To navigate using a wheeled mouse:

 * Holding left click and dragging rotates the camera around the model

 * Holding scroll wheel and dragging pans the camera

 * The scroll wheel zooms in and out

 * Holding right click and dragging scrolls in and out as well.

### Visualizing Sensor Information

Now we will add sensors to visualize. Click 'Add' to add a new item and add 'rviz > Camera'. Under the Camera item in the displays window, click the empty space to the right of 'Image Topic' and an empty field should appear. Select or type `/multisense_sl/camera/left/image_raw`. You should now see the camera video feed in a small frame. Note that rviz displays an overlay of the robot model on the camera image, which may cause circles to appear in the camera images. The overlays can be turned off by setting the Camera > Overlay Alpha to 1.0. Note: The camera images may appear grey as there is nothing in front of Atlas so try dropping a box in front of the robot to see it in the camera feed.

[[file:files/Rviz_atlas_with_camera_2.7.png|640px]]

Now add a LaserScan using a similar method as the camera, and change the 'Topic' to `/multisense_sl/laser/scan`. Again, to see the laser scan visualization in rviz, drop a box in front of the robot to see red points rendered over the box as it falls.

Poke around in rviz and add different sensors or robot information. TF visualizes the joint transformations, Map visualizes a 2D collision map, PointCloud(2) visualizes depth information from sensors like a Microsoft Kinect.

**Bonus: actuate the laser.**  The laser on the robot's head is mounted on a spindle that rotates.  To start it rotating:

~~~
rostopic pub /multisense_sl/set_spindle_speed std_msgs/Float64 '{ data: 6.0 } '
~~~

The data value is a desired angular velocity, in radians per second.  You should see the laser scan in rviz rotating around.  To build a poor-man's 3-D model of the environment, click in the "Decay Time" field of the "LaserScan" display and increase the time to something non-zero (it's a value in seconds).  You should see scans accumulate in rviz.

[[file:files/Rviz_accumulated_scans_2.7.png|640px]]

### Saving the rviz Configuration

To save the configuration as the default, click "File > Save Config". The next time you run rviz, it will load this configuration.

You can save and load non default configurations by using "File > Save Config as ... " and "File > Open Config" respectively.

You can also run rviz with a different configuration by specifying it at the command line. '''Note:''' Insert the path to the file.

~~~
rosrun rviz rviz -d /your/config/file.vcg
~~~

## Logging Sensor Data with rosbag

[rosbag](http://www.ros.org/wiki/rosbag) provides an easy tool for the efficient storage of topic streams during robot operation.

You can see a list of ROS topics by running

~~~
rostopic list
~~~

We will record the joint states, cameras, laser and [tf](http://www.ros.org/wiki/tf) (transform data) from a robot into a file called ROS.bag

~~~
rosbag record -O /tmp/ROS.bag /tf /atlas/joint_states /multisense_sl/camera/left/image_raw /multisense_sl/camera/left/camera_info /multisense_sl/laser/scan
~~~

After a while, stop the rosbag recording by pressing Control + C in its terminal.

## Playing and Visualizing rosbags

Bring down rviz and your simulation of the Atlas robot. We'll now create a launch file to play the rosbag file.

To play the bag file create a launch file called "playback.launch" with the following content:

~~~
<launch>
  <!-- Creates a command line argument called file -->
  <arg name="file"/>

  <!-- Run the rosbag play as a node with the file argument -->
  <node name="rosbag" pkg="rosbag" type="play" args="--loop $(arg file)" output="screen"/>
</launch>
~~~

You can launch the file with the following command, providing the full absolute path to your `ROS.bag` file.

~~~
roslaunch playback.launch file:=/tmp/ROS.bag
~~~

In a separate terminal run rviz and get the list of topics played by the rosbag

~~~
rosrun rviz rviz &
~~~

~~~
rostopic list
~~~

In rviz, add the robot model, cameras, laser scan and the tf if they weren't saved in your configuration. These streams should now be visualized in rviz as if the robot were running. You should see the robot model itself move to the positions you commanded during the rosbag recording (specifically, the laser should rotate).

Note: The playback will loop, but rviz needs to be told that time was reset at the end of each loop.  If you see the data stop displaying in rviz, click the "Reset" button in the "Time" bar (by default, it's in the lower right corner).  Data display should start up again.

## Wrap up

You should now be familiar with how to visualize ROS topics in rviz as well as how to log them to a rosbag. You should also now be familiar with how to replay rosbag files and visualize them in rviz.
