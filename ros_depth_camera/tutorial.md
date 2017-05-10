# Use a Gazebo Depth Camera with ROS

## Introduction

In this tutorial, you'll learn how to connect a Gazebo depth camera to ROS. The
tutorial consists of 3 main steps:

  1. Create a Gazebo model that includes a ROS depth camera plugin
  2. Set up the depth camera in Gazebo
  3. View the depth camera's output in RViz.

This is a self-contained tutorial; it does not use the RRBot that is developed
in other Gazebo ROS tutorials. It is designed to help you get up and running
quickly using computer vision in ROS and Gazebo.

### Prerequisites

You should [install gazebo\_ros\_pkgs](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
before doing this tutorial.

## Create a Gazebo Model with a Depth Camera Plugin

Because Gazebo and ROS are separate projects that
do not depend on each other, sensors such as depth cameras do not include ROS
plugins by default. This means you have to make a custom camera based on those
in the Gazebo model repository, and then add your own `<plugin>` tag to make the
depth camera data publish point clouds and images to ROS topics.

You should choose a depth camera to use from those available in Gazebo. This
tutorial will use the Microsoft Kinect, but the procedure should be the
same for other depth cameras on the list.

First, acquire the depth camera and modify its name. We've packaged the Kinect
sensor from `gazebo_models` repository for you, so all you have to do is
[download](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/ros_depth_camera/files/kinect.zip)
and unzip it.

Alternatively, you can follow the
[model contribution tutorial](http://gazebosim.org/tutorials?tut=model_contrib&cat=build_robot)
to make your own camera from scratch, or you can clone the `gazebo_models`
repository and copy one of the sensors from there.

However you acquire it, copy the `kinect` folder into your
`~/.gazebo/models` directory. Then, change the name of your model to something
meaningful, like `kinect_ros`. To change the model's name, you should update
the folder name, the `<name>` stored in the `.config` file, and the `model name`
in the `model.sdf` file.

Now you need to add the ROS plugin to publish depth camera information and
output to ROS topics. A list of ROS plugins, with example code, can be found in
the
[plugins tutorial](http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros).

In this tutorial, you'll be using the generic "Openni Kinect" plugin. You can and
should use this plugin for other types of depth cameras besides the Kinect (it
is an older plugin, and so it retains its old name).

Open the `model.sdf` file in your new model's directory. Add the following SDF
markup inside the `<sensor>` tag, immediately after the closing `</camera>` tag:

~~~
        <plugin name="camera_plugin" filename="libgazebo_ros_openni_kinect.so">
          <baseline>0.2</baseline>
          <alwaysOn>true</alwaysOn>
          <!-- Keep this zero, update_rate in the parent <sensor> tag
            will control the frame rate. -->
          <updateRate>0.0</updateRate>
          <cameraName>camera_ir</cameraName>
          <imageTopicName>/camera/depth/image_raw</imageTopicName>
          <cameraInfoTopicName>/camera/depth/camera_info</cameraInfoTopicName>
          <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
          <depthImageInfoTopicName>/camera/depth/camera_info</depthImageInfoTopicName>
          <pointCloudTopicName>/camera/depth/points</pointCloudTopicName>
          <frameName>camera_link</frameName>
          <pointCloudCutoff>0.05</pointCloudCutoff>
          <distortionK1>0</distortionK1>
          <distortionK2>0</distortionK2>
          <distortionK3>0</distortionK3>
          <distortionT1>0</distortionT1>
          <distortionT2>0</distortionT2>
          <CxPrime>0</CxPrime>
          <Cx>0</Cx>
          <Cy>0</Cy>
          <focalLength>0</focalLength>
          <hackBaseline>0</hackBaseline>
        </plugin>
~~~

As you can see, this plugin allows you a lot of fine-grained control over how
the information is passed to ROS. A few points to note:

  * The `updateRate` parameter should be set to 0, which will cause the plugin
  to publish depth information as the same rate as the parent SDF `sensor`. If
  updateRate is not 0, it will do additional throttling on top of the parent
  `sensor`'s update rate.
  * The topic names and `frameName` can be set to whatever you'd like, but the
  ones shown above match the default topics that are published by commonly used
  ROS packages, such as `openni2_launch`. Keeping the topic names the same will
  help make switching between real and simulated cameras easier.
  * The `distortionX` parameters should match those in the `<distortion>` tag of
  the parent camera. If there is no `<distortion>` tag, then use 0 for all the
  `distortionX` values.
  * `pointCloudCutoff` is the maximum distance for points. No points beyond
  this distance will be shown. This is an additional restriction to any
  clipping that has been set in the parent sensor.

Once you've renamed the model, added the above code to your `.sdf` file, and
saved your changes, you should be ready to roll!

## Set up the Depth Camera in Gazebo

Open Gazebo with ROS support enabled (e.g.
`roslaunch gazebo_ros gazebo_ros empty_world.launch`). Use the
Insert panel to find your "Kinect ROS" model, and insert it into the world!

**Important:** You should also add some other objects to the scene, otherwise your
camera might not have anything to see! Add some cubes, spheres, or anything
else, and make sure they are located in the visible range of the camera, like in
the screenshot below.

[[file:depth_camera_scene.png|600px]]

By default, the Kinect is not a static object in Gazebo. You may want to further
edit your `.sdf` to add `<static>true</static>`, which will allow your camera to
float in the air. This is probably much easier than recreating your entire
sensing setup using physically correct models.

## View Depth Camera Output in RViz

Now that the camera is in the Gazebo scene, it should be publishing images and
point clouds to ROS topics. You can check the topics that are being published
by running `rostopic list` in a new terminal. You should see the topics you
specified in the SDF plugin code listed.

Now you can run RViz (`rosrun rviz rviz`). First, set the RViz Fixed Frame in
the left panel's Global Options section to match the value you set for
`<frameName>` in the plugin XML code. Then, you can add a PointCloud2 and an
Image display to RViz. For an Image, set the Image Topic to the value you used
in the `<imageTopicName>` tag. For the PointCloud2, set the Topic to the name
you used in the `<depthImageTopicName>` tag. See the screenshot below for an
example that matches the values in the example sensor XML above:

[[file:rviz_topics.png|600px]]

After setting the correct topics and fixed frame, you should see something
similar to the following from the PointCloud2:

[[file:depth_camera_rviz.png|600px]]

An Image display will show a grayscale version of the depth camera results.
It should match what's seen in Gazebo if you use the Topic Visualizer on the
depth camera.

### Troubleshooting

**Problem:** `rostopic list` shows no camera topics.

*Solution:* Make sure you added the correct model in Gazebo. Make sure that
the Gazebo simulation is running, not paused. Check the `model.sdf` file and
ensure that the `<plugin>` tag is in the correct location in the file. Try
running Gazebo in verbose mode (`rosrun gazebo_ros gazebo --verbose`) and
see if there are any helpful warning or error messages that can help pinpoint
the problem.

**Problem:** The ROS topics are listed, but I don't see anything in Rviz.

**Solution:** Make sure that there are objects for the camera to see in Gazebo.
Make sure that you have an Image or PointCloud2 display added in RViz. Check
that your Image or PointCloud2 displays are set to show the correct topic. Check
that the Image or PointCloud2 displays are not disabled (checkbox).
Ensure that your RViz Fixed Frame matches the `frameName` you specified in
the `<plugin>` tag. Ensure that the sensor clipping parameters are not set up
so that all points are being clipped.