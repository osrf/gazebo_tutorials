# Introduction

Cameras lenses typically exhibit some degree of optical distortions, which result in warping of images. An example is a fisheye camera that is commonly used in robotics applications to obtain a wider field of view of the environment for object recognition or navigation tasks.

Using camera calibration tools such as Matlab or OpenCV, it is possible to extract distortion coefficients along with other camera intrinsic parameters. With these distortion coeffient, users can now create a distorted camera sensor in Gazebo.

## Current Implementation

Gazebo currently support simulation of camera based on the Brown's distortion model. It expects 5 distortion coefficients `k1`, `k2`, `k3`, `p1`, `p1` that you can get from the camera calibration tools. The `k` coefficients are the radial components of the disotrtion model, while the `p` coefficients are the tegnetial components.

There are a few limitations with the current implementation that needs to be taken into account:

1. Only barrel distortion is supported at the moment, which typically has a negative `k1` value.

1. Distortion is applied to the camera image texture. This means taking the generated image data and just warping it. This has the caveat that the final image (espeically at the corners) has a narrower field of view than a real camera lens with barrel distortion. One workaround to compensate for this effect is to increase the field of view of the camera sensor in Gazebo.

# Creating a camera with distortion

To add a camera model with distortion:

1. Create a model directory:

    ~~~
    mkdir -p ~/.gazebo/models/distorted_camera
    ~~~

1.  Create a model config file:

    ~~~
    gedit ~/.gazebo/models/distorted_camera/model.config
    ~~~

1.  Paste in the following contents:

    ~~~
    <?xml version="1.0"?>
    <model>
      <name>Distorted Camera</name>
      <version>1.0</version>
      <sdf version='1.5'>model.sdf</sdf>

      <author>
       <name>My Name</name>
       <email>me@my.email</email>
      </author>

      <description>
        My distorted camera.
      </description>
    </model>
    ~~~

1.  Create a `~/.gazebo/models/distorted_camera/model.sdf` file.

    ~~~
    gedit ~/.gazebo/models/distorted_camera/model.sdf
    ~~~

1. Paste in the following, which is a copy of the standard camera model with the addition of distortion:

    ~~~
    <?xml version="1.0" ?>
    <sdf version="1.5">
      <model name="distorted_camera">
        <link name="link">
          <pose>0.05 0.05 0.05 0 0 0</pose>
          <inertial>
            <mass>0.1</mass>
          </inertial>
          <collision name="collision">
            <geometry>
              <box>
                <size>0.1 0.1 0.1</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>0.1 0.1 0.1</size>
              </box>
            </geometry>
          </visual>
          <sensor name="camera" type="camera">
            <camera>
              <horizontal_fov>1.047</horizontal_fov>
              <image>
                <width>320</width>
                <height>240</height>
              </image>
              <clip>
                <near>0.1</near>
                <far>100</far>
              </clip>
              <distortion>
                <k1>-0.25</k1>
                <k2>0.12</k2>
                <k3>0.0</k3>
                <p1>-0.00028</p1>
                <p2>-0.00005</p2>
                <center>0.5 0.5</center>
              </distortion>
            </camera>
            <always_on>1</always_on>
            <update_rate>30</update_rate>
            <visualize>true</visualize>
          </sensor>
        </link>
      </model>
    </sdf>
    ~~~

1. Start Gazebo:

        gazebo

1. Insert the distorted camera model: in the left pane, select the `Insert` tab, then click on `Distorted Camera`.  Drop your camera somewhere in the world, and put a box in front of it.

    [[file:files/distorted_camera_inserted.png|640px]]

1. See the distorted camera images: click on Window->Topic Visualization (or press Ctrl-T) to bring up the Topic Selector.

    [[file:files/distorted_camera_topic_visualizer.png|640px]]

1. Find the the topic with a name like `/gazebo/default/distorted_camera/link/camera/image` and click on it, then click `Ok`.  You'll get get a Camera View window that shows you the camera image data.

    [[file:files/distorted_camera_image_visualizer.png|640px]]

As you can see, the camera image is distorted, made obvious by the curve edges of the box. To adjust the distortion, simply play with the `k1`, `k2`, `k3`, `p1`, `p1` distortion coefficients in the `model.sdf`.
