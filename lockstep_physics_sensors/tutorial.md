# Introduction
Sometimes, it may be desired to strictly adhere to update rates specified for
sensors, even if it means slowing down physics to wait for sensor updates to
catch up.
For example, running a high-resolution camera sensor on a computer with limited
computing power could result in the camera updates lagging behind physics,
causing the camera frames to be out of sync with physics.

The lockstep feature, enabled by passing `--lockstep` to Gazebo server, allows
the sensor update rate to be strictly followed.
This means that on computers with different computing resources, the specified
update rate can always be respected, allowing for sensor updates and physics to
be in sync.

# Create a world with a camera

We will use a high-resolution high-frame-rate camera for illustration.

Create a world file:

        gedit camera_strict_rate.world

Paste in the following content:

~~~
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <model name="camera_model">
      <static>true</static>
      <pose>-10.0 0.0 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
        <!-- High fps high-res camera to test strict rate -->
        <sensor name="camera_sensor" type="camera">
          <camera>
            <horizontal_fov>1.0472</horizontal_fov>
            <image>
              <width>1280</width>
              <height>720</height>
            </image>
          </camera>
          <always_on>1</always_on>
          <!-- We choose a high fps on purpose. The goal is to check the effect
            of lockstep. -->
          <update_rate>500</update_rate>
          <visualize>true</visualize>
        </sensor>
        <!-- Regular camera, to make sure strict rate is only applied to the sensor intended -->
        <sensor name="camera_sensor_regular" type="camera">
          <camera>
            <horizontal_fov>1.0472</horizontal_fov>
            <image>
              <width>320</width>
              <height>240</height>
            </image>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <visualize>true</visualize>
        </sensor>
      </link>
    </model>

    <!-- Double pendulum -->
    <include>
      <name>active_pendulum</name>
      <uri>model://double_pendulum_with_base</uri>
      <pose>2 0 0 0 0 0</pose>
      <scale>0.5 0.5 0.5</scale>
    </include>

  </world>
</sdf>
~~~

This world contains a high-resolution (1280x720) high-frame-rate (500 fps)
camera, as well as a low-resolution (320x240) low-frame-rate (30 fps) camera
for comparison.
It also includes a pendulum, for visual verification.

# Run the world without lockstep

1. First, we will inspect what happens if we run the world normally, without the
lockstep feature.

        gazebo camera_strict_rate.world

1. Visualize the camera image: click on Window->Topic Visualization (or press
Ctrl-T) to bring up the Topic Selector.
Select the first item under the Image type.

   [[file:files/topic_selector.png|640px]]

1. Examine the Hz field. It is likely unable to reach the specified 500 fps.

   [[file:files/no_lockstep.png|640px]]

1. Take note of the real time factor at the bottom of the window. It should be
close to 1.0.

This shows us that without enabling lockstep, the default behavior updates the
sensors as fast as the computing resources allow.
For a sensor that demands high computing power, it may never reach the specified
update rate.

# Run the world with lockstep

1. Now we will run the world with lockstep enabled.

        gazebo --lockstep camera_strict_rate.world

1. Bring up the camera image visualization as before.
The Hz field should show 500.

   [[file:files/lockstep_highres_camera.png|640px]]

   The real time factor is likely less than 1.0. The exact number depends on your
   computing power.

   This shows that the sensor's update rate is strictly followed, and physics has
   slowed down in order to accommodate for the high update rate.

1. In the Topic drop-down list, we can switch to the low-frame-rate camera and
observe the Hz and real time factor.

   [[file:files/lockstep_regular_camera.png|640px]]

   The Hz field shows around 30.

   However, the real time factor is still less than 1.0. This is a caveat with
using the lockstep feature. When lockstep is enabled, poses in the scene are
updated using function callbacks, which are slower than message transport used
in the default setting (lockstep disabled). Therefore, overall simulation speed
will slow down when lockstep is enabled.
