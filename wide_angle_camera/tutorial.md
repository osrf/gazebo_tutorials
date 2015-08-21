# Tutorial

## Introduction

This tutorial shows how to create use and setup wide-angle camera sensor.
The difference between regular camera and wide-angle is in projection types:
The regular camera sensor uses only [pinhole projection](https://en.wikipedia.org/wiki/Pinhole_camera), while wide-anglecamera have much more options.
For moderate view angle lens distortion could be simulated, as described in [corresponding tutorial](http://gazebosim.org/tutorials?tut=camera_distortion).
But if you need a camera with fisheye, or other type of wide-angle lens, whose field of view is `120°`, `180°` or `270°` then you should use wide-angle camera sensor.

## What to know?

* You should already know how to use camera sensor.
* Be familiar with plugins.

## Try it!

1. Create a new folder `fisheye-camera` in `~/.gazebo/models` directory.
  ~~~
  mkdir -p ~/.gazebo/models/fisheye-camera
  ~~~

1. Create a new text file in it called `model.config`

  ~~~
  gedit ~/.gazebo/models/fisheye-camera/model.sdf
  ~~~

  And paste the following content:

  ~~~
  <?xml version="1.0"?>
  <model>
    <name>Fisheye Camera</name>
    <version>1.0</version>
    <sdf version="1.5">model.sdf</sdf>
    <author>
      <name>My name</name>
      <email>e-mail@example.com</email>
    </author>
    <description>
      A simple fish-eye camera with a box for visualization.
    </description>
  </model>
  ~~~

1. Create a second file in the same directory called `model.sdf`

  ~~~
  gedit ~/.gazebo/models/fisheye-camera/model.sdf
  ~~~

  with the content:

  ~~~
  <?xml version="1.0" ?>
  <sdf version="1.5">
    <model name="wideanglecamera">
      <pose>0 0 0.05 0 0 0</pose>
      <link name="link">
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
        <sensor name="camera" type="wideanglecamera">
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
            <lens>
              <type>gnomonical</type>
              <scale_to_hfov>true</scale_to_hfov>
              <cutoff_angle>1.5707</cutoff_angle>
              <env_texture_size>512</env_texture_size>
            </lens>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
        </sensor>
      </link>
    </model>
  </sdf>
  ~~~

1. Start Gazebo:

  ~~~
  gazebo
  ~~~

1. Add several objects to the world, cubes, spheres, some objects from warehouse.
In the `Insert` tab you should see your brand-new "Fisheye camera" model.
Click on it and place anywhere in your world.

1. Press `Ctrl+T` to open list of topics, find corresponding element in `ImageStamped` section and start visualization.  
What you should see is exactly the same what you'd see using a regular camera sensor.

### Lets make things more interesting

* Increase `horizontal_fov` value to `3.1415`, in your `model.sdf` and change `type` to `stereographic`. Save it.
* Add a new instance of `Fisheye camera`. A new topic will be added.
* Switch to the topic corresponding to your second camera in visualization window.

Now you shold have something like this (except for the sky, if you don't enable it):

[[file:files/180.png|600px]]

* Now change lens `type` to `custom` and add following code to the `lens` section of `model.sdf`:

~~~
<custom_function>
  <c1>1.05</c1>
  <c2>4</c2>
  <f>1.0</f>
  <fun>tan</fun>
</custom_function>
~~~

* set `cutoff_angle` to `3.1415`
* `horizontal_fov` value to `6.2831`

Add one more camera. In topic visualization for this camera you should now see a whole `360°` degree image of the world:

[[file:files/360.png|600px]]

## What is going on?

~~~
<!-- You should be already familiar with sdf definition of the `camera` sensor, the only difference is that you change it's `type` to `wideanglecamera` -->
<sensor name="camera" type="wideanglecamera">
  <camera>
    <horizontal_fov>6.283</horizontal_fov>
    <image>
      <width>320</width>
      <height>240</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>

     <!-- A new section called `lens`. -->
    <lens>
      <!-- type element is mandatory -->
      <type>custom</type>

      <!-- manually defined mapping function r = c1*f*fun(theta/c2 + c3), More information [here](https://en.wikipedia.org/wiki/Fisheye_lens#Mapping_function) -->
      <custom_function>
        <c1>1.05</c1>   <!-- linear scaling -->
        <c2>4</c2>      <!-- angle scaling -->
        <f>1.0</f>      <!-- one more scaling parameter -->
        <fun>tan</fun>  <!-- one of sin,tan,id -->
      </custom_function>

      <!-- if it is set to `true` your horizontal FOV will ramain as defined, othervise it depends on lens type and custom function if there is one -->
      <scale_to_hfov>true</scale_to_hfov>
      <!-- clip everything that is outside of this angle -->
      <cutoff_angle>3.1415</cutoff_angle>
      <!-- resolution of the cubemap texture, the highter it is - the sharper is your image -->
      <env_texture_size>512</env_texture_size>
    </lens>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
</sensor>
</link>
~~~

# Plugin Example

It is possible play a bit with different `wideanglecamera` `lens` settings with a plugin.
The following section requires you to build a plugin from source.
You will need Gazebo headers to build it.
If you install Gazebo from source then you should already have the necessary files.
If you install the Gazebo binary deb version,
then you'll need to install dev packages according to your Gazebo and sdformat versions.

1. Create directory and pull source files:

  ~~~
  mkdir lens_control_plugin && cd lens_control_plugin
  curl -O https://bitbucket.org/0xb000/gazebo/raw/camera_lens_example_isolated/examples/plugins/camera_lens_control/CMakeLists.txt
  curl -O https://bitbucket.org/0xb000/gazebo/raw/camera_lens_example_isolated/examples/plugins/camera_lens_control/CameraLensControlExample.hh
  curl -O https://bitbucket.org/0xb000/gazebo/raw/camera_lens_example_isolated/examples/plugins/camera_lens_control/CameraLensControlExample.cc
  curl -O https://bitbucket.org/0xb000/gazebo/raw/camera_lens_example_isolated/examples/plugins/camera_lens_control/example.world
  curl -O https://bitbucket.org/0xb000/gazebo/raw/camera_lens_example_isolated/examples/plugins/camera_lens_control/mainwindow.ui
  ~~~

1. Create a build directory

  ~~~
  mkdir build
  cd build
  ~~~

1. Create symlink to a UI file used by a plugin

  ~~~
  ln -s ../mainwindow.ui mainwindow.ui
  ~~~

1. Configure and build plugin

  ~~~
  cmake ..
  make
  ~~~

1. Tell Gazebo to search for plugins in current directory

  ~~~
  export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
  ~~~

1. Start Gazebo with the example world

  ~~~
  gazebo ../example.world
  ~~~

1. Now add some objects to the world and click `Spawn` button in GUI overlay window.
A new instance of `wideanglecamera` will be created, open it's topic visualization.

1. Select `wideanglecamera_0`, now you can modify lens properties interactively with the provided controls.

# FAQ

### What lens types are supported?

`gnomonical`, `stereographic`, `equidistant`, `equisolid_angle`, `orthographic`

### How to choose cubemap texture resolution?

It depends on your FOV, the value specified in `env_texture_size` is a number of pixels for `90°` angle, so keep your pixel density in sane range.

### How to choose cut-off angle?

Normally keep it twice as low as HFOV for circular image, for full-frame you may not to specify it at all. If you disable scaling, it's up to you.

### I want a full-frame image with diagonal FOV!

Do not specify cut-off angle, it will be set to `180°`.
Calculate your new `horizontal_fov` using following formula:

[[file::files/dfov.gif]]

###### where:
* `dfov` is your diagonal FOV;
* [[file:files/r.gif]] is aspect ratio of your frame;
* [[file:files/fun_inv.gif]] is an inverse to `fun`.

### Should I use wide-angle camera instead of a regular camera?

If regular camera sensor is sufficent for your needs then use it, wide-angle camera sensor gives a significant performance hit on slow hardware.