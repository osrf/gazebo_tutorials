# Tutorial

## Introduction

This tutorial shows how to create use and setup wide anglecamera sensor.
The difference between regular camera and wideangle is in projection types:
The regular camera sensor uses only [pinhole projection](https://en.wikipedia.org/wiki/Pinhole_camera), while wideangle camera have much more options.
For moderate view angles lens ditortions could be simulated, as described in [corresponding tutorial](http://gazebosim.org/tutorials?tut=camera_distortion).
But if you need a camera with fisheye, or other type of wide angle lens, whose field of view is `120°`, `180°` or `270°` then you should use wide angle camera sensor.

## What to know?

* You should already know how to use camera sensor.
* Be familiar with plugins.

## Try it!

1. Create a new folder `wideanglecamera` in `~/.gazebo/models` directory.

2. Create a new text file in it called `model.config` with the following content:

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

2. Create a second file in the same directory called `model.sdf` with the content:

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

4. Start Gazebo:

  ~~~
  gazebo
  ~~~

5. Add several objects to the world, cubes, spheres, some objects from warehouse.
In the `Insert` tab you should see your brand-new "Fisheye camera" model.
Click on it and place anywhere in your world.

6. Press `Ctrl+T` to open list of topics, find corresponding element in `ImageStamped` section and start visualisation.  
What you should see is exactly the same what you'd see using a regular camera sensor.

### Lets make things more interesting

* Increase `horizontal_fov` value to `3.1415`, in your `model.sdf` and change `type` to `stereographic`. Save it.
* Add a new instance of Fisheye camera. A new topic will be added.
* Switch to the topic corresponding to your second camera in visualisation window.

* Now change lens `type` to `custom` and add following code to the `lens` section of `model.sdf`:

~~~
<custom_function>
  <c1>1.05</c1>
  <c2>4</c2>
  <f>1.0</f>
  <fun>tan</fun>
</custom_function>
~~~

<!-- * change `scale_to_hfov` to `false` -->
* `cutoff_angle` to `3.1415`

Add one more camera. In topic visualisation for this camera you should now see a whole `360°` degree image of the world.

## What is going on?

~~~
// You should be already familiar with sdf definition of the `camera` sensor, the only difference is that you change it's `type` to `wideanglecamera`
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

      <!-- if it is set to `true` your horisontal FOV will ramain as defined, othervise it depends on lens type and custom function if there is one -->
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

It is possible play a bit with different wideangle camera lens settings with a plugin.
The followind section requires you to build a plugin from source.
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

2. Configure and build plugin

  ~~~
  cmake ..
  make
  ~~~

3. Tell Gazebo to search for plugins in current directory

  ~~~
  export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
  ~~~

4. Start Gazebo with the example world

  ~~~
  gazebo ./example.world
  ~~~

5. Now add some objects to the world and click `Spawn` button in GUI overlay window.
A new instance of `wideanglecamera` will be created, open it's topic visualisation.

6. Select `wideanglecamera_0`, now you can control lens properties interactively with provided controls.

# FAQ

### What lens types are supported?

gnomonical, stereographic, equidistant, equisolid_angle, orthographic

### How to choose cubemap texture resolution?

It depends on your FOV, the value specified in env_texture_size is a number of pixels for 90° angle, so keep your pixel dencity in sane range.

### How to choose cut-off angle?

Normaly keep it twice as low as HFOV for circular image, for full-frame you may not to specify it at all. If you disable scaling, it's up to you.

### I want a full-frame image with diagonal FOV!

Do not specify cut-off angle, it will be set to 180°.
Calculate your new `horisontal_fov` using following formula:

<!-- ![hfov = 2c_2\cdot fun^{-1}(fun(\frac{dfov}{2c_2})\cdot \sqrt{1+r^2});](http://latex.codecogs.com/gif.download?hfov%20%3D%202c_2%5Ccdot%20fun%5E%7B-1%7D%28fun%28%5Cfrac%7Bdfov%7D%7B2c_2%7D%29%5Ccdot%20%5Csqrt%7B1+r%5E2%7D%29%3B)  
###### where:
* `dfov` is your diagonal FOV;  
* ![r = \frac{width}{height}](http://latex.codecogs.com/gif.download?r%20%3D%20%5Cfrac%7Bwidth%7D%7Bheight%7D) is aspect ratio of your frame;  
* ![fun^{-1}](http://latex.codecogs.com/gif.download?fun%5E%7B-1%7D) is an inverse to `fun`. -->

![hfov = 2c_2\cdot fun^{-1}(fun(\frac{dfov}{2c_2})\cdot \sqrt{1+r^2});](file:files/dfov.gif)  
###### where:
* `dfov` is your diagonal FOV;  
* ![r = \frac{width}{height}](file:files/r.gif) is aspect ratio of your frame;  
* ![fun^{-1}](file:files/fun_inv.gif) is an inverse to `fun`.

### Should I use wideangle camera instead of regular camera?

If regular camera sensor is sufficent for your needs then use it, Wideangle camera sensor gives a significant performance hit on slow hardware.