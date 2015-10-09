# Overview

This tutorial shows how to use a wide-angle camera sensor.  A regular camera
uses a [pinhole projection](https://en.wikipedia.org/wiki/Pinhole_camera),
while a wide-anglecamera haa more options. Distortion for a moderate view
angle lens can be simulated according to [this
tutorial](http://gazebosim.org/tutorials?tut=camera_distortion). But
a camera with a fisheye, or other type of wide-angle lens whose field of
view is `120°`, `180°` or `270°` will require a wide-angle camera sensor.

## What to know?

* You should already know how to use camera sensor.
* Be familiar with [plugins](http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin)

## Try it!

### Create a model

1.  Create a new folder `fisheye-camera` in `~/.gazebo/models` directory

        mkdir -p ~/.gazebo/models/fisheye-camera

1.  Create a new text file in it named `model.config`

        gedit ~/.gazebo/models/fisheye-camera/model.config

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

1.  Create a second file in the same directory named `model.sdf`

        gedit ~/.gazebo/models/fisheye-camera/model.sdf

    With the content:

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

1.  Start Gazebo:

        gazebo &

1.  Add several objects to the world, cubes, spheres, some objects from warehouse.
In the `Insert` tab you should see your brand-new "Fisheye camera" model.
Click on it and place anywhere in your world.

1.  Press `Ctrl+T` to open list of topics, find corresponding element in `ImageStamped` section and start visualization.
What you should see is exactly the same what you'd see using a regular camera sensor.


### Lets make things more interesting

*   In your `model.sdf`'s `<lens>` element,  increase `<horizontal_fov>` to `3.1415` and change the `<type>` to `stereographic`. Save it.
*   Add a new instance of `Fisheye camera`. A new topic will be added.
*   Switch to the topic corresponding to your second camera in visualization window.

Now you shold have something like this (except for the sky, if you don't enable it):

[[file:files/180.png|600px]]

*   Now change lens's `<type>` to `custom` and add following code to the `<lens>` section of `model.sdf`:

    ~~~
    <custom_function>
      <c1>1.05</c1>
      <c2>4</c2>
      <f>1.0</f>
      <fun>tan</fun>
    </custom_function>
    ~~~

*   set `<horizontal_fov>` value to `6.2831`
*   `<cutoff_angle>` to `3.1415`

The SDF should be looking like this:

%%%
<sensor name="camera" type="wideanglecamera">
  <camera>
%%%
~~~
    <horizontal_fov>6.2831</horizontal_fov>
~~~
%%%
    <image>
      <width>320</width>
      <height>240</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <lens>
%%%
~~~
      <type>custom</type>
      <custom_function>
        <c1>1.05</c1>
        <c2>4</c2>
        <f>1.0</f>
        <fun>tan</fun>
      </custom_function>
      <cutoff_angle>3.1415</cutoff_angle>
~~~
%%%
      <scale_to_hfov>true</scale_to_hfov>
      <env_texture_size>512</env_texture_size>
    </lens>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
</sensor>
%%%

Add one more camera. In topic visualization for this camera you should now see
a whole `360°` degree image of the world:

[[file:files/360.png|600px]]


## What is going on?

You should already be familiar with the SDF definition of a `camera` sensor.
Here, you must change its `type` to `wideanglecamera`.

~~~
<sensor name="camera" type="wideanglecamera">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
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
~~~

We add a new section named `lens` where the type element is mandatory:

~~~
    <lens>
      <type>custom</type>
~~~

For custom lenses, we can manually define the mapping function `r = c1*f*fun(theta/c2 + c3)`.
(More information [here](https://en.wikipedia.org/wiki/Fisheye_lens#Mapping_function))

~~~
      <custom_function>
        <c1>1.05</c1>   <!-- linear scaling -->
        <c2>4</c2>      <!-- angle scaling -->
        <f>1.0</f>      <!-- one more scaling parameter -->
        <fun>tan</fun>  <!-- one of sin,tan,id -->
      </custom_function>
~~~

If `scale_to_hfov` is set to `true` your horizontal FOV will ramain as defined.
Otherwise it depends on lens type and custom function, if there is one.

~~~
      <scale_to_hfov>true</scale_to_hfov>
~~~

Clip everything that is outside of the `cutoff_angle`.

~~~
      <cutoff_angle>3.1415</cutoff_angle>
~~~

Sets the resolution of the cubemap texture, the highter it is - the sharper is
your image.

~~~
      <env_texture_size>512</env_texture_size>
    </lens>
  </camera>
</sensor>
</link>
~~~


## Plugin Example

It is possible to adjust the `lens` settings from a plugin.  This section
requires you to build a plugin from source.  You will need Gazebo headers to
build it.  If you install Gazebo from source then you should already have
the necessary files.  If you installed the Gazebo binary deb version, then
you'll need to [install](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=6.0&cat=install)
`-dev` packages according to your Gazebo and sdformat versions.

1.  Create directory and pull source files:

    ~~~
    mkdir ~/lens_control_plugin && cd ~/lens_control_plugin
    wget https://bitbucket.org/osrf/gazebo/raw/default/examples/plugins/camera_lens_control/CMakeLists.txt
    wget https://bitbucket.org/osrf/gazebo/raw/default/examples/plugins/camera_lens_control/CameraLensControlExample.hh
    wget https://bitbucket.org/osrf/gazebo/raw/default/examples/plugins/camera_lens_control/CameraLensControlExample.cc
    wget https://bitbucket.org/osrf/gazebo/raw/default/examples/plugins/camera_lens_control/example.world
    wget https://bitbucket.org/osrf/gazebo/raw/default/examples/plugins/camera_lens_control/mainwindow.ui
    ~~~

1.  Create a build directory

    ~~~
    mkdir build
    cd build
    ~~~

1.  Create a symlink to the UI file used by the plugin

    ~~~
    ln -s ../mainwindow.ui mainwindow.ui
    ~~~

1.  Configure and build plugin

    ~~~
    cmake ..
    make
    ~~~

1.  Tell Gazebo to search for plugins in current directory

        export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH

1.  Start Gazebo with the example world

        gazebo ../example.world


1.  Now add some objects to the world and click `Spawn` button in GUI
overlay window.  A new instance of `wideanglecamera` will be created, open
its topic visualization.

1.  Select `wideanglecamera_0`, now you can modify lens properties
interactively with the provided controls.


## FAQ

### What lens types are supported?

`gnomonical`, `stereographic`, `equidistant`, `equisolid_angle`, `orthographic`

### How to choose cubemap texture resolution?

It depends on your FOV, the value specified in `env_texture_size` is
a number of pixels for `90°` angle, so keep your pixel density in sane
range.

### How to choose cut-off angle?

Normally keep it twice as low as HFOV for circular image, for full-frame you
may not to specify it at all. If you disable scaling, it's up to you.

### I want a full-frame image with diagonal FOV!

Do not specify cut-off angle, it will be set to `180°`.  Calculate your new
`<horizontal_fov>` using following formula:

[[file:files/dfov.gif]]

###### where:
*   [[file:files/dfov2.gif]] is your diagonal FOV;
*   [[file:files/r.gif]] is aspect ratio of your frame;
*   [[file:files/fun_inv.gif]] is an inverse to `fun`;
*   [[file:files/c_2.gif]] is a parameter from explicit mapping function definition, if you use one of the predefined lens types pick your parameters from the following table

    <table border="1" cellpadding="10">
    <tr>
    <th>type</th>
    <th>[[file:files/c_1.gif]]</th>
    <th>[[file:files/c_2.gif]]</th>
    <th>[[file:files/f.gif]]</th>
    <th>[[file:files/fun.gif]]</th>
    </tr>
    <tr>
    <td>gnomonical</td>
    <td>1</td>
    <td>1</td>
    <td>1</td>
    <td>tan</td>
    </tr>
    <tr>
    <td>stereographic</td>
    <td>2</td>
    <td>2</td>
    <td>1</td>
    <td>tan</td>
    </tr>
    <tr>
    <td>equidistant</td>
    <td>1</td>
    <td>1</td>
    <td>1</td>
    <td>id</td>
    </tr>
    <td>equisolid_angle</td>
    <td>2</td>
    <td>2</td>
    <td>1</td>
    <td>sin</td>
    </tr>
    <td>orthographic</td>
    <td>1</td>
    <td>1</td>
    <td>1</td>
    <td>sin</td>
    </tr>
    </table>

### Should I use wide-angle camera instead of a regular camera?

If a regular camera sensor is sufficent for your needs then use it.
Wide-angle camera sensors give significant performance hit on slow
hardware.
