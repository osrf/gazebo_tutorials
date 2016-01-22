# Introduction

Gazebo 6 and above supports the [Oculus Rift VR headset](http://www.oculusvr.com/) DK1 and DK2.

After this tutorial, you will be able to attach a virtual Oculus Camera to one of the visual links of your model.

# OculusVR SDK installation.

Follow the next instructions to install the Oculus SDK version that we prepared for Gazebo.

~~~
sudo apt-get install libusb-dev libudev-dev libxinerama-dev
hg clone https://bitbucket.org/osrf/oculussdk
cd oculussdk
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make
sudo make install
sudo cp ../LibOVR/90-oculus.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
~~~

# Gazebo compilation with OculusVR support.

Once Oculus VR SDK is installed, you should be able to compile Gazebo from source with Oculus Rift support.

Follow [this](http://gazebosim.org/tutorials?tut=install_from_source&cat=install) instructions to compile Gazebo. During the execution of the `cmake` command, you should see this message confirming that the Oculus SDK is found:

~~~
-- checking for module 'OculusVR'
--   found OculusVR, version 0.4.4
~~~

# Configuration

Before starting Gazebo, we must tell it to create a new window which will be
rendered by the Oculus headset.

1. Before turning on the Oculus headset, on Ubuntu, go to `Applications` >
`System Tools` > `Preferences` > `Displays`.

1. Plug in and turn on your Oculus Rift headset. Note that your main launcher
might have moved to the headset. You should make sure the Oculus window is
positioned correctly:

    1. Drag the Oculus display to the top right of the main monitor.

    1. Make sure the Oculus display is upright, you might need to set its
       `Rotation` to `Counterclockwise`.

    [[file:files/display_setup.png|640px]]

1. We need to tell Gazebo where we put the Oculus screen, that is, on the
top-right of the main display. The horizontal coordinate will be the end of
the main monitor (`x` = monitor width) and the vertical coordinate will be the
top of the screen (`y` = 0).

    [[file:files/display_setup2.png|640px]]

    Open up your favourite editor and edit the `~/.gazebo/gui.ini` file with
    the following content:

    ~~~
    [geometry]
    x=0
    y=0

    [oculus]
    x=<REPLACE_BY_YOUR_HORIZONTAL_RESOLUTION>
    y=0
    visual=<REPLACE_BY_THE_VISUAL_LINK_ATTACHED_TO_OCULUS>
    autolaunch=0
    ~~~

    Replace `x` with the value of the horizontal resolution of your monitor,
    and `visual` with the name of the visual which the Oculus Rift will be
    attached to.

    As an example, this is the content of our `gui.ini` file:

    ~~~
    [geometry]
    x=0
    y=0

    [oculus]
    x=2560
    y=0
    visual=camera::link::visual
    autolaunch=0
    ~~~

# Running

1. In a terminal, run the `oculusd` daemon.

    ~~~
    cd ~/oculussdk
    ./oculusd
    ~~~

1. In a second terminal, run Gazebo

    ~~~
    gazebo worlds/camera.world
    ~~~

1. Once Gazebo is up and running, click on Window->Oculus Rift and you should be able to see the world from your Oculus headset.

It is also possible to enable the Oculus window by default when starting Gazebo. Modify the `gui.ini` file and set `autolaunch=1`. Now, start gazebo and your Oculus Rift should be working without any intervention:

~~~
gazebo worlds/camera.world
~~~
