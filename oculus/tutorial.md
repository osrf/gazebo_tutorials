# Tutorial: Using the Oculus Rift VR headset

Gazebo supports the [Oculus Rift VR headset](http://www.oculusvr.com/). You will be able to attach a virtual Oculus Camera to one of the visual links of your model.

## OculusVR SDK installation.

Follow the next instructions to install the Oculus SDK version that we prepared for Gazebo.

~~~
hg clone https://bitbucket.org/osrf/oculussdk
cd oculussdk
mkdir build
cd build
cmake ..
make
sudo make install
sudo cp ../LibOVR/90-oculus.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
~~~

## Gazebo compilation with OculusVR support.

Once Oculus VR SDK is installed, you should be able to compile Gazebo from source with Oculus Rift support.

Follow [this](http://gazebosim.org/tutorials/?tut=install) instructions to compile Gazebo. During the executing of the `cmake` command, you should see this message confirming that the Oculus SDK is found:

~~~
-- checking for module 'OculusVR'
--   found OculusVR, version 0.2.5
~~~

## Configuring and running Gazebo with Oculus Rift support.

Before starting Gazebo, open up your favourite editor and edit the file `~/.gazebo/gui.ini` file with the following content:

~~~
[geometry]
x=0
y=0

[oculus]
x=<REPLACE_BY_YOUR_VERTICAL_RESOLUTION>
y=0
visual=<REPLACE_BY_THE_VISUAL_LINK_ATTACHED_TO_OCULUS>
autolaunch=0
~~~

Do not forget to replace `x` with the value of the vertical resolution of your monitor, and `visual` with the name of the visual link that you want to use with Oculus Rift. Gazebo will create a new window in the (x,y) coordinates, that should be rendered in the Oculus headset.

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

After plugging in and enable your Oculus Rift headset, go ahead and start Gazebo with a world containing a camera:

~~~
gazebo worlds/camera.world
~~~

Once Gazebo is up and running, click on Window->Oculus Rift and you should be able to see the world from your Oculus headset.
