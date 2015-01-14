# Tutorial: ROS Advanced Integration

## Dynamic Reconfigure

Some of the physics properties can be adjusted within Gazebo as we described in the [modifying a world tutorial](http://gazebosim.org/tutorials?tut=modifying_world&cat=build_world). In addition, we can modify this properties using ROS's dynamic reconfigure mechanism.

As an example, we'll invert the gravity in the simulation. Make sure you have the following installed for groovy:

~~~
sudo apt-get install ros-groovy-rqt-common-plugins ros-groovy-dynamic-reconfigure
~~~

Or these packages for hydro:

~~~
sudo apt-get install ros-hydro-rqt-common-plugins ros-hydro-dynamic-reconfigure
~~~

Start Gazebo:

~~~
rosrun gazebo_ros gazebo
~~~

Let's insert a model in gazebo before making any change in the physics. Go ahead and click ''Insert'' in the left side of the gazebo main window. Click on the ''Pioneer 2DX'' robot model.

Start RosGui tool for interacting with gazebo in runtime:

~~~
rosrun rqt_gui rqt_gui
~~~


[[file:RosGUI.png|600px]]

Resize the rqt (a.k.a. RosGui) tool until you see something similar to the upper picture. Click on ''gazebo'' in the left side of rqt. You will see the physics parameter list that you will be able to modify. Change the ''gravity_z'' parameter to ''+9.8'' and you should see how the gravity affects your robot.

[[file:pioneer_flying.png|600px]]

