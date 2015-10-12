# Tutorial: Using roslaunch to start Gazebo, world files and URDF models

There are many ways to start Gazebo, open world models and spawn robot models into the simulated environment. In this tutorial we cover the ROS-way of doing things: using `rosrun` and `roslaunch`. This includes storing your URDF files in ROS packages and keeping your various resource paths relative to your ROS workspace.

## Using `roslaunch` to Open World Models

The [roslaunch](http://www.ros.org/wiki/roslaunch) tool is the standard method for starting ROS nodes and bringing up robots in ROS. To start an empty Gazebo world similar to the `rosrun` command in the previous tutorial, simply run

<pre>
roslaunch gazebo_ros empty_world.launch
</pre>

### `roslaunch` Arguments

You can append the following arguments to the launch files to change the behavior of Gazebo:

**paused**

  > Start Gazebo in a paused state (default false)

**use_sim_time**

  > Tells ROS nodes asking for time to get the Gazebo-published simulation time, published over the ROS topic /clock (default true)

**gui**

  > Launch the user interface window of Gazebo (default true)

**headless**

  > Disable any function calls to simulator rendering (Ogre) components. Does not work with gui:=true (default false)

**debug**

  > Start gzserver (Gazebo Server) in debug mode using gdb (default false)

### Example `roslaunch` command

Normally the default values for these arguments are all you need, but just as an example:

<pre>
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false headless:=false debug:=true
</pre>

### Launching Other Demo Worlds

Other demo worlds are already included in the `gazebo_ros` package, including:

<pre>
roslaunch gazebo_ros willowgarage_world.launch
roslaunch gazebo_ros mud_world.launch
roslaunch gazebo_ros shapes_world.launch
roslaunch gazebo_ros rubble_world.launch
</pre>

Notice in `mud_world.launch` a simple jointed mechanism is launched. The launch file for `mud_world.launch` contains the following:

~~~
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="worlds/mud.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
</launch>
~~~

In this launch file we inherit most of the necessary functionality from empty_world.launch. The only parameter we need to change is the `world_name` parameter, substituting the `empty.world` world file with the `mud.world` file. The other arguments are simply set to their default values.

### World Files

Continuing with our examination of the `mud_world.launch` file, we will now look at the contents of the `mud.world` file. The first several components of the mud world is shown below:

~~~
  <sdf version="1.4">
    <world name="default">
      <include>
        <uri>model://sun</uri>
      </include>
      <include>
        <uri>model://ground_plane</uri>
      </include>
      <include>
        <uri>model://double_pendulum_with_base</uri>
        <name>pendulum_thick_mud</name>
        <pose>-2.0 0 0 0 0 0</pose>
      </include>
      ...
    </world>
  </sdf>
~~~

**See the section below to view this full world file on your computer.**

In this world file snippet you can see that three models are referenced. The three models are searched for within your local Gazebo Model Database. If not found there, they are automatically pulled from Gazebo's online database.

You can learn more about world files in the [Build A World](http://gazebosim.org/tutorials?cat=build_world) tutorial.

#### Finding World Files On Your Computer
World files are found within the `/worlds` directory of your Gazebo resource path. The location of this path depends on how you installed Gazebo and the type of system your are on. To find the location of your Gazebo resources, use the following command:

<pre>
env | grep GAZEBO_RESOURCE_PATH
</pre>

An typical path might be something like `/usr/local/share/gazebo-1.9`. Add `/worlds` to the end of the path and you should have the directory containing the world files Gazebo uses, including the `mud.world` file.

## Creating your own Gazebo ROS Package

Before continuing on how to spawn robots into Gazebo,
we will first go over file hierarchy standards for using
ROS with Gazebo so that we can make later assumptions.

For now, we will assume your catkin workspace is named `catkin_ws`,
though you can name this to whatever you want.
Thus, your catkin workspace might be located on your computer at something like:

<pre>
/home/user/catkin_ws/src
</pre>

Everything concerning your robot's model and description is located,
as per ROS standards, in a package named `/MYROBOT_description`
and all the world files and launch files used with Gazebo is located in a ROS package named
`/MYROBOT_gazebo`. Replace 'MYROBOT' with the name of your bot in lower case letters.
With these two packages, your hierarchy should be as follows:

<pre>
../catkin_ws/src
    /MYROBOT_description
        package.xml
        CMakeLists.txt
        /urdf
            MYROBOT.urdf
        /meshes
            mesh1.dae
            mesh2.dae
            ...
        /materials
        /cad
    /MYROBOT_gazebo
        /launch
            MYROBOT.launch
        /worlds
            MYROBOT.world
        /models
            world_object1.dae
            world_object2.stl
            world_object3.urdf
        /materials
        /plugins
</pre>

Remember that the command `catkin_create_pkg` is used for creating new packages,
though this can also easily be adapted for rosbuild if you must.
Most of these folders and files should be self explanatory.

The next section will walk you through making some of this setup for use with a custom world file.

### Creating a Custom World File

You can create custom `.world` files within your own ROS packages that are specific to your robots and packages. In this mini tutorial we'll make an empty world with a ground, a sun, and a gas station. The following is our recommended convention. Be sure to replace MYROBOT with the name of your bot, or if you don't have a robot to test with just replace it with something like 'test':

* Create a ROS package with the convention MYROBOT_gazebo
* Within this package, create a `launch` folder
* Within the `launch` folder create a YOUROBOT.launch file with the following contents (default arguments excluded):

~~~
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find MYROBOT_gazebo)/worlds/MYROBOT.world"/>
    <!-- more default parameters can be changed here -->
  </include>
</launch>
~~~

* Within the same package, create a `worlds` folder, and create a MYROBOT.world file with the following contents:

~~~
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://gas_station</uri>
      <name>gas_station</name>
      <pose>-2.0 7.0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
~~~

* You should now be able to launch your custom world (with a gas station) into Gazebo using the following command:

~~~
. ~/catkin_ws/devel/setup.bash
roslaunch MYROBOT_gazebo MYROBOT.launch
~~~

You should see the following world model (zoom out with the scroll wheel on your mouse):

[[file:figs/GasStation.png|800px]]

### Editing the World File Within Gazebo

You can insert additional models into your robot's world file and use the `File->Save` As command to export your edited world back into your ROS package.

## Using `roslaunch` to Spawn URDF Robots

There are two ways to launch your URDF-based robot into Gazebo using `roslaunch`:

**ROS Service Call Spawn Method**

  > The first method keeps your robot's ROS packages more portable between computers and repository check outs. It allows you to keep your robot's location relative to a ROS package path, but also requires you to make a ROS service call using a small (python) script.


**Model Database Method**

  > The second method allows you to include your robot within the `.world` file, which seems cleaner and more convenient but requires you to add your robot to the Gazebo model database by setting an environment variable.

We will go over both methods. Overall our recommended method is using the '''ROS Service Call Spawn Method'''

### "ROS Service Call" Robot Spawn Method

This method uses a small python script called `spawn_model` to make
a service call request to the `gazebo_ros` ROS node
(named simply "gazebo" in the rostopic namespace) to add a custom URDF into Gazebo.
The `spawn_model` script is located within the `gazebo_ros` package.
You can use this script in the following way:

~~~
rosrun gazebo_ros spawn_model -file `rospack find MYROBOT_description`/urdf/MYROBOT.urdf -urdf -x 0 -y 0 -z 1 -model MYROBOT
~~~

To see all of the available arguments for `spawn_model` including namespaces, trimesh properties, joint positions and RPY orientation run:

<pre>
rosrun gazebo_ros spawn_model -h
</pre>

#### URDF Example with Baxter

If you do not yet have a URDF to test, as an example you can download the baxter_description package from Rethink Robotics's [baxter_common](https://github.com/RethinkRobotics/baxter_common) repo. Put this package into your catkin workspace by running:

<pre>
git clone https://github.com/RethinkRobotics/baxter_common.git
</pre>

You should now have a URDF file named `baxter.urdf` located in a  within baxter_description/urdf/, and you can run:

~~~
rosrun gazebo_ros spawn_model -file `rospack find baxter_description`/urdf/baxter.urdf -urdf -z 1 -model baxter
~~~

You should then see something similar to:

[[file:figs/Gas_baxter.png|800px]]

To integrate this directly into a ROS launch file, reopen the file `MYROBOT_gazebo/launch/YOUROBOT.launch` and add the following before the `</launch>` tag:

~~~
<!-- Spawn a robot into Gazebo -->
<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-file $(find baxter_description)/urdf/baxter.urdf -urdf -z 1 -model baxter" />
~~~

Launching this file, you should see the same results as when using `rosrun`.

#### XACRO Example with PR2
If your URDF is not in XML format but rather in [XACRO](http://ros.org/wiki/xacro) format, you can make a similar modification to your launch file. You can run this PR2 example by installing this package:

**ROS Groovy:** - Note: PR2 in Groovy is currently broken until this [pull request](https://github.com/PR2/pr2_common/pull/222) is merged and released to public debians

<pre>
sudo apt-get install ros-groovy-pr2-common
</pre>

**ROS Hydro:**
<pre>
sudo apt-get install ros-hydro-pr2-common
</pre>

Then adding this to your launch file created previously in this tutorial:

~~~
<!-- Convert an xacro and put on parameter server -->
<param name="robot_description" command="$(find xacro)/xacro.py $(find pr2_description)/robots/pr2.urdf.xacro" />

<!-- Spawn a robot into Gazebo -->
<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model pr2" />
~~~

Launching this file, you should see the PR2 in the gas station as pictured:

[[file:figs/PR2_GasStation.png|800px]]

Note: at this writing there are still a lot of errors and warnings from the console output that need to be fixed from the PR2's URDF due to Gazebo API changes.


----

### "Model Database" Robot Spawn Method

The second method of spawning robots into Gazebo allows you to include your robot within the `.world` file, which seems cleaner and more convenient but also requires you to add your robot to the Gazebo model database by setting an environment variable. This environment variable is required because of the separation of ROS dependencies from Gazebo; URDF package paths cannot be used directly inside `.world` files because Gazebo does not have a notion of ROS packages.

To accomplish this method, you must make a new model database that contains just your single robot. This isn't the cleanest way to load your URDF into Gazebo but accomplishes the goal of not having to keep two copies of your robot URDF on your computer. If the following instructions are confusing, refer back to the [Gazebo Model Database](http://gazebosim.org/user_guide/started__models__database.html) documentation to understand why these steps are required.

We will assume your ROS workspace file hierarchy is setup as described in the above sections. The only difference is that now a `model.config` file is addded to your `MYROBOT_description` package like so:

~~~
../catkin_ws/src
    /MYROBOT_description
        package.xml
        CMakeLists.txt
        model.config
        /urdf
            MYROBOT.urdf
        /meshes
            mesh1.dae
            mesh2.dae
            ...
        /materials
        /plugins
        /cad
~~~

This hierarchy is specially adapted for use as a Gazebo model database by means of the following folders/files:

* **/home/user/catkin_workspace/src** - this is treated as the location of a Gazebo Model Database
* **/MYROBOT_description** - this directory is treated as a single Gazebo model folder
* **model.config** - this is a required configuration file for Gazebo to find this model in its database
* **MYROBOT.urdf** - this is your robot description file, also used by Rviz, MoveIt!, etc
* **/meshes** - put your .stl or .dae files in here, just as you would with regular URDFs

#### model.config

Each model must have a model.config file in the model's root directory that contains meta information about the model. Basically copy this into a model.config file, replacing model.urdf with your file name:

      <?xml version="1.0"?>
      <model>
        <name>MYROBOT</name>
        <version>1.0</version>
        <sdf>urdf/MYROBOT.urdf</sdf>
        <author>
          <name>My name</name>
          <email>name@email.address</email>
        </author>
        <description>
          A description of the model
        </description>
      </model>

Unlike for SDFs, no version is required for the <sdf> tag when it is used for URDFs. See the Gazebo Model Database documentation for more info.

#### Environment Variable

Finally, you need to add an environment variable to your .bashrc file that tells Gazebo where to look for model databases.
Using the editor of your choice edit "~/.bashrc".
Check if you already have a `GAZEBO_MODEL_PATH` defined.
If you already have one, append to it using a semi-colon, otherwise add the new export.
Assuming your Catkin workspace is in `~/catkin_ws/` Your path should look something like:

      export GAZEBO_MODEL_PATH=/home/user/catkin_ws/src/

#### Viewing In Gazebo - Manually

Now test to see if your new Gazebo Model Database is properly configured by launching Gazebo:

      gazebo

And clicking the "Insert" tab on the left. You will probably see several different drop down lists that represent different model databases available on your system, including the online database. Find the database corresponding to your robot, open the sub menu, click on the name of your robot and then choose a location within Gazebo to place the robot, using your mouse.

#### Viewing In Gazebo - `roslaunch` with the Model Database

The advantage of the model database method is that now you can include your robot directly within your world files, without using a ROS package path. We'll use the same setup from the section "Creating a world file" but modify the world file:

* Within the same `MYROBOT_description/launch` folder, edit the MYROBOT.world file with the following contents:

~~~
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://gas_station</uri>
      <name>gas_station</name>
      <pose>-2.0 7.0 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://MYROBOT</uri>
    </include>
  </world>
</sdf>
~~~

* You should now be able to launch your custom world with both the gas station and robot into Gazebo using the following command:
<pre>
roslaunch MYROBOT_gazebo MYROBOT.launch
</pre>

The disadvantage of this method is that your packaged `MYROBOT_description` and `MYROBOT_gazebo`
are not as easily portable between computers - you first have to set the `GAZEBO_MODEL_PATH`
on any new system before being able to use these ROS packages.

* Exporting model paths from a pcakage.xml

The useful info would be the format for exporting model paths from a package.xml:

~~~
<export>
  <gazebo_ros gazebo_model_path="${prefix}/models"/>
  <gazebo_ros gazebo_media_path="${prefix}/models"/>
</export>
~~~
  
The '${prefix}` is something that new users might not immediately know about either, and necessary here.

Also would be useful to have some info on how to debug these paths from the ROS side, 
e.g. that you can use `rospack plugins --attrib="gazebo_media_path" gazebo_ros`
To check the media path that will be picked up by gazebo.

## Next Steps

Now that you know how to create `roslaunch` files that open Gazebo, world files and URDF models, you are now ready to create your own Gazebo-ready URDF model in the tutorial [Using A URDF In Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf)
