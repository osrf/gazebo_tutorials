# Prerequisites:

 * [Model Manipulation](http://gazebosim.org/tutorials/?tut=plugins_model)
 * [Plugin Tutorial](http://gazebosim.org/tutorials/?tut=plugins_hello_world)

# Code:

Source code: [gazebo/examples/plugins/factory](https://bitbucket.org/osrf/gazebo/src/gazebo7/examples/plugins/factory)

It can be useful to control what models exist in a running simulation, and when they should be inserted. This tutorial demonstrates how to insert predefined and custom models into Gazebo.

Use the `gazebo_plugin_tutorial` from the previous plugin tutorials

~~~~
$ mkdir ~/gazebo_plugin_tutorial
$ cd ~/gazebo_plugin_tutorial
~~~~

Create a new source file:

~~~
$ gedit factory.cc
~~~

Copy the following code into the `factory.cc` file:
<include from="/#include/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/factory/factory.cc' />

## The Code Explained

The first part of the code creates a world plugin.

<include from="/#include/" to="/_sdf\*\/\)/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/factory/factory.cc' />

Within the `Load` function are three different methods for model insertion.


The first method uses a World method to load a model based on a file in the resource path defined by the `GAZEBO_MODEL_PATH` environment variable.

<include from="/    Option 1:/" to="/\/\/box"\);/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/factory/factory.cc' />

The second method uses a World method to load a model based on string data.

<include from="/    Option 2:/" to="/sphereSDF\);/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/factory/factory.cc' />

The third method uses the message passing mechanism to insert a model. This method is most useful for stand alone applications that communicate with Gazebo over a network connection.

<include from="!//    Option 3:!" to="/factoryPub.*Publish\(msg\);/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/factory/factory.cc' />


## Compile

Assuming the reader has gone through the [Plugin Overview Tutorial](http://gazebosim.org/tutorials/?tut=plugins_hello_world), all that needs to be done in addition is save the above code as `~/gazebo_plugin_tutorial/factory.cc` and add the following lines to `~/gazebo_plugin_tutorial/CMakeLists.txt`

<include from="/add_library/" src='http://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/factory/CMakeLists.txt' />

Compiling this code will result in a shared library, `~/gazebo_plugin_tutorial/build/libfactory.so`, that can be inserted in a Gazebo simulation.

~~~
$ mkdir ~/gazebo_plugin_tutorial/build
$ cd ~/gazebo_plugin_tutorial/build
$ cmake ../
$ make
~~~

# Make the shapes

Make a models directory with a box and a cylinder inside

~~~
$ mkdir ~/gazebo_plugin_tutorial/models
$ cd ~/gazebo_plugin_tutorial/models
$ mkdir box cylinder
~~~

Create a box model

~~~
$ cd box
$ gedit model.sdf
~~~

Copy and paste the following into box/model.sdf

~~~
<?xml version='1.0'?>
<sdf version ='1.6'>
  <model name ='box'>
    <pose>1 2 0 0 0 0</pose>
    <link name ='link'>
      <pose>0 0 .5 0 0 0</pose>
      <collision name ='collision'>
        <geometry>
          <box><size>1 1 1</size></box>
        </geometry>
      </collision>
      <visual name ='visual'>
        <geometry>
          <box><size>1 1 1</size></box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
~~~

Create a `model.config` file

~~~
$ gedit model.config
~~~

Copy the following into `model.config`

~~~
<?xml version='1.0'?>

<model>
  <name>box</name>
  <version>1.0</version>
  <sdf >model.sdf</sdf>

  <author>
    <name>me</name>
    <email>somebody@somewhere.com</email>
  </author>

  <description>
    A simple Box.
  </description>
</model>
~~~

Navigate to the cylinder directory, and create a new `model.sdf` file

~~~
$ cd ~/gazebo_plugin_tutorial/models/cylinder
$ gedit model.sdf
~~~

Copy the following into `model.sdf`

~~~
<?xml version='1.0'?>
<sdf version ='1.4'>
  <model name ='cylinder'>
    <pose>1 2 0 0 0 0</pose>
    <link name ='link'>
      <pose>0 0 .5 0 0 0</pose>
      <collision name ='collision'>
        <geometry>
          <cylinder><radius>0.5</radius><length>1</length></cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder><radius>0.5</radius><length>1</length></cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
~~~

Create a model.config file

~~~
$ gedit model.config
~~~

Copy the following into model.config

~~~
<?xml version='1.0'?>

<model>
  <name>cylinder</name>
  <version>1.0</version>
  <sdf>model.sdf</sdf>

  <author>
    <name>me</name>
    <email>somebody@somewhere.com</email>
  </author>

  <description>
    A simple cylinder.
  </description>
</model>
~~~


## Run the code

Make sure your `$GAZEBO_MODEL_PATH` refers to your new models directory

~~~
$ export GAZEBO_MODEL_PATH=$HOME/gazebo_plugin_tutorial/models:$GAZEBO_MODEL_PATH
~~~

Add your library path to the `GAZEBO_PLUGIN_PATH`:

~~~
$ export GAZEBO_PLUGIN_PATH=$HOME/gazebo_plugin_tutorial/build:$GAZEBO_PLUGIN_PATH
~~~

Create a world SDF file called `~/gazebo_plugin_tutorial/factory.world`

~~~
$ cd ~/gazebo_plugin_tutorial
$ gedit factory.world
~~~

Copy the following into the world
<include src='http://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/factory/factory.world' />

Run Gazebo

~~~
$ gazebo ~/gazebo_plugin_tutorial/factory.world
~~~

The Gazebo window should show an environment with a sphere, box, and cylinder arranged in a row.
