# Overview

*Gzserver* allows you to load an environment, insert your models, and
simulate the world. As you probably know, a robot simulator can be an
invaluable tool for testing and tuning in advance the code or behaviour that you
will run on a real robot. You might want to run multiple simulation
episodes to see how different parameters or approaches behave. Cloning a
simulation is useful for running different versions of your code in
parallel. This tutorial will guide you through the steps required to clone your
current simulation.

# Clone the simulation using the GUI

Start Gazebo by typing the following command at the command prompt:

~~~
gazebo
~~~

* Insert a simple sphere into the scene by using the upper toolbar.

* Clone your current simulation by clicking on **File-> Clone world**.

You should see a dialog window similar to the window below.

[[file:files/gazebo_clone_sim.png|640px]]

The new cloned world will run on a separate server that will have its own
*Master*. This dialog window allows you to specify the port in which the new
Master will accept connections from the clients. Note that you should select
a free port from the range 1025-65535. Our recommendation is to start with 11346
and keep incrementing this number for every concurrent server that you may have.

* Set the port for your new server and click *Okay*.

At this moment your new server should be running with an exact copy of your
current world. If you look in your server log file, you should see a message
confirming that the world has been cloned.

~~~
cat ~/.gazebo/gzserver.log
~~~

%%%
Gazebo multi-robot simulator, version 4.0.0
Copyright (C) 2012-2014 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org


(1409088199 32370140) Cloning world [default]. Contact the server by typing:
  GAZEBO_MASTER_URI=http://localhost:11346 gzclient
%%%

Your cloned server will store its log files in a directory named `~/.gazebo/server-<MASTER_PORT>`.
E.g.: In our example, the log files for the cloned gzserver will be located under `/.gazebo/server-11346`.

Open a new terminal and connect *gzclient* to your new server. If you did not use port 11346, be sure to replace "11346" with your port number:

~~~
GAZEBO_MASTER_URI=http://localhost:11346 gzclient
~~~

The above code modifies the environment variable `GAZEBO_MASTER_URI` to
point to the cloned server. Otherwise, you would be visualizing your original
world.

Although your two gzclients are visualizing similar worlds, the simulations are
running on different servers. Verify this by changing the gravity in one of the
simulations to `0.003` (under the *world tab*, click on physics, and then
change the z value of the property *gravity*). You should only see one of your
spheres slowly flying. This proves that after cloning a simulation, each world
is independent.

[[file:files/sidebyside.png|640px]]

Once the new server is cloned, it's totally detached from the orginal one, so
you will need to kill it manually:

~~~
killall gzserver
~~~

Note that the cloned server will be running on the same machine on which the original
server is located. Take this into account before cloning a simulation because
you may need SSH access to the server machine (if your server is running
remotely) in order to kill the new server after a cloning.

# Clone the simulation programmatically

It's also possible to clone a simulation programmatically using the Gazebo
transport system. As an example, we'll describe the example code shipped with
Gazebo in the folder `examples/stand_alone/clone_simulation`.

Create a new directory named `clone_simulation` for this tutorial:

~~~
mkdir ~/clone_simulation
cd ~/clone_simulation
~~~

Download the files [`CMakeLists.txt`](http://bitbucket.org/osrf/gazebo/src/default/examples/stand_alone/clone_simulation/CMakeLists.txt) and [`cloner.cc`](http://bitbucket.org/osrf/gazebo/src/default/examples/stand_alone/clone_simulation/cloner.cc) into the previous folder.

~~~
wget http://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/clone_simulation/CMakeLists.txt
wget http://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/clone_simulation/cloner.cc
~~~

Compile the example:

~~~
mkdir build
cd build
cmake ..
make
~~~

Run the example:

~~~
./cloner
~~~

%%%
./cloner

Press [ENTER] to clone the current simulation
%%%

The example will show a message telling you that the new server is running and that you should press `ENTER` to clone the current simulation. Before hitting
`ENTER`, connect a gzclient to the current server, by typing in a new terminal:

~~~
gzclient
~~~

Spawn a sphere using the upper toolbar.

Go back to the terminal where your `cloner` is running and press `ENTER` to trigger
the simulation cloning.

%%%
Press [ENTER] to clone the current simulation


World cloned. You can connect a client by tiping
  GAZEBO_MASTER_URI=http://localhost:11346 gzclient

Press [ENTER] to exit and kill all the servers.
%%%

You should see a confirmation message with the location of the new server and
instructions on how to connect a new gzclient to it. Open a new terminal and run gzclient:

~~~
GAZEBO_MASTER_URI=http://localhost:11346 gzclient
~~~

Verify again that the two simulations are independent by changing the gravity in
one of the simulations to `0.003` (as noted above). As before, you should only see one of your
spheres moving away.

[[file:files/sidebyside.png|640px]]

## The source code

Let's take a look at the source code for our example:

<include src='http://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/clone_simulation/cloner.cc' />

## The code explained

<include from='/int main/' to='/RunServer\);/' src='http://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/clone_simulation/cloner.cc' />

This fragment of the code spawns a new thread and executes a new server.

<include from='/  gazebo::transport::NodePtr/' to='/  getchar();/' src='http://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/clone_simulation/cloner.cc' />

The simulation cloning is performed via the transport system. First, we have to
initialize a transport node that will allow us to use the transport. We need a
topic publisher to send a new message with our cloning request. The topic is
`/gazebo/server/control`. In addition, we need a subscriber on the topic `/gazebo/world/modify` for receiving the result of our
clone request.

<include from='/  gazebo::msgs::ServerControl/' to='/  getchar();/' src='http://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/clone_simulation/cloner.cc' />

This is the part of the code where we prepare our `ServerControl` message for
our cloning request. The field `save_world_name` specifies the name of the world that
we want to clone. The empty string represents the *default* world. The field
`clone` is set to `true` when requesting a clone. The field
`new_port` sets the port that the new server will use for connections. This will
be the port that we will use to connect our future gzclient to display the new
simulation. Finally, the message is sent by calling the `Publish()` method with
our custom message.

<include from='/void OnWorldModify/' to='/  }\n}/' src='http://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/clone_simulation/cloner.cc' />

When the server processes our clone request, it sends us a response contained
in a `WorldModify` message. This is the callback that we registered during the
subscription and it will be triggered when the response from the server is
received. The field `cloned` will be true when a new server has been cloned.
Also, the field `cloned_uri` will show us the *URI* of the new server.

<include from='/  //Make sure/' to='/gazebo::shutdown/' src='http://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/clone_simulation/cloner.cc' />

These commands will terminate all the servers running in our system.