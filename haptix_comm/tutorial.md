# Overview

This tutorial will explain how to use the HAPTIX-Comm C-API for sending new
joint commands to the hand and receiving state updates.

# Simulation setup

We assume that you have already done the installation step.

# Compile your controller.

In this tutorial we include a very basic controller that applies a sinusoidal
wave to all the hand joints. First, you should compile your controller and link
it against the haptix_comm library.

1. Create a new directory named `haptix_controller` for this tutorial:

    ~~~
    mkdir ~/haptix_controller
    cd ~/haptix_controller
    ~~~

1. Download the source code of the controller and the cmake file:

    ~~~
    wget http://bitbucket.org/osrf/TODO
    wget http://bitbucket.org/osrf/TODO
    ~~~

1. Create a build directory and compile the source code.

    ~~~
    mkdir build
    cd build
    cmake ..
    make
    ~~~

Now, we are ready to test our controller with the HAPTIX simulator.

# Running the simulation with your controller

1. In a terminal, start the HAPTIX simulation:

    ~~~
    roslaunch handsim_gazebo handsim.launch
    ~~~

1. In a different terminal, go to the build directory where you have your
controller executable:

    ~~~
    cd ~/haptix_controller/build
    ~~~

1. Start the controller:

    ~~~
    ./hx_controller
    ~~~

You should see your fingers moving following a smooth trajectory in an infinite
loop.

# The code explained

<include from='/int main/' to='/return -1;/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/haptix_comm/files/hx_controller.c' />
