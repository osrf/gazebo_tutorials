# Overview

This page explains how to install the GazeboJs Node bindings to Gazebo.



### Ubuntu Linux

This tutorial shows how to download install and compile Gazebojs on a computer where Gazebo 4 and its  development libraries are installed.

#### setup

install the osrf repository and install libgazebo4-dev (http://gazebosim.org/tutorials?tut=install)

install nodejs and npm 

    sudo pat-get install nodejs nodejs-legacy npm

install jansson 

    sudo apt-get install libjansson-dev

Make sure that these packages can be found in your pkg-config path. You can check this by executing this command without any error:

    pkg-config --cflags gazebo jansson protobuf


#### Running

Now that everything is installed, here are the steps to test it:


Create a NodeJs project directory
 
    mkdir gz_node_inst
    cd gz_node_inst


Install Gazebojs

     npm install gazebojs

This operation should download and compile the latest gazebojs. There is a C++ compilation phase where a NodeJs module is created. There should now be a node_modules directory created in gz_node_inst.



Test your installation:

Launch Gazebo in a separate terminal and verify that the simulation is running (Sim Time increases):

    gazebo


Use the 'node' command to invoke the NodeJs REPL console

    node

Type in the following command to load the gazeboJs module, create a simulation client and pause the running simulation

    var gazebojs = require('gazebojs')
    var sim = new gazbojs.Gazebo()
    sim.pause()

You should see the simulation stop in Gazebo.

