# Overview

This page explains how to install the GazeboJs Node bindings to Gazebo.



### Ubuntu Linux

This tutorial shows how to download install and compile Gazebojs on a computer where Gazebo 4 and its  development libraries are installed.

#### setup

install nodejs and npm (sudo pat-get install nodejs nodejs-legacy npm)
install jansson (sudo apt-get install libjansson-dev)
install the osrf repository and install libgazebo4-dev (http://gazebosim.org/tutorials?tut=install)



#### Running

Now that everything is installed, here's how to run it:


1. Create a NodeJs project
        Step 1

1. The new interface needs `ulimit` set:

    ~~~
    ulimit -s unlimited
    ~~~

1. Launch drcsim as usual

    ~~~
    roslaunch drcmsim_gazebo atlas_v3.launch
    ~~~
