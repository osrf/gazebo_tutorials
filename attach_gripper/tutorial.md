# Overview

**Prerequisites:**

  [Make a Mobile Robot](http://gazebosim.org/tutorials/?tut=build_robot)

  [Make a Simple Gripper](http://gazebosim.org/tutorials/?tut=simple_gripper)

This tutorial explains how to create a composite robot from existing robot parts, i.e. mobile base, simple arm and simple gripper.

# Robot Components

Start up gazebo and make sure you can load the models from the two previous tutorials.

## Mobile Base

1. Per instructions in [Make a Mobile Robot](http://gazebosim.org/tutorials/?tut=build_robot) tutorial, you should have a mobile base robot at your disposal:

    [[file:files/Mobile_base.png|640px]]

1. For this exercise, modify `~/.gazebo/models/my_robot/model.sdf` to make the model larger so it can accommodate the gripper we are about to append to it:

        gedit ~/.gazebo/models/my_robot/model.sdf

    update the contents to make the model body larger and re-position the wheels accordingly:

    <include src='https://bitbucket.org/osrf/gazebo_tutorials/raw/attach_gripper/files/model.sdf' />

    [[file:files/files/Mobile_base_large.png|640px]]

## Assembling a Composite Robot

1. To create a mobile robot with a simple gripper attached, create a new models directory

        mkdir ~/.gazebo/models/simple_mobile_manipulator

    And edit the model config file:

        gedit ~/.gazebo/models/simple_mobile_manipulator/model.config

    populate it with the following contents:

    <include src='https://bitbucket.org/osrf/gazebo_tutorials/raw/attach_gripper/files/model.config' />

1. Next, create the model SDF file:

        gedit ~/.gazebo/models/simple_mobile_manipulator/manipulator.sdf

    and populate with following contents:

    <include src='https://bitbucket.org/osrf/gazebo_tutorials/raw/attach_gripper/files/manipulator.sdf' />

1. Make sure the `model.config` and `manipulator.sdf` files above are saved, start Gazebo and spawn the model above by using the **insert** tab and choosing **Simple Mobile Manipulator** model.  You should see something similar to:

    [[file:files/Simple_mobile_manipulator.png|640px]]
