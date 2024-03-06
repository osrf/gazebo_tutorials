# Overview

**Prerequisites:** [Make a mobile robot](/tutorials/?tut=build_robot)

Meshes can add realism to a model both visually and for sensors.  This tutorial demonstrates how the user can use custom meshes to define how their model will appear in simulation.

##Attach a Mesh as Visual##


The most common use case for a mesh is to create a realistic looking visual.

1.  Navigate to the `my_robot` directory

        cd ~/.gazebo/models/my_robot

1.  Open the `model.sdf` file using your favorite editor

        gedit ~/.gazebo/models/my_robot/model.sdf

1.  We'll add a mesh to the chassis visual. Find the visual with `name=visual`, which looks like:

    ~~~
        <visual name='visual'>
          <geometry>
            <box>
              <size>.4 .2 .1</size>
            </box>
          </geometry>
        </visual>
    ~~~

1.  A mesh can come a file on disk, or from another model. In this example we'll use a mesh from the pioneer2dx model. Change the visual element to the following (but keep the the rest of the file intact):

    ~~~
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://pioneer2dx/meshes/chassis.dae</uri>
            </mesh>
          </geometry>
        </visual>
    ~~~

1.  Look in your locally cached model database to see if you have the `pioneer2dx` model referenced by above `<mesh>` block:

        ls -l ~/.gazebo/models/pioneer2dx/meshes/chassis.dae

    If the mesh file does not exist, make Gazebo pull the model from the [Model Database](https://github.com/osrf/gazebo_models) by spawning the `Pioneer 2DX` model at least once in gazebo (under `Insert->http://gazebosim.org/models`).

    Or manually download the model files to your local cache:

        cd ~/.gazebo/models
        wget -q -R *index.html*,*.tar.gz --no-parent -r -x -nH http://models.gazebosim.org/pioneer2dx/

1.  In Gazebo, drag the `My Robot` model in the world. The visual for the chassis will look like a pioneer2dx.

    [[file:files/Mobile_robot_chassis_1.png|640px]]

1.  The chassis is obviously too big for our robot, so we need to scale the visual.

1.  Modify the visual to have a scaling factor.

    %%%
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://pioneer2dx/meshes/chassis.dae</uri>
    %%%

    ~~~
              <scale>0.9 0.5 0.5</scale>
    ~~~

    %%%
            </mesh>
          </geometry>
        </visual>
    %%%

    [[file:files/Mobile_robot_chassis_2.png|640px]]

1.  The visual is also a little too low (along the z-axis). Let's raise it up a little by specifying a pose for the visual:

    %%%
        <visual name='visual'>
    %%%

    ~~~
          <pose>0 0 0.05 0 0 0</pose>
    ~~~

    %%%
          <geometry>
            <mesh>
              <uri>model://pioneer2dx/meshes/chassis.dae</uri>
              <scale>0.9 0.5 0.5</scale>
            </mesh>
          </geometry>
        </visual>
    %%%

    [[file:files/Mobile_robot_chassis_3.png|640px]]

    Note that at this point we have simply modified the `<visual>` elements of the robot, so the robot will look like a scaled down version of the Pioneer 2DX model through the GUI and to GPU based sensors such as camera, depth camera and GPU Lasers.  Since we did not modify the `<collision>` elements in this model, the box geometry will still be used by the physics engine for collision dynamics and by CPU based ray sensors.

##Further Reading##

When creating a new robot, you'll likely want to use your own mesh file. The [import a mesh tutorial](/tutorials/?tut=import_mesh) describes how to go about importing a mesh into a format suitable for Gazebo.

## Try for yourself ##

1.  Find and download a new mesh on [3D Warehouse](https://3dwarehouse.sketchup.com/). Make sure the mesh is in the Collada (.dae) format.

1.  Put the mesh in the `~/.gazebo/models/my_robot/meshes`, creating the `meshes` subdirectory if necessary

1.  Use your new mesh on the robot, either as a replacement for the chassis, or as an additional `<visual>`.

Note: Materials (texture files such with extension like .png or .jpg), should be placed in `~/.gazebo/models/my_robot/materials/textures`.

##Next##

[Next: Add a Sensor to a Robot](/tutorials/?tut=add_laser)
