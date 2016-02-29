# Re-cap

We have created an SDF model of the Velodyne HDL-32 LiDAR that has visual
meshes and generates data with a noise model.

In this section, we will modify the model structure so that it is a stand-alone
model that can be shared and reused.

# Create the model structure

Gazebo has defined a model directory structure that supports stand-alone
models, and the ability to share models via an online model database. Review
[this tutorial](http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot) for more information.

1. Create a new directory to hold the Velodyne model. We will place the
   directory in `~/.gazebo/models`, since Gazebo knows to look there for
   models and this will speed the developement process.

    ```
    mkdir ~/.gazebo/models/velodyne_hdl32
    ```

1. Create the `model.config` file. Each model requires some meta information
   that describes the model, the author, and any dependencies.

    ```
    gedit ~/.gazebo/models/velodyne_hdl32/model.config
    ```

1. Copy the following into the `model.config` file.

    ```
    <?xml version="1.0"?>
    
    <model>
      <name>Velodyne HDL-32</name>
      <version>1.0</version>
      <sdf version="1.5">model.sdf</sdf>
    
      <author>
        <name>Optional: YOUR NAME</name>
        <email>Optional: YOUR EMAIL</email>
      </author>
    
      <description>
        A model of a Velodyne HDL-32 LiDAR sensor.
      </description>
    
    </model>
    ```

1. Notice that the `model.config` file references a `model.sdf` file. This
   `model.sdf` file will contain the description of the Velodyne laser.

1. Create the `model.sdf` file.

    ```
    gedit ~/.gazebo/models/velodyne_hdl32/model.sdf
    ```

1. Copy the Velodyne description from the `velodyne.world` file,
   created in the previous tutorial, or copy the model from below.

    ```
    <?xml version="1.0" ?>
    <sdf version="1.5">
      <model name="velodyne">
        <link name="base">
          <pose>0 0 0.029335 0 0 0</pose>
          <inertial>
            <mass>1.2</mass>
            <inertia>
              <ixx>0.001087473</ixx>
              <iyy>0.001087473</iyy>
              <izz>0.001092437</izz>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyz>0</iyz>
            </inertia>
          </inertial>
          <collision name="base_collision">
            <geometry>
              <cylinder>
                <radius>.04267</radius>
                <length>.05867</length>
              </cylinder>
            </geometry>
          </collision>
          <visual name="base_visual">
            <pose>0 0 -0.029335 0 0 0</pose>
            <geometry>
              <mesh>
                <uri>model://velodyne_hdl32/meshes/velodyne_base.dae</uri>
              </mesh>
            </geometry>
          </visual>
        </link>
        
        <link name="top">
          <pose>0 0 0.095455 0 0 0</pose>
          <inertial>
            <mass>0.1</mass>
            <inertia>
              <ixx>0.000090623</ixx>
              <iyy>0.000090623</iyy>
              <izz>0.000091036</izz>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyz>0</iyz>
            </inertia>
          </inertial>
          <collision name="top_collision">
            <geometry>
              <cylinder>
                <radius>0.04267</radius>
                <length>0.07357</length>
              </cylinder>
            </geometry>
          </collision>
          <visual name="top_visual">
            <pose>0 0 -0.0376785 0 0 1.5707</pose>
            <geometry>
              <mesh>
                <uri>model://velodyne_hdl32/meshes/velodyne_top.dae</uri>
              </mesh>
            </geometry>
          </visual>
    
          <sensor type="ray" name="sensor">
            <pose>0 0 -0.004645 1.5707 0 0</pose>
            <visualize>true</visualize>
            <update_rate>30</update_rate>
            <ray>
              <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.1</stddev>
              </noise>
    
              <scan>
                <horizontal>
                  <samples>32</samples>
                  <resolution>1</resolution>
                  <min_angle>-0.53529248</min_angle>
                  <max_angle>0.18622663</max_angle>
                </horizontal>
              </scan>
    
              <range>
                <min>0.1</min>
                <max>70</max>
                <resolution>0.02</resolution>
              </range>
            </ray>
          </sensor>
        </link>
    
        <joint type="revolute" name="joint">
          <pose>0 0 -0.036785 0 0 0</pose>
          <parent>base</parent>
          <child>top</child>
          <axis>
            <xyz>0 0 1</xyz>
            <limit>
              <lower>-10000000000000000</lower>
              <upper>10000000000000000</upper>
            </limit>
          </axis>
        </joint>
    
      </model>
    </sdf>
    ```

1. At this point, we should be able to start Gazebo, and dynamically insert
   the Velodyne model.

    1. ```gazebo```

    1. Select the `Insert` tab on the left, and scroll down to find the
       `Velodyne HDL-32` entry.

    1. Click on the `Velodyne HDL-32` and then left-click in the render window
       to spawn the model.

    [[file:files/velodyne_insertion.png|800px]]

# Contribute the model to the online-database

Contributing our model to Gazebo's online-databse benefits you and every
other user of Gazebo. When the model is hosted on the database, Gazebo will
automatically download it when requested. This means you don't have to
manage which computers have the model and which do not. Additionally, other
people can use your model.

1. Fork the `gazebo_models` database by visiting [https://bitbucket.org/osrf/gazebo_models/fork](https://bitbucket.org/osrf/gazebo_models/fork).

1. Clone your fork of model database.

    ```
    cd
    hg clone URL_OF_YOUR_FORK 
    ```

1. Look at the directories in your cloned repository to make sure your model
   does not already exist.

1. Copy the model from `~/.gazebo/models` to the cloned repository.

    ```
    cp -r ~/.gazebo/models/velodyne-hdl32 ~/gazebo_models
    ```

1. Make a new branch, which will make the pull-request process a bit easier.

    ```
    cd ~/gazebo_models
    hg branch velodyne_tutorial_do_not_merge
    ```

1. Add, commit, and push your model.

    1. ```hg add velodyne*```

    1. ```hg commit -m "Added a Velodyne HDL-32 LiDAR"```

    1. ```hg push --new-branch```

1. Create a pull-request back to the main `gazebo_models` repository.

    1. Open the `URL_OF_YOUR_FORK` in a webbrowser

    1. Select `Create pull request` on the left

    1. Enter a title and description, and select the `Create Pull Request`
       button.

1. Two approvals of your pull request are required before it will be merged
   into the main `gazebo_models` repository. Please respond to any comments
   quickly in order to expidite the process.

# Next Up

The next tutorial in this series will add a plugin to the Velodyne sensor.
This plugin will control the rotation of the sensor's upper portion.

[Control plugin](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5)
