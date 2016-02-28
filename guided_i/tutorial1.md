# Intro

Welcome to the Intermediate Module! This module will guide you through the
process of creating a new simulation feature and contributing the feature to
Gazebo.

We assume you are familiar with using Gazebo and Linux. We also assume that
you are an expert tutorials reader (read everything carefully, and
completely).

During this series of tutorials we will create a 
[Velodyne HDL-32 LiDAR](http://velodynelidar.com/hdl-32e.html). We will
walk through 

1. creating an SDF model of the HDL-32 sensor,
1. contributing the model to Gazebo's model database, 
1. improving the model's appearance and data output,
1. controlling the model using a plugin, and
1. visualizing the sensor data in Gazebo and RViz.

Each of the above topics will be covered in separate tutorials. The rest of
this tutorial will focus on the creation of the Velodyne SDF model. 

# Model Creation

The fist step when creating a new model is to collect information about the
model. In this case, the [Velodyne LiDAR company](http://velodynelidar.com/) has provided documentation about their sensors on their website
http://velodynelidar.com/hdl-32e.html. If detailed information about a model
is not available, then you can measure a physical version, ask the
manufacturer for specifications, or in the worst case guess.

Based on the Velodyne documentation, we will create a sensor that has:

1. a base cylinder and top cylinder, where the top cylinder spins, and
1. a set of laser rays oriented in the vertical fan.

## Step 1: Create a basic SDF model

1. Create a new world file.

    ```
    cd
    gedit velodyne.world
    ```

1. Populate the world file with a ground plane and light:

    ```
    <?xml version="1.0" ?>
    <sdf version="1.5">
      <world name="default">
    
        <!-- A global light source -->
        <include>
          <uri>model://sun</uri>
        </include>
    
        <!-- A ground plane -->
        <include> 
          <uri>model://ground_plane</uri>
        </include>
      </world>
    </sdf>
    ```

1. Next, we will add the basics of the Velodyne LiDAR to the SDF world file.
   We will Use the Velodyne sensor dimensions to construct a base cylinder and
   a top cylinder. Below is an screenshot of the [Velodyne 2D
   drawing](http://velodynelidar.com/lidar/hdldownloads/86-0106%20REV%20A%20OUTLINE%20DRAWING%20HDL-32E.pdf).

    [[file:files/velodyne_drawing.png|800px]]
  
    1. Copy the following into the SDF world file directly before the `</world>` tag.

        ```
        <model name="velodyne_hdl-32">
          <!-- Give the base link a unique name -->
          <link name="base">

            <!-- Offset the base by half the lenght of the cylinder -->
            <pose>0 0 0.029335 0 0 0</pose>
            <collision name="base_collision">
              <geometry>
                <cylinder>
                  <!-- Radius and length provided by Velodyne -->
                  <radius>.04267</radius>
                  <length>.05867</length>
                </cylinder>
              </geometry>
            </collision>

            <!-- The visual is mostly a copy of the collision -->
            <visual name="base_visual">
              <geometry>
                <cylinder>
                  <radius>.04267</radius>
                  <length>.05867</length>
                </cylinder>
              </geometry>
            </visual>
          </link>
        
          <!-- Give the base link a unique name -->
          <link name="top">

            <!-- Vertically offset the top cylinder by the length of the bottom
                cylinder and half the length of this cylinder. -->
            <pose>0 0 0.095455 0 0 0</pose>
            <collision name="top_collision">
              <geometry>
                <cylinder>
                  <!-- Radius and length provided by Velodyne -->
                  <radius>0.04267</radius>
                  <length>0.07357</length>
                </cylinder>
              </geometry>
            </collision>

            <!-- The visual is mostly a copy of the collision -->
            <visual name="top_visual">
              <geometry>
                <cylinder>
                  <radius>0.04267</radius>
                  <length>0.07357</length>
                </cylinder>
              </geometry>
            </visual>
          </link>
        </model>
        ```

1. When building a new model, it is a good idea to periodically try out small
changes. Start Gazebo paused so that you can
view the model without the physics engine altering the model's pose. There
are also a few other graphical tools that can assist the development process, which we will cover over the course of this tutorial.

1. Run the Velodyne world paused (the -u argument).

    ```
    cd
    gazebo velodyne.world -u
    ```

1. By default, you will see the `<visual>` elements in a model, which define
   how a model looks. The `<collision>` elements, on the other hand, define
   how the model will behave when colliding with other models. To see, and
   debug, the `<collision>` elements `Right-click` on a model, and choose
   `View->Collisions`. Try this now, and you should see two orange cylinders
   (that look like a single cylinder due to their proximity).

    [[file:files/velodyne_collisions.jpg|800px]]

## Step 2: Add inertia

At this point, we have a Velodyne model that lacks dynamic properties such
as moments of inertia. The physics engine uses inertia information to
calculate how a model will behave when forces act upon it. A model with
incorrect, or no, inertia values will behave in a strange manner.

1. Start by visualizing the current inertia values. With Gazebo running,
   right-click on the Velodyne and select ```View->Inertia```. This will
   causes two purple boxes to appear.

    [[file:files/velodyne_inertia.jpg|800px]]

    > Generally, each purple box should roughly match the size of the link it
is associated with. You'll notice that the current inertia boxes are grossly
oversized, which is due to our model lacking inertia information.

1. We can add inertia to a link by specifying both the mass and inertia
   matrix. We are basing the mass on the specified mass of the Velodyne,
   which is 1.3kg, and giving the base link a majority (this distribution is
   a guess on our part). The moment of inertia matrix can be computed
   using equations found on
   [Wikipedia](https://en.wikipedia.org/wiki/List_of_moments_of_inertia). 

    > The following are the inertia values for the `base` link. Copy the
    > `<inertial>` block into the indicated location.

    ````
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

    ````

    > and the inertia values for the `top` link. Copy the
    > `<inertial>` block into the indicated location.

    ````
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
    ````

1. With the inertia values in place, visualization should be similar to the
   following image.

    [[file:files/velodyne_inertia_good.jpg|800px]]

At this point in the model creation process, you should have a model that
has correct visual, collision, and inertia properties. We will now move onto
joints.

## Step 3: Add the joint

Joints define constraints between links. The most common
type of joint, in the robotic domain, is `revolute`. A revolute joint
defines a single rotational degree of freedom between two links. A complete
list of joints can be found on the [SDF
website](http://sdformat.org/spec?ver=1.6&elem=joint#joint_type).

As with most the previous sections, it is possible to visualize joints. With
Gazebo running, Right click on a model, and choose ```View->Joints```.
Joints are often located within a model, so you may have to make a model
transparent to see the joint visualization (Right click on the model and
select ```View->Transparent```).

1. We first have to add a joint to the Velodyne model. The joint will be
   revolute, since the top link will spin relative to the base link.
   
1. Open the SDF world, and add a ``revolute`` joint before the `</model>`
   tag.

    ```
    <!-- Each joint must have a unique name -->
    <joint type="revolute" name="joint">
    
      <!-- Position the joint at the bottom of the top link -->
      <pose>0 0 -0.036785 0 0 0</pose>
    
      <!-- Use the base link as the parent of the joint -->
      <parent>base</parent>
    
      <!-- Use the top link as the child of the joint -->
      <child>top</child>
    
      <!-- The axis defines the joint's degree of freedom -->
      <axis>
    
        <!-- Revolve around the z-axis -->
        <xyz>0 0 1</xyz>
    
        <!-- Limit refers to the range of motion of the joint -->
        <limit>
    
          <!-- Use a very large number to indicate a continuous revolution -->
          <lower>-10000000000000000</lower>
          <upper>10000000000000000</upper>
        </limit>
      </axis>
    </joint>
    ```

1. Run the SDF world, paused, and visualization the joint.

    1. ```gazebo velodyne.world -u```

    1. Right-click on the model and select `View->Joints`

    1. Right-click on the model and select `View->Transparent`


    [[file:files/velodyne_joints.jpg|800px]]

1. We can also verify that the joint rotates properly using the Joint
   Command graphical tool. Drag the right panel open on the main window, and
   select the Velodyne model.

    [[file:files/velodyne_joint_cmd_widget.png|800px]]

1. Use the `Force` tab in this widget to apply a small force, 0.001 will be fine, to the joint. You should see the visualized joint start to spin around the model's Z-axis. 


At this point we have a Velodyne model with good inertia, collision, and
joint properties. In the next section we'll cover the final part of model,
addition of the sensor. 

## Step 4: Add the sensor

A sensor is used to generate data, from the environment or a model. In this
section we'll add a `ray` sensor to the Velodyne model. A `ray` sensor in
Gazebo consists of one or more beams that generate distance, and potentially
intensity, data.

A `ray` sensor consists of one `<scan>` and one `<range>` SDF element. The `<scan>` element defines the layout and number of beams, and the `<range>` element defines properties of an individual beam. i

Within the `<scan>` elements is a `<horizontal>` and `<vertical>` element.
The `<horizontal>` component defines rays that fan out in a horizontal
plane, and the `<vertical>` component defines rays that fan out in
a vertical plane.

The Velodyne sensor requires vertical rays, that then rotate. We will simulated this as rotated horizontal fan. We're taking this approach because it will be a bit easier to visualize the data in Gazebo.  The Velodyne specification indicates that the HDL-32 has 32 rays with a vertical field of view between +10.67 and -30.67 degrees.

1. We will add the ray sensor to the top link. Copy the following into the
   `<link name="top">` element in the `velodyne.world` file.

    ```
    <!-- Add a ray sensor, and give it a name -->
    <sensor type="ray" name="sensor">
    
      <!-- Position the ray sensor based on the specification. Also rotate
           it by 90 degrees around the X-axis so that the <horizontal> rays
           become vertical -->
      <pose>0 0 -0.004645 1.5707 0 0</pose>
    
      <!-- Enable visualization to see the rays in the GUI -->
      <visualize>true</visualize>
    
    </sensor>
    ```

1. Next, we will add the `<ray>` element, which defines the `<scan>`
   and `<range>` elements. Place the following SDF within the `<sensor>`
   element.

    ```
    <ray>
    
      <!-- The scan element contains the horizontal and vertical beams.
           We are leaving out the vertical beams for this tutorial. -->
      <scan>
    
        <!-- The horizontal beams -->
        <horizontal>
          <!-- The velodyne has 32 beams(samples) -->
          <samples>32</samples>

          <!-- Resolution is multiplied by samples to determine number of
               simulated beams vs interpolated beams. See:
               http://sdformat.org/spec?ver=1.6&elem=sensor#horizontal_resolution
               -->
          <resolution>1</resolution>

          <!-- Minimum angle in radians -->
          <min_angle>-0.53529248</min_angle>
    
          <!-- Maximum angle in radians -->
          <max_angle>0.18622663</max_angle>
        </horizontal>
      </scan>
    
      <!-- Range defines characteristics of an individual beam -->
      <range>
        
        <!-- Minimum distance of the beam -->
        <min>0.05</min>
    
        <!-- Maximum distance of the beam -->
        <max>70</max>
    
        <!-- Linear resolution of the beam -->
        <resolution>0.02</resolution>
      </range>
    </ray>
    ```

1. Start up simulation again, and you should see the 32 sensor beams.

    [[file:files/velodyne_rays.jpg|800px]]

# Next Up

In the next section we will modify the Velodyne model so that it can be
easily shared and reused.

[Next Section](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i2)
