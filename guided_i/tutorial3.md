# Improve Appearance

Models with textures and 3D meshes can improve your visual experience, and
more importantly improve the realism of an environment. Simulated cameras
that feed information to vision processing algorithms will benefit from
models that appear realistic as well.

In this section, we will use 3D meshes available on the Velodyne website to
improve the visual appearance of our model. More manufactures are making 3D
meshes available, however it can sometimes be difficult to find an existing
mesh.  In these cases you can try your hand at mesh creation, work with an
artist, or contact the manufacturer directly.

Velodyne has a [STEP file](http://velodynelidar.com/docs/drawings/HDL32E_Outline_Model.STEP) for the HDL-32 located on [their
website](http://velodynelidar.com/docs/drawings/HDL32E_Outline_Model.STEP).
Gazebo can only use STL or Collada files, so we'll have to convert this
file and then add it to our model.

# Mesh Acquisition

1. Download the STEP file. Right-click and save-as on this [link](http://velodynelidar.com/docs/drawings/HDL32E_Outline_Model.STEP) to save the STEP file.

1. Open the STEP file in FreeCad. If you're using Ubuntu, you can install freecad using: ```sudo apt-get install freecad```

    ```
    freecad ~/Downloads/HDL32E_Outline_Model.STEP
    ```

1. Select the base of Velodyne, by clicking on "HDL32 OUTLINE MODEL" in the
   left-hand `Labels & Attributes` panel.

    [[file:files/freecad_base.png|800px]]

1. Export to Collada, into a file called `velodyne_base.dae`.

    ```
    File->Export
    ```

1. We need to modify the `velodyne_base.dae` file in
   [Blender](https://www.blender.org/),
   because the units are incorrect and we want the mesh centered on the
   origin.

    ```
    blender
    ```

1. Import the `velodyne_base.dae` file.

    ```
    File->Import->Collada
    ```

    > Note: You may have to download a newer version of [Blender](https://www.blender.org/) to get the Collada import feature.

    [[file:files/blender_import.png|800px]]

1. The units are currently in millimeters, and Gazebo requires meters. The
   model is also rotated so that the top is facing along the Y-axis, and we
   would like the top to face along the Z-axis.

1. Pull out the right-tab in blender (look for a plus
   sign near the upper right of the render window). Under the `Dimensions`
   section of this tab, divide the x,y,z components by 1000. See image
   below.

1. In the same tab, rotate the model by 90 degrees around the X-axis. See
   image below.

1. The mesh should not look like the following image.

    [[file:files/blender_shrunk.png|800px]]

1. Export the mesh as a collada file.

    ```
    File->Export->Collada
    ```

1. Repeat this process for the top of the velodyne. You will also have to
   translate this mesh so that the bottom is on the XY-plane. Use the
   `Translate` button in the upper left (click on it twice to open a dialog in the lower left)  to move the model down the Z-axis by -0.06096. 

    [[file:files/blender_translate.png|800px]]
   
At this point You should have two collada files: `velodyne_base.dae` and `velodyne_top.dae`. These two files are also available from the following links:


1.  [velodyne_base.dae](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/guided_i/files/velodyne_base.dae)

1.  [velodyne_top.dae](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/guided_i/files/velodyne_top.dae)
  
In the next section, we will cover adding these meshes to the SDF model. 

# Add meshes to SDF

Another benefit of Gazebo's model structure is that it conveniently
organizes resources, such as mesh files, required by the model. In this
section, we will add the two mesh files, `velodyne_base.dae` and
`velodyne_top.dae`, to the Velodyne SDF model file.


1. Create a `meshes` directory in the `~/.gazebo/models/velodyne_hdl32`
   directory.

    ```
    mkdir ~/.gazebo/models/velodyne_hdl32/meshes
    ```

1. Copy your two Collada files into the new directory.

    ```
    cp velodyne_base.dae ~/.gazebo/models/velodyne_hdl32/meshes
    cp velodyne_top.dae ~/.gazebo/models/velodyne_hdl32/meshes
    ```

1. Now we will modify the model's SDF to use the `velodyne_top` mesh.

    1. Within the `<visual name="top_visual">` element, replace the
       `<cylinder>` element within with a `<mesh>` element. The `<mesh>`
       element should have a child `<uri>` that points to the top collada
       visual. The following snippet is what your top visual should contain.

         ```
         <visual name="top_visual">
           <geometry>
             <!-- The mesh tag indicates that we will use a 3D mesh as
                  a visual -->
             <mesh>
               <!-- The URI should refer to the 3D mesh. The "model:" 
                   URI scheme indicates that the we are referencing a Gazebo
                   model. -->
               <uri>model://velodyne_hdl32/meshes/velodyne_top.dae</uri>
             </mesh>
           </geometry>
         </visual>
         ```

    1. Test this change out in Gazebo, and you should see the following.

        [[file:files/velodyne_top_visual_unrotated.jpg|800px]]

    1. Notice that the visual is rotated incorrectly and has a vertical
       offset. These errors occur because the mesh's coordinate frame
       does not exactly match up with the coordinate frame of the SDF
       link. You can either edit the mesh in Blender to move the mesh, or
       you can apply a transform in SDF. Let's use the second option.

        ```
        <visual name="top_visual">
          <!-- Lower the mesh by half the height, and rotate by 90 degrees -->
          <pose>0 0 -0.0376785 0 0 1.5707</pose>
          <geometry>
            <mesh>
              <uri>model://velodyne_hdl32/meshes/velodyne_top.dae</uri>
            </mesh>
          </geometry>
        </visual>
        ```

    1.  The result should now be correct.

        [[file:files/velodyne_top_visual_rotated.jpg|800px]]

1. Now let's add the `velodyne_base` mesh, using what we learned from the
   `velodyne_top` mesh.

    ```
    <visual name="base_visual">
      <!-- Offset the visual by have the base's height. We are not rotating
           mesh since symmetrical -->
      <pose>0 0 -0.029335 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>model://velodyne_hdl32/meshes/velodyne_base.dae</uri>
        </mesh>
      </geometry>
    </visual>
    ```

1. We are done! The Velodyne model is looking good.

    [[file:files/velodyne_complete_visual.jpg|800px]]

# Next up

In the next tutorial we will add noise to the sensor reading. By default
simulation will provide near-perfect data, and this does not match the data
received in the real world.

[Sensor Noise](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i4)
