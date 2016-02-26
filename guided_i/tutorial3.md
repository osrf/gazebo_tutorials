# Improve Appearance

Models with textures and 3D meshes can improve your visual experience, and
more importantly improve the realism of an environment. Simulated cameras
that feed information to vision processing algorithms will benefit from
models that appear realisitic as well.

In this section, we will use 3D meshes available on the Velodyne website to
improve the visual apperance of our model. More manufactures are making 3D
meshes available, however it can sometimes be difficult to find an exisitng
mesh.  In thses cases you can try your hand at mesh creation, work with an
artist, or contact the manufacturer directly.

Velodyne has a [STEP file](http://velodynelidar.com/docs/drawings/HDL32E_Outline_Model.STEP) for the HDL-32 located on [their
website](http://velodynelidar.com/docs/drawings/HDL32E_Outline_Model.STEP).
Gazebo can only use STL or Collada files, so we'll have to convert this
file and then add it to our model.

# Mesh Acquisition

1. Download the STEP file. Right-click and save-as on this [link](http://velodynelidar.com/docs/drawings/HDL32E_Outline_Model.STEP) to save the STEP file.

1. Open the STEP file in FreeCad.

    1. If you're using Ubuntu, you can install freecad using:
        ```
        sudo apt-get install freecad
        ```

    ```
    freecad ~/Downloads/HDL32E_Outline_Model.STEP
    ```

1. Select the base of Velodyne, by clicking on "HDL32 OUTLINE MODEL" in the
   left-hand `Labels & Attributes` panel.

    [[file://files/freecad_base.png|800px]]

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
   
At this point You should have two collada files: `velodyne_base.dae` and `velodyne_top.dae`. In the next section, we will cover adding these meshes to the SDF model. 


