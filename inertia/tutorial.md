For physically plausible behavior of the model you need to correctly set up the inertia matrix, the center of mass and the weight of all links. This tutorial will guide you through the process of obtaining and filling in these information if you don't have them (and you have 3D models of the links).

This tutorial shows how to obtain inertial data using the free software MeshLab (assuming homogeneous bodies). You can also use the commercial product SolidWorks to compute these information. For a guide on using SolidWorks, plese refer to [this question on answers.ros.org.](http://answers.ros.org/question/30539/choosing-the-right-coefficients-for-gazebo-simulation/)   

# Preparation

## Installing MeshLab

[Download MeshLab](http://meshlab.sourceforge.net/) from the official website and install it on your computer. The installation should be straightforward.

Once installed, you can view your meshes in MeshLab (both DAE and STL formats are supported, which are those ones supported by Gazebo/ROS).

# Computing the inertial parameters

## First try in MeshLab

Open the mesh file in MeshLab. To compute the inertial parameters, you first need to display the Layers dialog - `View->Show Layer Dialog`. A panel opens in the right part of the window which is split in half - we're interested in the lower part containing text output.

Next, command MeshLab to compute the inertial parameters. Choose `Filters->Quality Measure and Computations->Compute Geometric Measures` from the menu. The lower part of the Layers dialog should now show some info about the inertial measures. An example output:

    Mesh Bounding Box Size 0.044000 0.221000 0.388410
    Mesh Bounding Box Diag 0.449043 
    Mesh Volume is 0.001576
    Mesh Surface is 0.136169
    Thin shell barycenter -0.021954 0.008976 0.012835
    Center of Mass is -0.021993 0.001259 0.001489
    Inertia Tensor is :
    | 0.000008 -0.000000 -0.000000 |
    | -0.000000 0.000001 -0.000000 |
    | -0.000000 -0.000000 0.000007 |
    Principal axes are :
    | 0.999999 0.000166 0.001241 |
    | -0.000113 0.999104 -0.042310 |
    | -0.001247 0.042310 0.999104 |
    axis momenta are :
    | 0.000008 0.000001 0.000007 |

It seems like we have what we were seeking for. But when you look thoroughly, you will see one bad thing - the output is written out only up to 6 decimal digits. As a consequence, we lose most of the valuable information in the inertia tensor.

Now comes the time to explain what does the MeshLab output mean. In MeshLab, there is no way to tell what the mass of the model is. Therefore, MeshLab has to use some other number in place of the mass. What makes sense in such case is to take the volume of the model (thus assuming unit density of the material). The `Mesh Volume` entry is present in the text output (and is expressed in some "generic" units corresponding to the units used for length).

[[file:files/meshlab.jpg|800px]]

## Getting the Center of Mass

It is not always the case that MeshLab uses the same length units as you'd want (meters for Gazebo). However, you can easily tell the ratio of MeshLab units to your desired units by looking at the `Mesh Bounding Box Size` entry. You can e.g. compute the bounding box size in your desired units and compare to the MeshLab's one. 

Multiply the `Center of Mass` entry with the computed ratio and you have the coordinates of the Center of Mass of your mesh. However, if the link you are modelling is not homogeneous, you will have to compute the Center of Mass using other methods (most probably by real experiments). 

## Getting the Inertia Tensor

How to overcome the problem with low precision of the Inertia Tensor floats in the text output? You can scale up the model so that all the numbers grow sufficiently large for not being cropped. The model can be scaled using `Filters->Normals, Curvatures and Orientation->Transform: Scale`. Enter a scale in the dialog and hit `Apply`. What scale to choose? It is reasonable to scale the model in such way that it has unit volume in MeshLab. That is done using a scale of `1/Mesh Volume` with the Mesh Volume read from the text output in Layers Dialog.  

Now, instruct MeshLab to recompute the geometrical measures again, et voilà, the `Inertia Tensor` entry looks much better now. The only thing that remains is to copy the inertia tensor into a mathematical software and multiply it by `1/s^2` where `s` is the reciprocal of the scale used for upscaling the model.

# Filling in the tags in URDF/SDF

The next step is to record the computed values to the URDF/SDF file containing your robot (it is assumed you've already had the robot model; if not, follow the tutorial [Make a Model](http://gazebosim.org/tutorials?tut=build_model&cat=build_robot)).

In each link you should have the `<inertial>` tag. It should look like the following (in SDF):

    <link name='antenna'>
      <inertial>
        <pose>-0.022 0.0203 0.02917 0 -0 0</pose>
        <mass>0.56</mass>
        <inertia>
          <ixx>0.004878</ixx>
          <ixy>-6.2341e-07</ixy>
          <ixz>-7.4538e-07</ixz>
          <iyy>0.00090164</iyy>
          <iyz>-0.00014394</iyz>
          <izz>0.0042946</izz>
        </inertia>
      </inertial>
      <collision name='antenna_collision'>
        ...
      </collision>
      <visual name='antenna_visual'>
        ...
      </visual>
      ...
    </link> 

or like this one (in URDF):

    <link name="antenna">
      <inertial>
        <origin rpy="0 0 0" xyz="-0.022 0.0203 0.02917"/>
        <mass value="0.56"/>
        <inertia ixx="0.004878" ixy="-6.2341e-07" ixz="-7.4538e-07" iyy="0.00090164" iyz="-0.00014394" izz="0.0042946"/>
      </inertial>
      <visual>
        ...
      </visual>
      <collision>
        ...
      </collision>
	</link>

The `<mass>` should be entered in kilograms and you have to find it out experimentally (or from specifications).

The `<origin>` or `<pose>` are used to enter the Center of Mass position (relative to the link's origin; especially not relative to the link's visual or collision origin). The rotational elements will be ignored. If you've found out the center of mass experimentally, fill in this value, otherwise fill in the correctly scaled value computed by MeshLab.

The `<inertia>` tag contains the inertia tensor you have computed in the previous step. Since the matrix is symmetric, only 6 numbers are sufficient to represent it. The mapping from MeshLab's output is the following:

    | ixx ixy ixz |
    | ixy iyy iyz |
    | ixz iyz izz |

As a quick check that the matrix is sane, you can use the rule that the diagonal entries should have the largest values, and the off-diagonal numbers should more or less approach zero. No diagonal entry can be negative.

# Checking in Gazebo

To check if everything is done correctly, you can use Gazebo's GUI client. We'll cover the case where Gazebo is used together with ROS. First, run Gazebo:

    roslaunch gazebo_ros empty_world.launch

Then spawn your robot (SDF model):

    rosrun gazebo_ros spawn_model -sdf -file `rospack find nifti_robot_description`/urdf/nifti_robot.sdf -model NIFTi

or (URDF model):

    rosrun gazebo_ros spawn_model -urdf -file `rospack find nifti_robot_description`/urdf/nifti_robot.urdf -model NIFTi

and as soon as it loads, pause the world and delete the ground_plane (this is not needed, but it usually makes debugging easier).

Go to the Gazebo menu and select `View->Center of Mass / Inertia`. Every link should now display a purple box with green axes. The center of each box is aligned with the specified center of mass of its link. The sizes and orientations of the boxes correspond to unit-mass boxes with the same inertial behavior as their corresponding links. This is useful for debugging the inertial parameters, but we can make one more thing to have the debugging easier.

You can temporarily set all the links to have a mass of 1.0 (by editing the URDF/SDF). Then all the purple boxes should have more or less the same shapes as the bounding boxes of their links. This way you can easily detect problems like misplaced Center of Mass or wrongly rotated Inertia Matrix. Do not forget to enter the correct masses when you finish debugging.

To fix a wrongly rotated Inertia Matrix (which in fact happens often), just swap the ixx, iyy, izz entries in the model file until the purple box aligns with its link. Then you obviously also have to appropriately swap the ixy, ixz and iyz values (when you swap ixx<->iyy, then you should negate ixy and swap ixz<->iyz). 

[[file:files/gazebo_inertia.jpg|800px]]

If your model has a lot of links and the Gazebo image becomes a mess, you can always delete the links you do not need at the moment (by selecting them on the World tab and pressing Delete on your keyboard).

# Further improvements

## Simplify the model

MeshLab only computes correct inertia parameters for closed shapes. If your link is open or if it is a very complex or concave shape, it might be a good idea to simplify the model (e.g. in Blender) before computing the inertial parameters. Or, if you have the collision shapes for your model, use them in place of the full-resolution model.

## Non-homogeneous bodies

For strongly non-homogeneous bodies, this tutorial might not work. There are two problems. The first one is that MeshLab assumes uniform-density bodies. The other is that MeshLab computes the Inertia Tensor relative to the computed center of mass. However, for strongly non-homogeneous bodies, the computed center of mass will be far from the real center of mass, and therefore the computed inertia tensor might be just wrong.

One solution is to subdivide your link to more homogeneous parts and connect them with fixed joints, but that is not always possible. The only other solution would be to find out the inertia tensor experimentally, which would surely take a lot of time and effort.

# Conclusion

We have shown the process of getting the correct inertia parameters for your robot model, the way how to enter them in a URDF/SDF file, and also the way how to make sure the parameters are entered correctly.