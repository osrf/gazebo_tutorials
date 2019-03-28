# Overview

An accurate simulation requires physically plausible inertial parameters:
the mass, center of mass location,
and the moment of inertia matrix of all links.
This tutorial will guide you through the process of obtaining and setting
these parameters if you have 3D models of the links.

Assuming homogeneous bodies (uniform mass density),
it is shown how to obtain inertial data using the free software MeshLab.
If you wish to skip the setup and only compute the volume, center of mass,
or inertia properties of your model, or quickly clean the model,
you can use [Mesh Cleaner](https://www.hamzamerzic.info/mesh_cleaner/),
a tool which runs MeshLab internally for this purpose.
You can also use the commercial product SolidWorks to compute these information.
For a guide on using SolidWorks, please refer to
[this question on answers.ros.org.](http://answers.ros.org/question/30539/choosing-the-right-coefficients-for-gazebo-simulation/)

# Summary of inertial parameters

## Mass

The mass is most easily measured by weighing an object.
It is a scalar with default units in Gazebo of kilograms (kg).
For a 3D uniform mesh, mass is computed by
calculating the geometric volume [length<sup>3</sup>]
and multiplying by density [mass / length<sup>3</sup>].

## [Center of Mass](https://en.wikipedia.org/wiki/Center_of_mass)

The center of mass is the point where the sum of weighted mass moments is zero.
For a uniform body, this is equivalent to the geometric centroid.
This parameter is a Vector3 with units of position [length].

## Moment of Inertia Matrix

The [moments of inertia](https://en.wikipedia.org/wiki/Moment_of_inertia)
represent the spatial distribution of mass in a rigid body.
It depends on the mass, size, and shape of a body
with units of [mass * length<sup>2</sup>].
The moments of inertia can be expressed as the components
of a symmetric positive-definite 3x3 matrix,
with 3 diagonal elements, and 3 unique off-diagonal elements.
Each inertia matrix is defined relative to a coordinate frame
or set of axes.
Diagonalizing the matrix
yields its principal moments of inertia (the eigenvalues)
and the orientation of its principal axes (the eigenvectors).

The moments of inertia are proportional to mass
but vary in a non-linear manner with respect to size.
Additionally, there are [constraints on the relative values
of the principal moments](http://physics.stackexchange.com/a/48273)
that typically make it much more difficult to estimate moments of inertia
than mass or center of mass location.
This difficulty motivates the use of software tools for computing
moment of inertia.

If you're curious about the math behind the inertia matrix, or just want an easy way to calculate the tensor for simple shapes,
[this wikipedia entry is a great resource](https://en.wikipedia.org/wiki/List_of_moments_of_inertia).

# Preparation

## Installing MeshLab

[Download MeshLab](http://meshlab.sourceforge.net/) from the official website and install it on your computer.
The installation should be straightforward.

Once installed, you can view your meshes in MeshLab (both DAE and STL formats are supported, which are those ones supported by Gazebo/ROS).

# Computing the inertial parameters

## Computing inertia of sphere

Open the mesh file in MeshLab.
For this example, a
[sphere.dae](https://github.com/assimp/assimp/raw/master/test/models/Collada/sphere.dae)
mesh is used.
To compute the inertial parameters, you first need to display the Layers dialog - `View->Show Layer Dialog`.
A panel opens in the right part of the window which is split in half - we're interested in the lower part containing text output.

Next, command MeshLab to compute the inertial parameters.
Choose `Filters->Quality Measure and Computations->Compute Geometric Measures` from the menu.
The lower part of the Layers dialog should now show some info about the inertial measures.
The sphere gives the following output:

    Mesh Bounding Box Size 2.000000 2.000000 2.000000
    Mesh Bounding Box Diag 3.464102
    Mesh Volume is 4.094867
    Mesh Surface is 12.425012
    Thin shell barycenter -0.000000 -0.000000 -0.000000
    Center of Mass is -0.000000 0.000000 -0.000000
    Inertia Tensor is :
    | 1.617916 -0.000000 0.000000 |
    | -0.000000 1.604620 -0.000000 |
    | 0.000000 -0.000000 1.617916 |
    Principal axes are :
    | 0.000000 1.000000 0.000000 |
    | -0.711101 -0.000000 0.703089 |
    | -0.703089 0.000000 -0.711101 |
    axis momenta are :
    | 1.604620 1.617916 1.617916 |

### Radius
The bounding box of the sphere is a cube with side length 2.0,
which implies that the sphere has a radius of 1.0.

### Volume
A sphere of radius 1.0 should have a volume of `4/3*PI` (4.189),
which is close to the computed value of 4.095.
It is not exact since it is a triangular approximation.

### Surface Area
The surface area should be `4*PI` (12.566),
which is close to the computed value of 12.425.

### Center of Mass
The center of mass is given as the origin (0,0,0).

### Inertia matrix
The inertia matrix (aka inertia tensor) of a sphere should be diagonal with
[principal moments of inertia](https://en.wikipedia.org/wiki/List_of_moments_of_inertia)
of `2/5 mass` since `radius = 1`.
It is not explicitly stated in the output, but the mass
is equal to the volume (implicitly using a density of 1),
so we would expect diagonal matrix entries of `8/15*PI` (1.676).
The computed inertia tensor appears diagonal
for the given precision with principal moments
ranging from [1.604,1.618], which is close to the expected value.

## Duplicate faces

One thing to keep in mind is that duplicate faces within a mesh will affect
the calculation of volume and moment of inertia.
For example, consider another spherical mesh:
[ball.dae](https://bitbucket.org/osrf/gazebo_models/raw/9f0a80a06cf3/robocup_3Dsim_ball/meshes/ball.dae).
Meshlab gives the following output for this mesh:

    Mesh Bounding Box Size 1.923457 1.990389 1.967965
    Mesh Bounding Box Diag 3.396207
    Mesh Volume is 7.690343
    Mesh Surface is 23.967396
    Thin shell barycenter 0.000265 0.000185 0.000255
    Center of Mass is 0.000257 0.000195 0.000292
    Inertia Tensor is :
    | 2.912301 0.001190 0.000026 |
    | 0.001190 2.903731 0.002124 |
    | 0.000026 0.002124 2.906963 |
    Principal axes are :
    | 0.108262 -0.895479 0.431738 |
    | -0.120000 0.419343 0.899862 |
    | 0.986853 0.149229 0.062058 |
    axis momenta are :
    | 2.902563 2.907949 2.912483 |

This mesh is approximately the same size,
with bounding box dimensions in the range [1.92,1.99],
but its calculations are different by nearly double:

* volume: 7.69 vs. 4.09
* principal moments: [2.90,2.91] vs. [1.60,1.62]

There is a clue to the difference when you look at the numbers
of vertices and faces (listed in the bottom of the MeshLab window):

* sphere.dae: 382 vertices, 760 faces
* ball.dae: 362 vertices, 1440 faces

Each mesh has a similar number of vertices, but `ball.dae` has
roughly twice as many faces.
Running the command
`Filters` `->` `Cleaning and Repairing` `->` `Remove Duplicate Faces`
reduces the number of faces in `ball.dae` to 720 and gives
more reasonable values for the volume (3.84)
and principal moments of inertia (1.45).
It makes sense that these values are slightly smaller
since the bounding box is slightly smaller as well.

## Scaling to increase numerical precision

Meshlab currently prints the geometric information
with 6 digits of fixed point precision.
If your mesh is too small, this may substantially
limit the precision of the inertia tensor,
for example:

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

It seems like we have what we were seeking for.
But when you look thoroughly, you will see one bad thing - the output is written out only up to 6 decimal digits.
As a consequence, we lose most of the valuable information in the inertia tensor.
To overcome lack of precision in the Inertia Tensor,
you can scale up the model so that the magnitude of the inertia is increased.
The model can be scaled using `Filters->Normals, Curvatures and Orientation->Transform: Scale`.
Enter a scale in the dialog and hit `Apply`.

[[file:files/meshlab.jpg|800px]]

To decide the scaling factor `s` to choose, recall that MeshLab uses the volume
as a proxy for mass, which will vary as <span class='typ'>s<sup>3</sup></span>.
Furthermore, the inertia has an addition dependence on <span class='typ'>length<sup>2</sup></span>,
so the moment of inertia will change according to <span class='typ'>s<sup>5</sup></span>.
Since there is such a large dependence on `s`,
scaling by a factor of 10 or 100 may be sufficient.

Now, instruct MeshLab to recompute the geometrical measures again,
and the `Inertia Tensor` entry should have more precision.
Then multiply the inertia tensor by <span class='typ'>1/s<sup>5</sup></span> to undo the scaling.

## Getting the Center of Mass

It is not always the case that MeshLab uses the same length units as you'd want (meters for Gazebo).
However, you can easily tell the ratio of MeshLab units to your desired units by looking at the `Mesh Bounding Box Size` entry.
You can e.g. compute the bounding box size in your desired units and compare to the MeshLab's one.

Multiply the `Center of Mass` entry with the computed ratio and you have the coordinates of the Center of Mass of your mesh.
However, if the link you are modeling is not homogeneous, you will have to compute the Center of Mass using other methods (most probably by real experiments).

## Rescaling the moment of inertia values

Just like the center of mass location must be scaled to the correct units,
the moment of inertia should be scaled as well,
though the scale factor should be squared to account for the
<span class='typ'>length<sup>2</sup></span> dependence in the moment of inertia.
In addition, the inertia should be multiplied by
the measured `mass` and divided by the computed volume from
the text output.

# Filling in the tags in URDF or [SDF](http://sdformat.org)

The next step is to record the computed values to the URDF or [SDF](http://sdformat.org) file containing your robot (it is assumed you already have the robot model; if not, follow the tutorial
[Make a Model](http://gazebosim.org/tutorials?tut=build_model&cat=build_robot)).

In each link you should have the `<inertial>` tag.
It should look like the following (in [SDF](http://sdformat.org)):

    <link name='antenna'>
      <inertial>
        <pose>-0.022 0.0203 0.02917 0 0 0</pose>
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

The `<origin>` or `<pose>` are used to enter the Center of Mass position (relative to the link's origin; especially not relative to the link's visual or collision origin).
The rotational elements can define a different coordinate from for the moment of inertia axes.
If you've found out the center of mass experimentally, fill in this value, otherwise fill in the correctly scaled value computed by MeshLab.

The `<inertia>` tag contains the inertia tensor you have computed in the previous step.
Since the matrix is symmetric, only 6 numbers are sufficient to represent it.
The mapping from MeshLab's output is the following:

    | ixx ixy ixz |
    | ixy iyy iyz |
    | ixz iyz izz |

As a quick check that the matrix is sane, you can use the rule that the diagonal entries should have the largest values and be positive, and the off-diagonal numbers should more or less approach zero.

Precisely, the matrix has to be positive definite (use your favorite maths tool to verify that).
Its diagonal entries also have to
[satisfy the triangle inequality](http://physics.stackexchange.com/a/48273),
ie. `ixx + iyy >= izz`, `ixx + izz >= iyy` and `iyy + izz >= ixx`.

# Checking in Gazebo

To check if everything is done correctly, you can use Gazebo's GUI client.

* Using Gazebo standalone
  1. Run Gazebo

            gazebo
  1. Spawn your robot

            gz model -f my_robot.sdf

* Using Gazebo with ROS
  1. Run Gazebo

            roslaunch gazebo_ros empty_world.launch
  1. Spawn your robot (substitute `my_robot`, `my_robot_description` and `MyRobot` with your robot's package/name):
      * [SDF](http://sdformat.org) model:

                rosrun gazebo_ros spawn_model -sdf -file `rospack find my_robot_description`/urdf/my_robot.sdf -model MyRobot
      * URDF model:

                rosrun gazebo_ros spawn_model -urdf -file `rospack find my_robot_description`/urdf/my_robot.urdf -model MyRobot

As soon as your model loads, pause the world and delete the ground_plane (this is not needed, but it usually makes debugging easier).

Go to the Gazebo menu and select `View->Inertia`.
Every link should now display a purple box with green axes.
The center of each box is aligned with the specified center of mass of its link.
The sizes and orientations of the boxes correspond to unit-mass boxes with the same inertial behavior as their corresponding links.
This is useful for debugging the inertial parameters, but we can make one more thing to have the debugging easier.

You can temporarily set all the links to have a mass of 1.0 (by editing the URDF or [SDF](http://sdformat.org) file).
Then all the purple boxes should have more or less the same shapes as the bounding boxes of their links.
This way you can easily detect problems like misplaced Center of Mass or wrongly rotated Inertia Matrix.
Do not forget to enter the correct masses when you finish debugging.

To fix a wrongly rotated Inertia Matrix (which in fact happens often), just swap the ixx, iyy, izz entries in the model file until the purple box aligns with its link.
Then you obviously also have to appropriately swap the ixy, ixz and iyz values (when you swap ixx`<->`iyy, then you should negate ixy and swap ixz`<->`iyz).

[[file:files/gazebo_inertia.jpg|800px]]

# Further improvements

## Simplify the model

MeshLab only computes correct inertia parameters for closed shapes.
If your link is open or if it is a very complex or concave shape, it might be a good idea to simplify the model (e.g.
in Blender) before computing the inertial parameters.
Or, if you have the collision shapes for your model, use them in place of the full-resolution model.

## Non-homogeneous bodies

For strongly non-homogeneous bodies, this tutorial might not work.
There are two problems.
The first one is that MeshLab assumes uniform-density bodies.
The other is that MeshLab computes the Inertia Tensor relative to the computed center of mass.
However, for strongly non-homogeneous bodies, the computed center of mass will be far from the real center of mass, and therefore the computed inertia tensor might be just wrong.

One solution is to subdivide your link to more homogeneous parts and connect them with fixed joints, but that is not always possible.
The only other solution would be to find out the inertia tensor experimentally, which would surely take a lot of time and effort.

# Conclusion

We have shown the process of getting the correct inertia parameters for your robot model,
the way how to enter them in a URDF or [SDF](http://sdformat.org) file,
and also the way how to make sure the parameters are entered correctly.
