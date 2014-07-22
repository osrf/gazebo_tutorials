#Tutorial: Importing Meshes#
This tutorial describes how to import 3D meshes into Gazebo.

### Prepare the Mesh

Gazebo uses a right-hand coordinate system where +Z is up (vertical), +X is forward (into the screen), and +Y is to the left.

**Reduce Complexity**

 Many meshes can be overly complex. A mesh with many thousands of triangles should be reduced or split into separate meshes for efficiency. Look at the documentation of your 3D mesh editor for information about reducing triangle count or splitting up a mesh.

**Center the mesh**

 The first step is to center the mesh at (0,0,0) and orient the front (which can be subjective) along the x-axis.

**Scale the mesh**

 Gazebo uses the metric system. Many meshes (especially those from 3D warehouse), use English units. Use your favorite 3D editor to scale the mesh to a metric size.

### Export the Mesh ###

Once the mesh has been properly prepared, export it as a Collada file. This format will contain all the 3D information and the materials.

### Test the Mesh ###

The easiest way to test a mesh is to create a simple world file [my_mesh.world](http://bitbucket.org/osrf/gazebo_tutorials/raw/import_mesh/files/my_mesh.world) that loads the mesh. Replace `my_mesh.dae` with the actual filename of the mesh.

<include from='/#include/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/import_mesh/files/my_mesh.world' />

Then just launch Gazebo in the directory where is the file:

        gazebo my_mesh.world

### Test Mesh ###

You can use these [duck.dae](http://www.c3dl.org/wp-content/2.0Release/Resources/duck.dae) and [duck.png](http://www.c3dl.org/wp-content/2.0Release/Resources/duck.png) mesh files. Put them together in the same directory as the world file. Since the duck mesh is defined with the y-axis as up, you can put a rotation in the sdf so that it displays upright:

~~~
<visual name="visual">
  <pose>0 0 0 1.5708 0 0</pose>
  <geometry>
    <mesh><uri>file://duck.dae</uri></mesh>
  </geometry>
</visual>
~~~

![duck.dae mesh in Gazebo](http://gazebosim.org/w/images/c/cb/TutorialMeshDuck.png)
