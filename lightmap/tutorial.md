# Tutorial: Visual Lightmap#

Lightmaps which can be used to improve the rendering performance of complex scenes. Lightmaps can be thought of as texture maps with lighting information pre-baked into the texture. The resulting models appear as if they are shaded by lights in the environment but in fact they are just textures.

Lightmaps are typically used for models that do not move with respect to the pose of the lights in the scene. For example, static buildings in an environment with a directional light.

## Create a model with lightmap

Lightmaps can be created using popular 3D modeling tools. e.g. [Blender](http://wiki.blender.org/index.php/Doc:2.6/Manual/Textures/Mapping/UV/Unwrapping#Lightmap). In this tutorial, we'll save you the time by supplying a mesh and a pre-generated lightmap.

In Gazebo, a lightmap is associated to a model as opposed to the whole scene because there is almost always the chance that something moves in a robot simulation.

### Create Table Model

First create a table model. The model will not use the lightmap yet.

~~~
mkdir -p ~/.gazebo/models/my_lightmap_table
~~~


Download the mesh and texture files.

~~~
mkdir -p ~/.gazebo/models/my_lightmap_table/meshes
cd ~/.gazebo/models/my_lightmap_table/meshes
wget http://gazebosim.org/models/table_marble/meshes/table_lightmap.dae
~~~

~~~
mkdir -p ~/.gazebo/models/my_lightmap_table/materials/scripts
mkdir -p ~/.gazebo/models/my_lightmap_table/materials/textures
cd ~/.gazebo/models/my_lightmap_table/materials/textures
wget http://gazebosim.org/models/table_marble/materials/textures/marble.png
wget http://gazebosim.org/models/table_marble/materials/textures/table_lightmap.png
~~~

Create a material script that will be used by the table model:

~~~
gedit ~/.gazebo/models/my_lightmap_table/materials/scripts/table_lightmap.material
~~~

Paste the following contents:

~~~
material Table/Marble_Lightmap
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture marble.png
      }
    }
  }
}
~~~

Create a `model.config` file:

~~~
gedit ~/.gazebo/models/my_lightmap_table/model.config
~~~

Paste the following contents:

~~~
<?xml version="1.0"?>

<model>
  <name>Lightmap Table</name>
  <version>1.0</version>
  <sdf version="1.5">model.sdf</sdf>

  <author>
    <name>Joe</name>
    <email>joe@sixpack.org</email>
  </author>

  <description>
    A table with lightmap.
  </description>
</model>
~~~

Create a `model.sdf` file

~~~
gedit ~/.gazebo/models/my_lightmap_table/model.sdf
~~~

Paste the following:

~~~
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="my_lightmap_table">
    <static>true</static>
    <pose>0 0 0.648 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model:///my_lightmap_table/meshes/table_lightmap.dae</uri>
            <scale>0.25 0.25 0.25</scale>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://my_lightmap_table/meshes/table_lightmap.dae</uri>
            <scale>0.25 0.25 0.25</scale>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://my_lightmap_table/materials/scripts</uri>
            <uri>model://my_lightmap_table/materials/textures</uri>
            <name>Table/Marble_Lightmap</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
~~~

Run gazebo:

        gazebo

Insert the `Lightmap Table` model into the scene:

[[file:Table_no_lightmap.png|640px]]


### Apply Lightmap

If you pay attention earlier, you will notice that we have not made use of `table_lightmap.png` yet. This is the lightmap to be applied. But first, we will need to tell Gazebo that dynamic lighting should not be applied to the model since we want to use lightmaps.

Edit the `model.sdf` file:

~~~
gedit ~/.gazebo/models/my_lightmap_table/model.sdf
~~~

Locate the material SDF element in the model.sdf and set `lighting` to be `false`

%%%
        <material>
          <script>
            <uri>model://my_lightmap_table/materials/scripts</uri>
            <uri>model://my_lightmap_table/materials/textures</uri>
            <name>Table/Marble_Lightmap</name>
          </script>
%%%
~~~
            <lighting>false</lighting>
~~~
%%%
        </material>
%%%

If you insert `my_lightmap_table` into the scene again, you should notice that the model now only has a texture but is no longer shaded.

[[file:Table_no_lighting.png | 640px]]

Then tell the material script to blend the lightmap with the existing texture.

~~~
gedit ~/.gazebo/models/my_lightmap_table/materials/scripts/table_lightmap.material
~~~

Add a `texture_unit` that uses the `table_lightmap.png` texture

%%%
material Table/Marble_Lightmap
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture marble.png
      }
%%%
~~~
      texture_unit
      {
        texture table_lightmap.png
      }
~~~
%%%
    }
  }
}
%%%

Finally, relaunch Gazebo and insert the table model:

[[file:Table_lightmap.png | 640px]]
