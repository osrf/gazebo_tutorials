# Adding Color and Textures to a Model
This tutorial describes how color works in gazebo.
After reading you will know how to make objects in your simulation look more like their real-world counterparts.

## About Ambient, Diffuse, Specular, and Emissive
At the end this section you'll know what parameters relate to color and how they work.

The color of an object is determined using a [Blinn-Phong shading model](https://en.wikipedia.org/wiki/Blinn%E2%80%93Phong_shading_model).
There are four components that control the color: ambient, diffuse, specular, and emissive.
The [OpenGL Programming guide chapter on Lighting](http://www.glprogramming.com/red/chapter05.html) has detailed information about how these work.

### Component Values on Lights Versus Materials
The final color of an object depends both on the material and the lights shining on it.
Values on lights represent the intensity of light emitted.
Values on a material represent the percentage of light an object reflects.
The same material may be different colors depending on the light that hits it.

For example, consider a material with a diffuse color of `RGBA(1, 1, 0.5, 1.0)`.
Under a white light it would look <span style="background: #ffff80">yellow</span>.
If exposed to a light emitting a diffuse `RGBA(0 1 0 1)` then it would appear <span style="background:#00ff00">green</span>.
If exposed to a light emitting a diffuse `RGBA(1 0 0 1)` then it would appear <span style="background:#ff0000">red</span>.
A light with diffuse `RGBA(0 0 0.75 1)` would make the object appear <span style="background:#00005f; color: #ffffff">dark blue</span>.

[[file:files/light_and_material_interaction.png|256px]]

### Ambient
Ambient light is the color of an object when no lights are pointing at it.
It is completely uniform about the object.
Ambient light is meant to approximate light that has been reflected so many times it is hard to tell where it came from.

### Diffuse
This is the color of an object under a pure white light.
It is calculated using the light's direction and the surface normal where the light hits.
The part of an object with a normal antiparallel to a light source will be brightest from this component.

### Specular
The Specular component is the color and intensity of a highlight from a [specular reflection](https://en.wikipedia.org/wiki/Specular_reflection)
Higher values make an object appear more shiny.
A polished metal surface would have a very large specular value, while a piece of paper would have almost none.

### Emissive
The Emissive component can only be set on a material.
Like ambient light, emissive adds uniform color to an object.
It appears as if light was emitted from the object, though emissive light does not add light to other objects in the world.

### Compination of components

Each of the four components adds color to an object.
The final color of an object is the sum of all components.
After summation, if any red, green, or blue value goes above 1.0 then it is set to 1.0.

[[file:files/component_affects.png|600px]]

## Where Color Parameters Can Be Set
Specifying the color of an object means configuring both lights and the object's material.
At the end of this section you'll know where color parameters for lights and models are, and how they can be tweaked.

### Setting the Color Components of Lights
Light color can be specified in the world SDF file.
Ambient light is set globally in [&lt;scene&gt;](http://sdformat.org/spec?ver=1.6&elem=scene#scene_ambient).
The amount of ambient light in the world is a design choice that is up to you.
An indoor world may need a large global ambient light since every wall and surface is an opportunity to reflect light.
A simulation of satellites may have almost no ambient light since most is radiated out into space.

The [&lt;diffuse&gt;](http://sdformat.org/spec?ver=1.6&elem=light#light_diffuse) and [&lt;specular&gt;](http://sdformat.org/spec?ver=1.6&elem=light#light_specular) tags on a [&lt;light&gt;](http://sdformat.org/spec?ver=1.6&elem=light) set the color of diffuse and specular components emitted.
These tags require four floating point numbers (RGBA) between 0.0 and 1.0.
The last number (alpha) has no affect on lights.

Lights do not have emissive or ambient components.

### Setting the Color Components of Objects
The color components on objects can be set from SDF, an Ogre Material Script, or some types of meshes.

#### SDF
Every [&lt;visual&gt;](http://sdformat.org/spec?ver=1.6&elem=visual) has a [&lt;material&gt;](http://sdformat.org/spec?ver=1.6&elem=material) tag that controls the color components of an object.
Those tags are:

* [&lt;ambient&gt;](http://sdformat.org/spec?ver=1.6&elem=material#material_ambient)
* [&lt;diffuse&gt;](http://sdformat.org/spec?ver=1.6&elem=material#material_diffuse)
* [&lt;specular&gt;](http://sdformat.org/spec?ver=1.6&elem=material#material_specular)
    * **Note:** Specular intensity via SDF is currently not functioning in gazebo.
      See [issue #2120](https://bitbucket.org/osrf/gazebo/issues/2120/specular-material-not-working).
* [&lt;emissive&gt;](http://sdformat.org/spec?ver=1.6&elem=material#material_emissive)

#### Ogre Material Script

* [ambient](https://ogrecave.github.io/ogre/api/1.10/Material-Scripts.html#ambient)
* [diffuse](https://ogrecave.github.io/ogre/api/1.10/Material-Scripts.html#diffuse)
* [specular](https://ogrecave.github.io/ogre/api/1.10/Material-Scripts.html#specular)
* [emissive](https://ogrecave.github.io/ogre/api/1.10/Material-Scripts.html#emissive)

If the components are set both using SDF and an Ogre Material Script, then the SDF values are used for ambient, diffuse and emissive.
If both define specular then the final specular value is the addition of the Ogre Material Script and the SDF value.

#### Collada and OBJ meshes
If a visual uses a [&lt;mesh&gt;](http://sdformat.org/spec?ver=1.6&elem=geometry#geometry_mesh) and the mesh type is Collada (.dae) or Wavefront (.obj) then gazebo will try to get color values from those formats.

Collada color components for objects are taken from a [&lt;phong&gt; (page 8-69)](https://www.khronos.org/files/collada_spec_1_4.pdf) effect.
If a collada mesh does not have any effects then in gazebo 7 it will be displayed as if it had an emissive value of `RGBA(1 1 1 1)`.
In gazebo 8+ it will be displayed with gray diffuse and ambient components, but no emissive or specular components.

[[file:files/collada_default_color.png|256px]]

Wavefront obj color values are taken from the object's material: `Ka`, `Kd`, `Ks`, and `Ns`.
See [this Wikipedia article](https://en.wikipedia.org/wiki/Wavefront_.obj_file) for more info about OBJ files.

**Note:** OBJ materials may not [display correctly in versions earlier than gazebo 8](https://bitbucket.org/osrf/gazebo/issues/2455/wavefront-obj-wrong-normals-and-material).

## About Textures
Textures map an image onto a shape.
They add detail to a model without adding geometry.

The color added by a texture is made visible by both ambient and diffuse light.

[[file:files/texture_ambient_diffuse.png|600px]]

### Setting the Texture of an Object

#### Ogre Material Scripts
Ogre Material Scripts have many options for applying a texture to an object.
Read the [Ogre Documentation](https://ogrecave.github.io/ogre/api/1.10/Material-Scripts.html) for more information.

#### Collada
Collada files can apply textures to objects.
How to create such a file is outside the scope of this tutorial.
[Blender](https://www.blender.org/) is an open source modelling tool capable exporting to Collada.

Make sure the paths for textures given in `<library_images>` are relative to the collada file.

## Example Adding Color and Textures to a Model
This example will walk through how to add color to a basic model.

[[file:files/robot_before_after.png|256px]]

### Basic Files and Folders

Make a directory to contain files from this tutorial.

```
mkdir ~/color_tutorial
```

There are two parts to color: the model and the lights that light up a model.

Lights are specified in the world, so create a world with some light.
Save the following as:

```
~/color_tutorial/lit_world.world
```

It has a bright white directional light with some ambient light.

<include lang='xml' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/lit_world.world'/>

The next step is to create a model.
Refer to this image to see where files below need to be saved.
See the [model structure tutorial](tutorials?tut=model_structure) for more information about these files.

[[file:files/model_files.png|256px]]

The `models` folder itself must be in the environment variable [`GAZEBO_MODEL_PATH`](http://gazebosim.org/tutorials?tut=components#EnvironmentVariables).
This tutorial will assume you use the folder `~/color_tutorial/models`.

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/color_tutorial/models
```

Create directories for the model.

```
mkdir -p ~/color_tutorial/models/example_model/materials/scripts
mkdir -p ~/color_tutorial/models/example_model/materials/textures
mkdir -p ~/color_tutorial/models/example_model/meshes
```

Save this file as `model.config`.

<include lang='xml' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/model.config'/>

Save this file as `model.sdf`.

<include lang='xml' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/model.no_materials.sdf'/>

### Color Wheels and Power LED Using SDF

The wheels and power LED of this model wil be single uniform color, so they'll be set using SDF.
Here is an example material that makes the wheels dark blue.
Add it to both of the wheel `<visual>` tags.

<include lang='xml' from='/        <material> <!-- Wheel material -->/' to='/        </material> <!-- End wheel material -->/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/model.sdf'/>

[[file:files/dark_colored_wheels.png|200]]

The power LED is the source of its own light, so the emissive component will be used.
Add this to the `power_led` `<visual>` to make it always be fully green.

<include lang='xml' from='/        <material> <!-- LED material -->/' to='/        </material> <!-- End LED material -->/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/model.sdf'/>

[[file:files/green_led.png|300]]

### Color Body Using an Ogre Material Script

The body will be covered in a repeating texture.

Save this image as `seamless_texture.png`

[[file:files/seamless_texture.png]]

Then save this file as `repeated.material`.
It is an Ogre Material Script that repeats the texture over each face of the box.

<include lang='xml' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/repeated.material'/>

The result is a cube with each face having four copies of the seamless texture.

[[file:files/scaled_ogre_script.png|400px]]

### Color Head Using a Collada with a Texture
Save this image as `head_texture.png`.

[[file:files/head_texture.png|256]]

[Download and save this Collada file as head.dae](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/collada.dae).

This collada file references an image to use as a texture.
We need to double check that the paths are what we want them to be.

Open `head.dae` with your favorite text editor and look for `<library_images>`.

<include lang='xml' from='/ <library_images>/' to='/  </library_images>/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/head.dae'/>

The model is referencing an image without any path information.
It expects the png file to be in the same folder as `head.dae`, but we want to store the texture in another folder.
Change the path in `head.dae` to reference the texture location relative to `head.dae`.

```
  <library_images>
    <image id="head_texture_png" name="head_texture_png">
      <init_from>../materials/textures/head_texture.png</init_from>
    </image>
  </library_images>
```

The final step is to open it up in gazebo

[[file:files/collada_head.png|200px]]

### Open the Result in Gazebo
The environment variable `GAZEBO_MODEL_PATH` must be set to the path to the models folder you created.
Set it and launch gazebo with the world you saved earlier.

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/color_tutorial/models
gazebo --verbose ~/color_tutorial/lit_world.world
```

Insert the model you created into the world.

[[file:files/model_in_gazebo.png|480px]]
