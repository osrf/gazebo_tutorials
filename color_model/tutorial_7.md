# Adding Color to a Model
This tutorial describes how to make your model appear a certain color.
At the end you'll know what parameters relate to color and where they can be tweaked.

## About Ambient, Diffuse, Specular Emissive
The color of an object is determined using a [Blinn-Phong shading model](https://en.wikipedia.org/wiki/Blinn%E2%80%93Phong_shading_model).
There are four components that control the color: ambient, diffuse, specular, and emissive.
The [OpenGL Programming guide chapter on Lighting](http://www.glprogramming.com/red/chapter05.html) has detailed information about how these work.

### Component values on Lights versus Materials
The final color of an object depends both on the material and the lights shining on it.
Values on lights represent the intensity of light emitted.
Values on a material represent the percentage of light an object reflects.
The same material may be different colors depending on the light that hits it.

For example, consider a material with a diffuse color of `RGBA(1, 1, 0.5, 1.0)`.
Under a pure white like it would look <span style="background: #ffff80">yellow</span>.
If exposed to a light emitting a diffuse `RGBA(0 1 0 1)` then it would appear <span style="background:#00ff00">green</span>.
If exposed to a light emitting a diffuse `RGBA(1 0 0 1)` then it would appear <span style="background:#ff0000">red</span>.
A light with diffuse `RGBA(0 0 0.75 1)` would make the object appear <span style="background:#00005f; color: #ffffff">dark blue</span>.

[[file:files/light_and_material_interaction.png|256px]]

Each of the four components adds color to an object.
The final color of an object is the sum of all components.
After summation, if any red, green, or blue value goes above 1.0 then it is set to 1.0.

[[file:files/component_affects.png|600px]]

### Ambient
Ambient light is the color of an object when no lights are pointing at it.
It is completely uniform about the object.

Ambient light is meant to approximate light that has been reflected so many times it is hard to tell where it came from.
Indoors, global ambient light may be a large value since every wall and surface is an opportunity to reflect light.
A simulation of satellites may have almost no ambient light since most is radiated out into space.

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

## Adding color to objects
Specifying the color an object means configuring both lights and the material on the object itself.

### Setting the Color components of Lights
Light color can only be set through SDF.
Ambient light is set globally in [&lt;scene&gt;](http://sdformat.org/spec?ver=1.6&elem=scene#scene_ambient).

The [&lt;diffuse&gt;](http://sdformat.org/spec?ver=1.6&elem=light#light_diffuse) and [&lt;specular&gt;](http://sdformat.org/spec?ver=1.6&elem=light#light_specular) tags on a [&lt;light&gt;](http://sdformat.org/spec?ver=1.6&elem=light#light_diffuse) set the color of diffuse and specular components emitted by the light.
These tags require four floating point numbers (RGBA) between 0.0 and 1.0.
The last number (alpha) has no affect on lights.

Lights do not have an emissive or ambient components.

### Setting the Color components of objects
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

### Setting the texture of an object

#### Ogre Material Scripts
Ogre Material Scripts have many options for applying a texture to an object.
Read the [Ogre Documentation](https://ogrecave.github.io/ogre/api/1.10/Material-Scripts.html) for more information.

#### Collada
Collada files can apply textures to objects.
How to create such a file is outside the scope of this tutorial.
[Blender](https://www.blender.org/) is an open source modelling tool capable of exporting a textured model as a Collada file.

Make sure the paths for textures given in `<library_images>` are relative to the collada file.

## Example adding Color To a Model

This will walk through how to add color to a basic model.

[[file:files/robot_before_after.png|256px]]

### Basic files and folders

There are two parts to color: the model and the lights that light up a model.

The first step is to create a world with some light.
Save the following as `lit_world.world`.
It has a bright white directional light with some ambient light.

<include lang='xml' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/lit_world.world'/>

The next step is to create a model.
At the end of this section you'll have the following files and folders.
See the [model structure tutorial](tutorials?tut=model_structure) for more information about these files.

Create the directories first.

``
mkdir -p models/example_model/materials/scripts
mkdir -p models/example_model/materials/textures
mkdir -p models/example_model/materials/meshes
```

[[file:files/model_files.png|256px]]

Save this file as `model.config`.

<include lang='xml' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/model.config'/>

Save this file as `model.sdf`.

<include lang='xml' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/model.no_materials.sdf'/>

### Color Wheels and Power LED using SDF

The wheels and power LED of this model wil be single uniform color, so using SDF values will be good enough.
First add a material to both wheels.
Here is an example that makes the wheels dark blue.

<include lang='xml' from='/      <visual name="wheel1_visual">/' to='/      </visual> <!-- End wheel visuals -->/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/model.sdf'/>

The power LED should always be on.
It is the source of it's own light, so the emissive component will be used.
Add this to the model to make a green LED

<include lang='xml' from='/      <visual name="power_led_visual">/' to='/      </visual> <!-- End LED visual -->/' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/model.sdf'/>

### Color body using an Ogre Material Script

Save this file as `repeated.material`.
This is an Ogre Material Script that repeats a texture in a file `seamless_texture.png` over each face of the box.

<include lang='xml' src='http://bitbucket.org/osrf/gazebo_tutorials/raw/default/color_model/files/repeated.material'/>

Insert the model into gazebo.
The result should be a cube with each face having four copies of the seamless texture.

[[file:files/scaled_ogre_script.png|400px]]

### Color head using a Collada with a Texture
