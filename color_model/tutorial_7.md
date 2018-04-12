# Adding Color to a Model
This tutorial describes how to add color to a model.
The first two sections describe color related parameters and where they can be tweaked.
Afterwards there is an example that walks through using all of the techniques.

## About Ambient, Diffuse, Specular Emissive
The color of an object is determined using a [Blinn-Phong shading model](https://en.wikipedia.org/wiki/Blinn%E2%80%93Phong_shading_model).
There are four components that control the color: ambient, diffuse, specular, and emissive.
The [OpenGL chapter on Lighting](http://www.glprogramming.com/red/chapter05.html) has detailed information about how it works.

### Component values on Lights versus Materials
The final color of an object depends both on the material and the lights shining on it.
Values on lights represent the intensity of light emitted.
Values on a material represent the percentage of light an object reflects.
The same material may be different colors depending on the light that hits it.

For example, consider a material with a diffuse color of `RGBA(1, 1, 0.5, 1.0)`.
If exposed to a light emitting a diffuse `RGBA(0 1 0 1)` then it would appear green.
If exposed to a light emitting a diffuse `RGBA(1 0 0 1)` then it would appear blue.
A light with diffuse `RGBA(0 0 0.75 1)` would make the object would be dark blue.

[[file:light_and_material_interaction.png|256px]]

Each of the four components adds color to an object.
The final color of an object is the sum of all components.
Any red, green, or blue value goes above 1.0 after summation is clipped to 1.0.

[[file:component_affects.png|600px]]

### Ambient
Ambient light is the color of an object when no lights are pointing at it.
It is completely uniform.

Ambient light is meant to approximate light that has been reflected so many times it is hard to tell where it came from.
For an indoor environment global ambient light may be a large value.
Every wall and surface indoors gives light many opportunities to be reflected.
A simulation of satellites may want almost no ambient light since most light is radiated out into space instead of reflected.

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
The alpha value has no affect on lights.

Lights do not have an emissive or ambient components.

### Setting the Color components of objects
The color components on objects can be set using SDF or an Ogre Material Script.

Every [&lt;visual&gt;](http://sdformat.org/spec?ver=1.6&elem=visual) has a [&lt;material&gt;](http://sdformat.org/spec?ver=1.6&elem=material) tag that controls the color components of an object.
Those tags are:

* [&lt;ambient&gt;](http://sdformat.org/spec?ver=1.6&elem=material#material_ambient)
* [&lt;diffuse&gt;](http://sdformat.org/spec?ver=1.6&elem=material#material_diffuse)
* [&lt;specular&gt;](http://sdformat.org/spec?ver=1.6&elem=material#material_specular)
    * **Note:** Specular intensity via SDF is currently not functioning in gazebo.
      See [issue #2120](https://bitbucket.org/osrf/gazebo/issues/2120/specular-material-not-working).
* [&lt;emissive&gt;](http://sdformat.org/spec?ver=1.6&elem=material#material_emissive)

The values can also be set using an ogre material.

* [ambient](https://ogrecave.github.io/ogre/api/1.10/Material-Scripts.html#ambient)
* [diffuse](https://ogrecave.github.io/ogre/api/1.10/Material-Scripts.html#diffuse)
* [specular](https://ogrecave.github.io/ogre/api/1.10/Material-Scripts.html#specular)
* [emissive](https://ogrecave.github.io/ogre/api/1.10/Material-Scripts.html#emissive)

If the components are set both using SDF and an ogre script, then the SDF values are used for ambient, diffuse and emissive.
If both define &lt;specular&gt; then the final specular value is the addition of the Ogre script and the SDF value.

If a &lt;visual&gt; geometry uses a mesh and the mesh type is collada or wavefront obj then gazebo will get color values from those formats.

Collada color components for objects are taken from a [&lt;phong&gt; (page 8-69)](https://www.khronos.org/files/collada_spec_1_4.pdf) effect.
If a mesh does not have any effects then in gazebo 7 it will be displayed as if it had an emissive value of `RGBA(1 1 1 1)`.
In gazebo 8+ it will be displayed with gray diffuse and ambient components, but no emissive or specular components.

[[file:collada_default_color.png|256px]]

Wavefront obj color values are taken from the object's material.

**Note:** OBJ materials may not [display correctly in versions earlier than gazebo 8](https://bitbucket.org/osrf/gazebo/issues/2455/wavefront-obj-wrong-normals-and-material).

TODO can SDF color be combined with textures set using Ogre Materials?
TODO can SDF color be combined with textures set using Collada?
TODO can Ogre Material color be combined with textures set using Collada?

## About Textures

### Ogre Material

### Collada

### Example: Color a small Robot

Trash bot:
World with a yellowish directional light
and some ambient light
Emissive green power LED
Collada for robot texture
  Texture with "Trashbot text"
Shiny cylinder for top
  Ogre script for Shiny metal
Flat matt color for wheels
  Cylinders, SDF color values
