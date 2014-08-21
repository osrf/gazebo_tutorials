# Introduction

The package allows the simulation of fluids in Gazebo. The fluid particle interactions are computed on the GPU 
using the [Fluidix](http://onezero.ca/documentation/) library (if a nvidia GPU is not available the simulation will run in CPU mode).

**Prerequisites:**

 * Get the package from [bitbucket](https://bitbucket.org/ahaidu/gz_fluid).
 * Install [CUDA](https://developer.nvidia.com/cuda-downloads) (recommended 6.0).
 * Install [Fluidix](http://onezero.ca/documentation/).
 * Go through the basic Gazebo tutorials, especially through [world plugins](http://gazebosim.org/tutorials?tut=plugins_world), [system plugins](http://gazebosim.org/tutorials?tut=system_plugin), [transport library](http://gazebosim.org/tutorials?cat=transport) examples.
 * For deeper understanding of the particle simulation look through some [Fluidix examples](http://onezero.ca/sample/?id=general_basic).

# How the package works

The fluid simulation runs as a separate physics engine which interacts with the rigid body physics engine of Gazebo through an interface (`include/FluidEngine.hh`). 

The interaction includes:
 * collision detection
 * forces / torques application on the rigid objects
 * visualization of the particles

The core of the fluid simulation is written in the `src/FluidEngine.cu` cuda file and it is built upon the [basic SPH example](http://onezero.ca/sample/?id=general_sph).

The package contains two plugins, one world plugin for updating the fluid and its interactions (`FluidWorldPlugin.cc`). And one GUI system plugin for visualizing the fluid particles(`FluidVisPlugin.cc`).

## Build instructions

In a terminal go to the downloaded `gz_fluid` folder and run the following commands:

~~~
mkdir build
cd build
cmake ..
make
~~~

## Running the plugin:

 * Set the gazebo plugin and model paths
~~~
echo "export GAZEBO_PLUGIN_PATH=<install_path>/gz_fluid/build:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=<path>/gz_fluid/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
source ~/.bashrc
~~~
 * Add the plugin to your custom world file, or use one of the examples from the package
~~~
<?xml version="1.0"?>
<sdf version="1.5">
  <world name="fluid_world">
    ...
    <plugin name="FluidWorldPlugin" filename="libFluidWorldPlugin.so">
		<world_position>0 0 5.01</world_position>
		<world_size>1.5 1 10</world_size>
		<fluid_position>-0.5 0.0 0.8</fluid_position>
		<fluid_volume>0.4 0.95 0.5</fluid_volume>
		<particle_nr>0</particle_nr>
	</plugin>

  </world>
</sdf>
~~~
  * `<world_position>` and `<world_size>` set the fluid world center position and its size
  * `<fluid_position>` and `<fluid_volume>` set the center position of the fluid and its volume to be filled with particles
  * `<particle_nr>` if set to `0`, the given volume will be filled with fluid particles, otherwise the given particle number will be spawned.


 * Run gazebo server with the world plugin:
~~~
gzserver worlds/fluid.world
~~~
 * Run gazebo client with the system plugin:
~~~
gzclient -g build/libFluidVisPlugin.so
~~~


# To know:

### Collisions meshes

In order for the fluid simulation to detect collisions gazebo needs to use `.stl` files for collision meshes.


### Possible issues:

In CMakeLists.txt, the cuda compiler might need graphics card specific flags:

  `SET(CUDA_NVCC_FLAGS "-arch;sm_30 -use_fast_math -lm -ldl -lrt -Xcompiler \"-fPIC\"")`
  
### Some code explanation:

 * The world plugin `FluidWorldPlugin.cc`:

  * in the constructor the fluid engine is initialized
  * in `FluidWorldPlugin::Load` the sdf parameters are loaded, the fluid world is created, fluid is added, the objects from the environment are recreated in the fluid environment (when possible)
  * in `FluidWorldPlugin::Init` the publishers of the objects and fluids particles positions are initialized
  * `FluidWorldPlugin::OnUpdate` is called every world update event, there the fluid engine is updated one timestamp, a msg is sent for the rendering plugin with all the new particles position, and computed forces and torques are applied on the rigid objects.

 * The system plugin `FluidVisPlugin.cc`, used for rendering the fluid:
 
  * in `FluidVisPlugin::Load` the arguments (if given) are loaded for the type of rendering, `sphere` or default `point`. When sphere is selected the rendering gets slower if many particles are loaded.
  * in `FluidVisPlugin::Init` the subscribes for the fluid particle positions are loaded.
  * in `FluidVisPlugin::RenderAsPointsUpdate` or `FluidVisPlugin::RenderAsSpheresUpdate`, (depending on the rendering type) if a new message with the particle positions is available, these will be rendered.

 * The fluid simulation engine `FluidEngine.cu` is similar to the SPH example from Fluidix.



# Unfinished parts, TODOs:
If somebody is interested in further contributing to the package, many features still need work:

 * Implementing a newer SPH: [PCISPH](https://sph-sjtu-f06.googlecode.com/files/a40-solenthaler.pdf) or [IISPH](http://cg.informatik.uni-freiburg.de/publications/2013_TVCG_IISPH.pdf) for faster simulation and no compression of the fluid. This can be done in the `src/FluidEngine.cu` file by changing the algorithm.

 * Currently the simulation only has the box implemented as a standard shape, see `FluidEngine::AddMovableBox` from `src/FluidEngine.cu`, it is implemented similarly to this [example](http://onezero.ca/sample/?id=init_manual). Using the same idea the rest of the collision types can be implemented as well: cylinder, sphere, plane. After implementation these need to be added in the `FluidWorldPlugin::CreateFluidCollision` method, similarly to the `box` type.

 * The force and torque interaction is done via the particles surface collisions, pressure force from the liquid is not taken into account. A fancier algorithm would greatly increase realism.
