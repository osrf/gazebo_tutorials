# Introduction

**IMPORTANT: This is experimental and outdated. Use with caution.**

The package allows the simulation of fluids in Gazebo. The fluid particle
interactions are computed on the GPU using the
[Fluidix](http://onezero.ca/documentation/) library (if a nvidia GPU is not
available the simulation will run in CPU mode).

**Prerequisites:**

  * Nvidia graphics card
  * Go through the basic Gazebo tutorials, especially through [world plugins](/tutorials?tut=plugins_world), [system plugins](/tutorials?tut=system_plugin), [transport library](/tutorials?cat=transport) examples.
 * For deeper understanding of the particle simulation look through some [Fluidix examples](http://onezero.ca/sample/?id=general_basic).

# Install

1. Install [CUDA](https://developer.nvidia.com/cuda-downloads) (recommended 6.0)

    ~~~
    sudo apt-get install nvidia-cuda-dev nvidia-cuda-toolkit
    ~~~

1. Install [Fluidix](http://onezero.ca/downloads/)

    Use the online form to get links to Fluidix

    ~~~
    mkdir /tmp/fluidix
    unzip ~/Downloads/Fluidix*.zip -d /tmp/fluidix
    cd /tmp/fluidix
    sudo ./install.sh
    ~~~

1. Install [gazebo](/tutorials?cat=install) from source using the `fluid_sph` branch

    ~~~
    [...]
    cd ~; git clone https://github.com/osrf/gazebo
    cd ~/gazebo
    git checkout fluid_sph
    [...]
    ~~~

# How the package works

The fluid simulation runs as a separate physics engine which interacts with
the rigid body physics engine of Gazebo through an interface
(`include/FluidEngine.hh`).

The interaction includes:
 * collision detection
 * forces / torques application on the rigid objects
 * visualization of the particles

The core of the fluid simulation is written in the `src/FluidEngine.cu` cuda file and it is built upon the [basic SPH example](http://onezero.ca/sample/?id=general_sph).

The package contains two plugins, one world plugin for updating the fluid and its interactions (`FluidWorldPlugin.cc`). And one GUI system plugin for visualizing the fluid particles(`FluidVisPlugin.cc`).

## Running the plugin:

1. Set the gazebo plugin and model paths

    ~~~
    echo "export GAZEBO_PLUGIN_PATH=/<path_to_gazebo_source>/gazebo/build/plugins:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
    echo "export GAZEBO_MODEL_PATH=/<path_to_gazebo_source>/gazebo/media/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
    source ~/.bashrc
    ~~~

1. Add the plugin to your world file, or use one of the examples from the package.

    ~~~
    gedit ~/fluid.world
    ~~~

    Copy the following into the open editor, save, and quit.

    ~~~
    <?xml version="1.0"?>
    <sdf version="1.5">
      <world name="fluid_world">

        <!-- A global light source -->
        <include>
          <uri>model://sun</uri>
        </include>

        <!-- A box (plane + fluid is not supported) -->
        <model name="box">
          <static>true</static>
          <pose>0 0 0 0 0 0</pose>
          <link name="link">
            <collision name="collision">
              <geometry>
                <box>
                  <size>20 20 0.1</size>
                </box>
              </geometry>
            </collision>
            <visual name="visual">
              <geometry>
                <box>
                  <size>20 20 0.1</size>
                </box>
              </geometry>
            </visual>
          </link>
        </model>


        <plugin name="FluidWorldPlugin" filename="libFluidWorldPlugin.so">
          <world_position>0 0 1.01</world_position>
          <world_size>1.5 1 10</world_size>
          <fluid_position>-0.5 0.0 0.8</fluid_position>
          <fluid_volume>0.4 0.95 0.5</fluid_volume>
          <particle_nr>0</particle_nr>
        </plugin>

      </world>
    </sdf>
    ~~~

    Where:
      * `<world_position>` and `<world_size>` set the fluid world center position and its size
      * `<fluid_position>` and `<fluid_volume>` set the center position of the fluid and its volume to be filled with particles
      * `<particle_nr>` if set to `0`, the given volume will be filled with fluid particles, otherwise the given particle number will be spawned.


1. Run gazebo client with the system plugin:

    ~~~
    gazebo ~/fluid.world -g /tmp/gz_fluid/build/libFluidVisPlugin.so
    ~~~

# To know:

### Collisions meshes

In order for the fluid simulation to detect collisions gazebo needs to use `.stl` files for collision meshes.


### Possible issues:

In CMakeLists.txt, the cuda compiler might need graphics card specific flags:

  `SET(CUDA_NVCC_FLAGS "-arch;sm_30 -use_fast_math -lm -ldl -lrt -Xcompiler \"-fPIC\"")`

### Fluidix CMake include path

For a default install the fluidix headers are located in `/opt/fluidix/include`, if the paths differ make sure to change them accordingly in the `src/CMakeLists.txt` file.

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
