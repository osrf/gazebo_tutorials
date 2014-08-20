# Gazebo Fluid Simulation Plugin using the Fluidix library #

The package allows the simulation of fluids in Gazebo. The fluid particle interactions are computed on the GPU 
using the [Fluidix](http://onezero.ca/documentation/) library (if a nvidia GPU is not available the simulation will run in CPU mode).

The fluid simulation runs as a separate physics engine which interacts with the rigid body physics engine of Gazebo through an interface (`include/FluidEngine.hh`). The interaction includes collision detection and the application of forces and torques from the fluid on the rigid objects.

The core of the fluid simulation is written in the `src/FluidEngine.cu` cuda file, it is built upon the [basic SPH example](http://onezero.ca/sample/?id=general_sph).

The package contains two plugins, one world plugin for updating the fluid and its interactions (`FluidWorldPlugin.cc`). And one client based system plugin for visualizing the fluid particles(`FluidVisPlugin.cc`).

### Installation requirements:

 * Install [Gazebo](http://gazebosim.org/)
 * Install [CUDA](https://developer.nvidia.com/cuda-downloads) (recommended 6.0)
 * Install [Fluidix](http://onezero.ca/documentation/)

### Buid instructions:

 * $ mkdir build
 * $ cd build
 * $ cmake ..
 * $ make 


### Running the plugin:

 * set the gazebo plugin paths
 * add the plugin to your world file (e.g worlds/fluid.world), or use one of the world examples from the package
 * run gazebo server with the world plugin $ gzserver worlds/fluid.world
 * run gazebo client with the system plugin $ gzclient -g build/libFluidVisPlugin.so


### Changing simulation:

 * change fluid plugin parameters from the sdf tags:
  * `<world_position>` and `<world_size>` set the fluid worlds center position and its size
  * `<fluid_position>` and `<fluid_volume>` set the center position of the fluid and its volume to be filled with particles
  * `<particle_nr>` if set to `0`, the given volume will be filled with fluid particles, otherwise the given particle number will be spawned.

 * by changing the source code from FluidWorldPlugin.cc




### TODOs:

 * implementing a newer SPH: [PCISPH](https://sph-sjtu-f06.googlecode.com/files/a40-solenthaler.pdf) or [IISPH](http://cg.informatik.uni-freiburg.de/publications/2013_TVCG_IISPH.pdf) for faster simulation and no compression of the fluid. This can be done in the `src/FluidEngine.cu` file by changing the algorithm.

 * currently the simulation only has the box implemented as a standard shape, see `FluidEngine::AddMovableBox` from `src/FluidEngine.cu`, it is implemented similarly to this [example](http://onezero.ca/sample/?id=init_manual). Using the same idea the rest of the collision types can be implemented as well: cylinder, sphere, plane. After implementation these need to be added in the `FluidWorldPlugin::CreateFluidCollision` method, similarly to the `box` type.

 * the force and torque interaction is done via the particle collisions, pressure force from the liquid is not taken into account.

### To know:

 * in order for the fluid simulation to detect collisions gazebo needs to use `.stl` files for collision.


### Possible issues:

 * in CMakeLists.txt, the cuda compiler might need graphics card specific flags:

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

 * The fluid simulation engine `FluidEngine.cc`:













# Overview

**Prerequisites:** [Attach a Mesh as Visual](http://gazebosim.org/tutorials/?tut=attach_meshes)

This tutorials demonstrates how the user can create composite models directly from other models in the [Gazebo Model Database](http://gazebosim.org/user_guide/started__models__database.html) by using the `<include>` tags and [`<joint>`](http://gazebosim.org/sdf/1.4.html#joint309) to connect different components of a composite model.

## Adding a Laser

Adding a laser to a robot, or any model, is simply a matter of including the sensor in the model.

1.  Go into your model directory from the previous tutorial:

        cd ~/.gazebo/models/my_robot

1.  Open `model.sdf` in your favorite editor.

1.  Add the following lines directly before the `</model>` tag near the end of the file.

    ~~~
        <include>
          <uri>model://hokuyo</uri>
          <pose>0.2 0 0.2 0 0 0</pose>
        </include>
        <joint name="hokuyo_joint" type="revolute">
          <child>hokuyo::link</child>
          <parent>chassis</parent>
          <axis>
            <xyz>0 0 1</xyz>
            <limit>
              <upper>0</upper>
              <lower>0</lower>
            </limit>
          </axis>
        </joint>
    ~~~

    The `<include>` block tells Gazebo to find a model, and insert it at a given `<pose>` relative to the parent model. In this case we place the hokuyo laser forward and above the robot.  The `<uri>` block tells gazebo where to find the model inside its model database (note, you can see a listing of the model database uri used by these tutorials at [here](http://gazebosim.org/models/), and the corresponding [mercurial repository](https://bitbucket.org/osrf/gazebo_models).

    The new `<joint>` connects the inserted hokuyo laser onto the chassis of the robot. The joint has and `<upper>` and `<lower>` limit of zero to prevent it from moving.

    The `<child>` name in the joint is derived from the [hokuyo model's SDF](https://bitbucket.org/osrf/gazebo_models/src/6cd587c0a30e/hokuyo/model.sdf?at=default), which begins with:

    ~~~
        <?xml version="1.0" ?>
        <sdf version="1.4">
          <model name="hokuyo">
            <link name="link">
    ~~~

    When the hokuyo model is inserted, the hokuyo's links are namespaced with their model name. In this case the model name is `hokuyo`, so each link in the hokuyo model is prefaced with `hokuyo::`.

1.  Now start gazebo, and add the robot to the simulation using the Insert tab on the GUI. You should see the robot with a laser attached.

    [[file:files/Add_laser_pioneer.png|640px]]

1.  (Optional)  Try adding a camera to the robot. The camera's model URI is `model://camera`, it should have been locally caches for you in:

        ls ~/.gazebo/models/camera/


    For reference, the SDF documentation can be found [here](http://gazebosim.org/sdf/).

## Next

[Next: Make a Simple Gripper](http://gazebosim.org/tutorials/?tut=simple_gripper)
