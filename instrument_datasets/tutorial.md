# Tutorial: Instrument HDF5 datasets

# Introduction

Instrument tools are provided to dump physical data into HDF5 format. The datasets, together with the [Benchmark Problems for Multibody Dynamis (BPMD)](https://grasp.robotics.cs.rpi.edu/bpmd/) framework are used to unbiasly compare the different methods in solving multibdoy system with bilateral joints and unilateral frictional contacts. These datasets will hep analyzing and constructing new, better and faster solvers, concentrating on algorithms instead of the whole physics engine.

# Example Usage
## Build Gazebo
Build Gazebo with the cmake parameter `HDF5_INSTRUMENT` [default False] as True
~~~
cd ~/gazebo
mkdir build
cd build
cmake -DHDF5_INSTRUMENT=True ../
make -j4
sudo make install
~~~

## Collect Datasets
### Use only gzserver

~~~
./test/integration/INTEGRATION_physics_inertia_ratio
~~~

### Use the world file

~~~
gazebo ~/gazebo/worlds/friction_demo.world
~~~


Then a file named `ode_frames.hdf5` would be generated at the directory exactly where the above command is run.

## View the HDF5 file

hdfvivew is used open the hdf5 files, install it via terminal:

~~~
sudo apt-get install hdfview
~~~

Then open the stored file with:

~~~
hdfview ode_frames.hdf5
~~~

A hierarchical file shows up:

[[file:files/hdf5.png|800px]]


*Note* The instrument tool will save hierarchical data for each time step, so it will be slow to write the data into the `ode_frames.hdf5` file. Be patient, especially for complex simulation scenarios such as Atlas robots or many body simulation.



