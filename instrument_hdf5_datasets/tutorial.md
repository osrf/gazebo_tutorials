# Introduction

Instrument tools are provided to dump physical data into [HDF5](https://www.hdfgroup.org/HDF5/)
format. The datasets, together with
the [Benchmark Problems for Multibody Dynamis (BPMD)](https://grasp.robotics.cs.rpi.edu/bpmd/)
framework are used to compare different methods in solving multibody systems with bilateral
joints and unilateral frictional contacts in an unbiased way. These datasets will help researchers
to concentrate on analysis of existing methods and construction of more accurate solvers,
without worrying about implementation of the whole physics engine.

# Example Usage

## Install hdf5
~~~
sudo apt-get install libhdf5-dev
~~~

## Build Gazebo
This HDF5 instrument tool requires building Gazebo from source, with the cmake parameter
`HDF5_INSTRUMENT` [default False] as True.
[Learn how to build Gazebo from source](/tutorials?tut=install_from_source&cat=install)

    cd ~/gazebo
    mkdir build
    cd build
    cmake -DHDF5_INSTRUMENT=True ../
    make -j4
    sudo make install

## Collect Datasets
### Use only gzserver

~~~
./test/integration/INTEGRATION_physics_inertia_ratio
~~~

### Use the world file

~~~
gazebo ~/gazebo/worlds/friction_demo.world
~~~


Then a file named `ode_frames.hdf5` will be generated at the directory exactly where the
above command is run.

## View the HDF5 file

hdfvivew is used to open the hdf5 files. You can install it via the terminal:

~~~
sudo apt-get install hdfview
~~~

Then open the stored file with:

~~~
hdfview ode_frames.hdf5
~~~

A hierarchical file shows up:

[[file:files/hdf5.png|800px]]


**Note**: The instrument tool will save hierarchical data for each time step, so it will be
slow to write the data into the `ode_frames.hdf5` file.
Be patient, especially for complex simulation scenarios such as Atlas robots or many body simulation.



