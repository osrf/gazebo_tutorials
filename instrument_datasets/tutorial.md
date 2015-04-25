# Tutorial: Instrument HDF5 datasets for benchmark 

# Introduction

Instrument tools are provided to dump physical data into HDF5 format. The datasets, together with the [Benchmark Problems for Multibody Dynamis (BPMD)](https://grasp.robotics.cs.rpi.edu/bpmd/) framework are used to unbiasly compare the different methods in solving multibdoy system with bilateral joints and unilateral frictional contacts. These datasets will hep analyzing and constructing new, better and faster solvers, concentrating on algorithms instead of the whole physics engine. 

# Example Usage 
**Note:** This tool requires building Gazebo from source

## Step 1: Enable the Macro variable `HDF5_INSTRUMENT` in file [gazebo/cmake/gazebo_config.h.in](https://bitbucket.org/osrf/gazebo/src/32287918490425e4f9fd5a06fbca3ade3e95f55d/cmake/gazebo_config.h.in?at=default), by adding the following line to the file: 
~~~
#cmakedefine HDF5_INSTRUMENT
~~~

Then save the file, build and install gazebo: 

~~~
mkdir build  
cd build    
cmake ../
make -j4  
sudo make install
~~~


## Step 2: Collect the HDF5 datasets 

### Use only gzserver 

~~~
./test/regression/REGRESSION_351_world_step 
~~~

### Use gazebo with world file 

~~~
gazebo ~/gazebo/worlds/friction_demo.world 
~~~

Then a file named `ode_frames.hdf5` would be generated at the directory exactly where the above command is run. 

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



We will use the PR2 world to create a state log file.

Start by running the Gazebo server with the `-r` command line option
~~~
gzserver -r worlds/pr2.world
~~~

After a few seconds, stop the server using ctrl-c.

A new time stamped directory should exist in `~/.gazebo/log` with one subdirectory and a `state.log` file. Here is an example
~~~
~/.gazebo/log/2013-07-25T07\:29\:05.122275/gzserver/state.log
~~~

You can verify this log file by replaying it in Gazebo.
~~~
gazebo -p ~/.gazebo/log/*/gzserver/state.log
~~~

## Step 2: Filter a state log file

The [[Tools#Data_Log_Tool | `gzlog`]] command line tool provides mechanisms for stepping through a log file and echoing the contents of a log file to screen. The echo to screen feature can be combined with a filter to produce a log file that contains specific information such as just the pose of models and links.

Try echoing the recorded state log file to screen.
~~~
gzlog echo ~/.gazebo/log/*/gzserver/state.log
~~~

You should see a lot of information scroll by.

Now let's remove all velocity, acceleration, and force information from the log file. This will leave just pose information.
~~~
gzlog echo ~/.gazebo/log/*/gzserver/state.log --filter *.pose/*.pose
~~~

The `--filter` option is a flexible command line argument to extract information from a log file. More information can be found [[Tools#Data_Log_Tool | here]].

It is also possible to filter based on simulation time using a Hz filter.
For example, we can output state information at 30 Hz using:
~~~
gzlog echo ~/.gazebo/log/*/gzserver/state.log -z 30
~~~

These filters can be combined and piped to a file for playback. This may take some time depending on the size of the state.log.
~~~
gzlog echo ~/.gazebo/log/*/gzserver/state.log -z 30 --filter *.pose/*.pose > /tmp/filtered_state.log
~~~

This log file can then be replayed in Gazebo
~~~
gazebo -p /tmp/filtered_state.log
~~~
