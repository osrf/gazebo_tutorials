# Overview
The complexity of simulated robots, spatial size of environments, and fidelity of
sensor simulation all play a role in determining what can be simulated with the
constraint of operating at or near real-time. The Gazebo physics update loop is
one of the primary consumers of CPU cycles. With an limitation in the speed of
solution algorithms to solve the underlying mathematical problem that represents
those physical constraints, parallelization of the physics engine is the direction
to go in order to help improve performance, with the goal of running complex robots
and environments in real-time.

# Parallel Strategies
Two strategies to parallelize physics have been implemented: island thread and
position error correction thread. For more details about these two strategies results
and analysis, please refer to the parallel physics reports on the Gazebo webpage.

## Island Thread
The first strategy attempts to parallelize simulation of non-interacting entities.
Simulated entities are interacting if they are connected by an articulated joint (such
as a revolute or universal joint) or are connected via contact. Groups of interacting
entities are clustered into "islands" that are mathematically decoupled from each other.
Thus each island can be simulated in parallel. After each step, the clustering of islands
is recalculated.

## Position Error Correction Thread
The second strategy attempts to speed up the constraint resolution algorithm within
islands for the QuickStep solver. The ODE QuickStep solver is the default solver in
Gazebo and solves constraints posed as a Linear Complementarity Problem (LCP). As an
iterative, fixed time step solver, it is prone to position errors, such as
interpenetration of objects.

In order to correct these errors, an impulse is computed that is applied to the
interpenetrating objects to push them apart. This method of position correction
adds artificial energy into the system. To correct for this additional energy, two
equations are solved. An error correcting LCP is used to correct the object position,
while the velocity is updated without interpenetration error correction. These two
equations can be solved in parallel, due to the independence between each other, two
threads can be used simultaneously to do the computation, which comprises the second
parallelization strategy.

# Running the code
Currently these parallelization strategies require building gazebo and sdformat from
source.
The following section provides instructions for doing this with a catkin workspace.

## Set up catkin workspace
Currently, the sdformat needs to be build using a specific branch, we have to set up a
catkin workspace to build gazebo and sdformat againt system installs of dependencies.

Using catkin requires the python `catkin-pkg` to be installed:

~~~
# Ubuntu
sudo apt-get install python-catkin-pkg
# Others
sudo pip install catkin-pkg
~~~

Here we create a `ws` folder as workspace under `HOME` directory. Then we clone
catkin, gazebo and sdformat into the `ws/src` folder.

~~~
export WS=${HOME}/ws/gazebo_parallel
mkdir -p ${WS}/src
cd ${WS}/src
git clone https://github.com/ros/catkin.git
git clone https://github.com/osrf/gazebo
git clone https://github.com/osrf/sdformat
~~~

Then we update gazebo and sdformat to the diagnostics related branch:

~~~
cd ${WS}/src/gazebo
git checkout diagnostics_scpeters
cd ${WS}/src/sdformat
git checkout island_threads
~~~

Next we download [package_gazebo.xml](https://github.com/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml)
and [package_sdformat.xml](https://github.com/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml),
copy them to the cloned `gazebo` and `sdformat` source folder as `package.xml`

~~~
curl https://github.com/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml > ${WS}/src/gazebo/package.xml
curl https://github.com/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml > ${WS}/src/sdformat/package.xml
~~~

Gazebo can then be built using the `catkin_make_isolated` command:

~~~
cd ${WS}
./src/catkin/bin/catkin_make_isolated
~~~

If you want to use diagnostic timers to evaluate performance,
define the `ENABLE_DIAGNOSTICS` symbol during compilation.
This will output the diagnostic timing data to the `~/.gazebo/diagnostics` folder.

~~~
cd ${WS}
./src/catkin/bin/catkin_make_isolated -DENABLE_DIAGNOSTICS=1
~~~

## Run the code

Threading is currently enabled using custom sdformat parameters
on the [island_threads branch](https://github.com/osrf/sdformat/branches/compare/island_threads%0Ddefault#diff):

* `island_threads`: integer number of threads to use for island threading
* `thread_position_correction`: flag to turn threading on for ODE quickstep position error correction

These flags have been added to [physics profiles](http://gazebosim.org/tutorials?tut=preset_manager&cat=physics)
in several world files on the `diagnostics_scpeters` branch of gazebo:

* [test/worlds/revolute\_joint\_test.world](https://github.com/osrf/gazebo/src/diagnostics_scpeters/test/worlds/revolute_joint_test.world#cl-12)
* [worlds/pr2.world](https://github.com/osrf/gazebo/src/diagnostics_scpeters/worlds/pr2.world#cl-12)
* [worlds/dual_pr2.world](https://github.com/osrf/gazebo/src/diagnostics_scpeters/worlds/dual_pr2.world#cl-12)

Simulate two pr2 robots without threading:

~~~
. ${WS}/devel_isolated/setup.bash
gazebo --verbose -o unthrottled0 \
  ${WS}/src/gazebo/worlds/dual_pr2.world
~~~

Simulate two pr2 robots with island threading:

~~~
. ${WS}/devel_isolated/setup.bash
gazebo --verbose -o unthrottled2 \
  ${WS}/src/gazebo/worlds/dual_pr2.world
~~~

Simulate two pr2 robots with threaded position error correction:

~~~
. ${WS}/devel_isolated/setup.bash
gazebo --verbose -o split_unthrottled0 \
  ${WS}/src/gazebo/worlds/dual_pr2.world
~~~

Simulate two pr2 robots with both types of threading:

~~~
. ${WS}/devel_isolated/setup.bash
gazebo --verbose -o split_unthrottled2 \
  ${WS}/src/gazebo/worlds/dual_pr2.world
~~~
