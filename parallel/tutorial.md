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
entities are clustered into“islands” that are mathematically decoupled from each other.
Thus each island can be simulated in parallel. After each step, the clustering of islands
is recalculated.

### Example Usage
### Set up catkin workspace
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
cd ~
mkdir -p ws/src
cd ws/src
git clone https://github.com/ros/catkin.git
hg clone https://bitbucket.org/osrf/gazebo
hg clone https://bitbucket.org/osrf/sdformat
~~~

Then we update gazebo and sdformat to the diagnostics related branch:
~~~
cd gazebo
hg update diagnostics_scpeters
cd ../sdformat
hg update island_threads
~~~

Next we download [package_gazebo.xml](https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml)
and [package_sdformat.xml](https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml),
copy them to the cloned `gazebo` and `sdformat` source folder as `package.xml`
~~~
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml > ~/ws/src/gazebo/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml > ~/ws/src/sdformat/package.xml
~~~



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