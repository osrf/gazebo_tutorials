# Introduction

The set of ROS 2 packages for interfacing with Gazebo are contained within a
meta package named `gazebo_ros_pkgs`.
See
[ROS 2 Overview](http://gazebosim.org/tutorials/?tut=ros2_overview)
for background information before continuing here.

The packages support ROS 2 Crystal and Gazebo 9, and can be installed from
debian packages or from source.

## Prerequisites

You should understand the basic concepts of ROS 2 and have gone through some
[ROS 2 Tutorials](https://index.ros.org/doc/ros2/Tutorials).

### Install ROS 2

ROS2 can be installed either through binary installation or source installation,
see the [ROS 2 installation page](https://index.ros.org/doc/ros2/Installation).
The current stable distribution is **Crystal**.

Alternatively, if you are an active developer setting up to contributing to code
base, it is advisable to have the source installation, as it provides more
access and control over the workflow. Additionally, the [master](https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos) provides the most updated code and currently set to ROS2 Dashing Diademate,
not ROS2 Crystal Clemmys.

> **TIP:**  Source the ros2 environment to load the libraries and executables
correctly. This will be achieved by sourcing `local_setup.bash` file from the
`install` directory when ros2 is built from sources.

### Install Gazebo

You can install Gazebo either from source or from pre-build packages. See
[Install Gazebo](http://gazebosim.org/tutorials?cat=install).

You should install Gazebo 9. If installing from source, be sure to build the
`gazebo9` branch.

> **Tip**: You may need to source Gazebo's setup file if you're having
difficulty finding plugins and other resources.
For example:

```
source /usr/share/gazebo/setup.sh
```

## Install gazebo\_ros\_pkgs

Follow either the instructions to install from debian packages,
or the instructions to install from source.

### Install from debian packages (on Ubuntu)

Assuming you already have some Crystal debian packages installed, install
`gazebo_ros_pkgs` as follows:

    sudo apt install ros-crystal-gazebo-ros-pkgs

### Install from source (on Ubuntu)

> **Tip**: These instructions require the use of the
  [colcon](https://colcon.readthedocs.io/en/released/) build tool, which is the
  standard tool used in ROS 2.

> **Tip:** The stable branch that will work with ros2 crystal and gazebo9 is
`crystal` branch. If you have ros2 from `master` branch, you need to use `ros2`
branch of `gazebo_ros_pkgs`. The following setup assumes intallation with `ros2`
branch of `gazebo_ros_pkgs`.

1. Create a workspace and get gazebo_ros_pkgs

```
mkdir -p ~/gazebo_ros_pks_ws/src
cd ~/gazebo_ros_pkgs
```

2. Create a file named `ros2_gazebo_ros_pkgs_supplement.repos` and copy the
contents of [this file](https://bitbucket.org/snippets/chapulina/geRKyA/ros2repos-supplement-gazebo_ros_pkgs)
that gets gazebo_ros_pkgs and additional packages needed.

> **NOTE:** The `version` tag indicates the branch we are checking out for a
particular repository

3. Get the packages to the src directory

```
vcs import src < ros2_gazebo_ros_pkgs_supplement.repos
```

4. Build the packages gazebo_ros_pkgs_ws workspace

```
colcon build --symlink-install
```

> **NOTE:** Before building this ensure that ros2 environment is sourced
correctly.

5. Source the gazebo_ros_pkgs environment to load the build libraries

```
source ~/gazebo_ros_pkgs_ws/install/local_setup.bash
```


> **NOTE:** If you've had any problems building, be sure to ask for help at
   [answers.gazebosim.org](http://answers.gazebosim.org/questions/).

> **NOTE:** Be sure to source the environment of a particular workspace in order
to run the programs from the packages in the workspace


```
source ~/<workspace>/install/setup.bash
```
> **Tip**: You can make this be automatically sourced for every new terminal
by running this once: `echo "source ~/ws/install/setup.bash" >> ~/.bashrc`

## Testing Gazebo and ROS 2 integration

Assuming your ROS 2 and Gazebo environments have been properly setup and built,
you should now be able to load Gazebo worlds which contain ROS 2 plugins, and to
insert models at runtime which have ROS 2 plugins in them.

Gazebo ROS packages provides several demo worlds for you to get a quick start
with the plugins. The demo worlds can be found
[here](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/worlds),
and are installed by default under
`/opt/ros/<distro>/share/gazebo_plugins/worlds/`.

Each world file comes with instructions at the top with some example commands
that you can run to test its functionality, be sure to check that out!

Let's try loading one of them now!

1. Open a new terminal

2. Source  ROS 2 as instructed when you installed ROS 2.

3. Make sure you have some core tools installed

```
sudo apt install ros-crystal-ros-core ros-crystal-geometry2
```

4. If you installed `gazebo_ros_pkgs` from source, source the workspace

```
~/<workspace>/install/setup.bash
```

5. Load the differential drive world with Gazebo

```
gazebo --verbose /opt/ros/crystal/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
```

6. The Gazebo GUI should appear with a simple vehicle

    [[file:figs/gazebo_ros_diff_drive.png|600px]]

7. On a new terminal (this is the 2nd one), run the following command to take a
   look at the world file.

```
gedit /opt/ros/crystal/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
````

8. See how the block on the top has a few example commands? Let's open a 3rd
   terminal and, again, source ROS 2 and `gazebo_ros_pkgs` as described above.

9. Then run one of the commands, for example:

```
ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1
```

10. You'll see the vehicle moving forward:

    [[file:figs/gazebo_ros_diff_drive_lin_vel.gif|600px]]

11. Try out the other commands listed on the file, and try mofidying their
   values to get a feeling of how things work. Also try out other demo worlds!
