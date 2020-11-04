# ROS 2 integration overview

> For ROS 1, see
  [ROS integration overview](http://gazebosim.org/tutorials?tut=ros_overview).

Gazebo is a stand-alone application which can be used independently of ROS or
ROS 2. The integration of Gazebo with either ROS version is done through a set
of packages called
[gazebo\_ros\_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs).
These packages provide a bridge between Gazebo's C++ API and transport system,
and ROS 2 messages and services.

> **Not all functionality from ROS 1 has been ported to ROS 2 yet. You can
check the progress on
[this issue](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/512).**

> If you're interested in porting official or your own custom plugins, check
out the
[ROS2 migration contribution guide](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-migration-contribution-guide).

The ROS 2 package `gazebo_ros_pkgs` is a metapackage which contains the
following packages:

* `gazebo_dev`: Provides a cmake configuration for the default version of
                Gazebo for the ROS distribution. So downstream packages can
                just depend on `gazebo_dev` instead of needing to find
                Gazebo by themselves.

* `gazebo_msgs`: Message and service data structures for interacting with
                 Gazebo from ROS 2.

* `gazebo_ros`: Provides convenient C++ classes and functions which can be
                used by other plugins, such as `gazebo_ros::Node`, conversion
                and testing utilities. It also provides some generally useful
                plugins.

* `gazebo_plugins`: A series of Gazebo plugins exposing sensors and other
                    features to ROS 2. For example, `gazebo_ros_camera`
                    publishes ROS 2 images, and `gazebo_ros_diff_drive` provides
                    an interface for controlling and instrospecting differential
                    drive robots through ROS 2.

## Target versions

The ROS 2 port of `gazebo_ros_pkgs` has debian packages released
for the Crystal Clemmys distribution.

The code can also be built from source using the
[ros2](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2) branch
against
[ROS 2 master branches](https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos).

Currently, the only supported Gazebo version is Gazebo 9.

## Differences from ROS 1

Several changes and refactoring were done to `gazebo_ros_pkgs` when migrating
from ROS 1 to ROS 2.

Some goals of the refactoring were:

* Take advantage of new ROS 2 features, such as masterless discovery.
* Remove code which duplicates functionality already present in Gazebo.
* Reduce duplication by standardizing common functionality, such as how to set
  ROS namespaces, parameters and topic remapping.
* Modernize the codebase, making use of the latest SDFormat, Gazebo and Ignition  APIs, as well as ROS 2's
  [style guidelines](https://github.com/ros2/ros2/wiki/Developer-Guide#c-1)
  and linters.
* Add tests and demos for all ported functionality.

**Detailed migration guides for each plugin can be found on the
[gazebo\_ros\_pkgs wiki](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki).**
See some general highlights below.

### Init

The ROS 1 integration required that Gazebo be launched with the
`gazebo_ros_api_plugin` system plugin, which would initialize ROS.

There's no such requirement with ROS 2. Gazebo can be started without any
plugins and ROS-2-enabled plugins can be added at runtime.

### Node

In ROS 1, each plugin typically had one or more `ros::NodeHandle` instances to
interact with ROS.

In ROS 2, plugins use `gazebo_ros::Node` instead, which allows each plugin to
exist as its own node in the ROS graph, with its own parameters, namespace,
loggers, etc. Plugins also don't need to worry about spinning the node or
keeping callback queues - `gazebo_ros` handles all that internally.

### SDF parsing

There are several configurations which Gazebo ROS plugins commonly want to
set through SDF, and in the ROS 1 implementation, there was a lot of duplicate
code on plugins parsing the same things, sometimes following loose conventions.

In ROS 2, common configurations like namespace, ROS parameters and topic
remapping rules are parsed by `gazebo_ros::Node`, so there's no need for plugins
to reimplement them every time.

### `gazebo_ros_api_plugin`

In ROS 1, `gazebo_ros_api_plugin` was a large plugin which offered a lot of
functionality in a bundle, giving users little flexibility to opt-in/out of
features.

In ROS 2, this plugin is being split into smaller, more focused plugins. You can
read the migration details on
[ROS 2 Migration: gazebo\_ros\_api\_plugin](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-gazebo_ros_api_plugin).

