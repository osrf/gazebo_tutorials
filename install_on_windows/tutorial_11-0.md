# Install on Windows

This document collects the different options available on Windows to bring
Gazebo into a Windows system.

Windows is not a fully officially supported platform by Open Robotics for
Gazebo. Open Robotics guarantees that the code is compatible with compilations
on Windows but there are no packages officially available. Since Gazebo is open
source, the Gazebo Community provides distribution using different projects.

## Community support

The Gazebo Community runs different projects to facilitate the
Windows installation providing all the dependencies and patch the code to solve
problems at runtime (help from Open Robotics is usually provided).

For the different Community projects available,
[checkout the issue tracker](https://github.com/osrf/gazebo/issues/2901). Below
is a quick overview of some of them.

### Gazebo on Windows via conda-forge

[conda-forge](https://conda-forge.org/) is a collection of packages to be used
with the conda package manager.

General feedback on using conda-forge is collected
[in this Gazebo issue](https://github.com/osrf/gazebo/issues/2899).

For contributions or bug reports on using Gazebo with conda the best place is the
[conda-forge issue tracker](https://github.com/conda-forge/gazebo-feedstock).

### Gazebo on Windows via vcpkg

[vcpkg](https://github.com/microsoft/vcpkg) is a package manager oriented to C
and C++ libraries and tools managed by Microsoft. There is a port for
[Gazebo](https://github.com/microsoft/vcpkg/tree/master/ports/gazebo).

General feedback on using vcpkg is collected
[in this Gazebo issue](https://github.com/osrf/gazebo/issues/3202).

For contributions or bug reports on using Gazebo with vcpkg the best place is the
[vcpkg issue tracker](https://github.com/microsoft/vcpkg/issues)

### Gazebo on Windows via WSL (Windows Subsystem for Linux)

[WSL](https://docs.microsoft.com/en-us/windows/wsl/install) is a compatibility
layer for running Linux binary executables on Windows 10 and newer.

The [Project DAVE](https://github.com/Field-Robotics-Lab/dave/wiki) provides
[detailed instructions](https://github.com/Field-Robotics-Lab/dave/wiki/Install-on-Windows-using-WSL2) 
to setup WSL and install Gazebo together with ROS Noetic using Ubuntu Linux 
installed on Windows. ROS installation can be avoided and just follow 
instructions until the point of installing ROS Noetic packages and
use apt to install `libgazebo11-dev`.
