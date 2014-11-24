# Install Gazebo on Linux distributions (non Ubuntu)

Gazebo is available to install on other Linux distributions different than Ubuntu 
([Ubuntu install instructions](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=installation)
are hosted in a different tutorial). Linux distributions providing gazebo packages: 
Debian, Fedora and Arch. 

## Debian

Gazebo is available as an 
[official package in Debian Sid](https://packages.debian.org/source/sid/gazebo).
Currently the version in Sid is gazebo3, but should be updated before the end 
of 2014 to the latest release of Gazebo.

1. Install Gazebo.

        sudo apt-get install gazebo3
        # For developers that works on top of Gazebo, one extra package
        sudo apt-get install libgazebo-dev

## Fedora

Gazebo is available as an [official package in Fedora](https://apps.fedoraproject.org/packages/gazebo)
(the official maintainer is Rich Mattes). Depending on the Fedora version, the 
version of Gazebo available is different:

 * Rawhide is usually hosting one of the latest releases
 * Fedora 21: gazebo-3.1
 * Fedora 20: gazebo-3.1
 * Fedora 19: gazebo-3.0

1. Install Gazebo

        sudo yum install gazebo
        # For developers that works on top of Gazebo, one extra package
        sudo yum install gazebo-devel

## Arch

Gazebo is currently in the AUR: Arch User Repository (Benjamin Chrétien is the
maintainer). This means that it is not in the official package repositories and 
users need to compile it from source. The easiest way to install it is to use 
an AUR helper, such as yaourt or packer:

1. Install Gazebo

        sudo yaourt -S gazebo
        # or 
        sudo packer -S gazebo

## Other linux distributions?

If you know of any other Linux distribution supporting Gazebo installation,
feel free to [create an issue](https://bitbucket.org/osrf/gazebo_tutorials/issues)
to expand this tutorial.
