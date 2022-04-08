# Install Gazebo using Ubuntu packages

This tutorial covers the installation of Gazebo packages using the the
`osrfoundation.org` repository, designed for those who want to follow the
faster development path but still a stable one.

Gazebo is also released as an Ubuntu official package: ([check which
version](https://packages.ubuntu.com/search?suite=all&section=all&arch=any&keywords=gazebo&searchon=sourcenames)
is available for every distribution. If you are a [ROS](http://ros.org) user, please
read the tutorial about [ROS/Gazebo
installation](http://gazebosim.org/tutorials?tut=ros_wrapper_versions&cat=connect_ros).

## Default installation: one-liner

1. Install

        curl -sSL http://get.gazebosim.org | sh

2. Run

        gazebo

## Alternative installation: step-by-step

1. Setup your computer to accept software from packages.osrfoundation.org.

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

    You can check to see if the file was written correctly. For example, in Ubuntu Xenial, you can type:

        $ cat /etc/apt/sources.list.d/gazebo-stable.list
        deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main

1. Setup keys

        wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

1. Install Gazebo.

    First update the debian database:

        sudo apt-get update

    Hint: make sure the apt-get update process ends without any errors, the console output ends in `Done` similar to below:

        $ sudo apt-get update
        ...
        Hit http://ppa.launchpad.net xenial/main Translation-en
        Ign http://us.archive.ubuntu.com xenial/main Translation-en_US
        Ign http://us.archive.ubuntu.com xenial/multiverse Translation-en_US
        Ign http://us.archive.ubuntu.com xenial/restricted Translation-en_US
        Ign http://us.archive.ubuntu.com xenial/universe Translation-en_US
        Reading package lists... Done

    Next install gazebo-9 by:

        sudo apt-get install gazebo9
        # For developers that work on top of Gazebo, one extra package
        sudo apt-get install libgazebo9-dev

    If you see the error below:

        $ sudo apt-get install gazebo9
        Reading package lists... Done
        Building dependency tree
        Reading state information... Done
        E: Unable to locate package gazebo9

    It's possible the version of Gazebo you are looking for is not supported on the version of OS you are using.
    For example, installing gazebo9 on Ubuntu Jammy (22.04) will produce the error above.
    Hint: Take a look at "Project Status" section at [http://gazebosim.org/#status](http://gazebosim.org/#status),
    next to each version is the supported ubuntu versions and ROS versions.


1. Check your installation

    ***Note*** The first time `gazebo` is executed requires the download of some models and it could take some time, please be patient.

        gazebo
