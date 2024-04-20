# Install Gazebo-classic using Ubuntu packages

This tutorial covers the installation of Gazebo-classic packages using the
`packages.osrfoundation.org` repository, designed for those who want to follow the
faster development path but still a stable one.

Some notes:

  * Gazebo11 is also released as an **Ubuntu official package**:
    [check which version](https://packages.ubuntu.com/search?suite=all&section=all&arch=any&keywords=gazebo&searchon=sourcenames)
    is available for every distribution. The version in Ubuntu repositories
    rarely change for stability reasons.

  * Starting with Ubuntu Jammy (22.04) **Open Robotics won't be
    releasing more binary packages of Gazebo11 in `packages.osrfoundation.org`**
    for the new Ubuntu versions. [The migration to new Gz, formerly Ignition is recommended](https://community.gazebosim.org/t/a-new-era-for-gazebo/1356).
    Binaries coming directly from Ubuntu repositories can be used for these
    new platforms. The upstream code will keep accepting patches for new
    software versions so from-source installations will also be an option.

  * On Ubuntu Jammy (22.04) [Gazebo](https://packages.ubuntu.com/jammy/science/gazebo)
    binaries are only available for amd64. ROS uses Ubuntu official packages for Gazebo
    so there are only amd64 ROS packages. Packages for arm64. armhf and ppc64
    were released for the same version in Jammy in a
    [PPA https://launchpad.net/~openrobotics/+archive/ubuntu/gazebo11-non-amd64](https://launchpad.net/~openrobotics/+archive/ubuntu/gazebo11-non-amd64)
    but there is no official support nor updates for these binaries.

  * If you are a [ROS](http://ros.org) user, please read the tutorial about the
    [ROS/Gazebo-classic installation](/tutorials?tut=ros_wrapper_versions&cat=connect_ros).

## Default installation: one-liner

1. Install

        curl -sSL http://get.gazebosim.org | sh

2. Run

        gazebo

## Alternative installation: step-by-step

1. Setup your computer to accept software from packages.osrfoundation.org.

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

    You can check to see if the file was written correctly. For example, in Ubuntu Bionic (18.04), you can type:

        cat /etc/apt/sources.list.d/gazebo-stable.list

    And if everything is correct, you should see:

        deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main

1. Setup keys

        wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

1. Install Gazebo.

    First update the debian database:

        sudo apt-get update

    Hint: make sure the apt-get update process ends without any errors, the console output ends in `Done` similar to below:

        $ sudo apt-get update
        ...
        Hit http://ppa.launchpad.net bionic/main Translation-en
        Ign http://us.archive.ubuntu.com bionic/main Translation-en_US
        Ign http://us.archive.ubuntu.com bionic/multiverse Translation-en_US
        Ign http://us.archive.ubuntu.com bionic/restricted Translation-en_US
        Ign http://us.archive.ubuntu.com bionic/universe Translation-en_US
        Reading package lists... Done

    Next install gazebo-11 by:

        sudo apt-get install gazebo11
        # For developers that work on top of Gazebo, one extra package
        sudo apt-get install libgazebo11-dev

1. Check your installation

        gazebo
