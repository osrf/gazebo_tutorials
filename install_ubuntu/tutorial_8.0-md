# Install Gazebo using Ubuntu packages

## Default installation: one-liner

1. Install

        curl -ssL http://get.gazebosim.org | sh

2. Run

        gazebo

## Alternative installation: step-by-step

1. Setup your computer to accept software from packages.osrfoundation.org.

    ***Note:*** there is a list of [available mirrors](https://bitbucket.org/osrf/gazebo/wiki/gazebo_mirrors) for this repository which could improve the download speed.

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

    You can check to see if the file was written correctly. For example, in Ubuntu Trusty, you can type:

        $ cat /etc/apt/sources.list.d/gazebo-stable.list
        deb http://packages.osrfoundation.org/gazebo/ubuntu-stable trusty main

1. Setup keys

        wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

1. Install Gazebo.

    First update the debian database:

        sudo apt-get update

    Hint: make sure the apt-get update process ends without any errors, the console output ends in `Done` similar to below:

        $ sudo apt-get update
        ...
        Hit http://ppa.launchpad.net trusty/main Translation-en
        Ign http://us.archive.ubuntu.com trusty/main Translation-en_US
        Ign http://us.archive.ubuntu.com trusty/multiverse Translation-en_US
        Ign http://us.archive.ubuntu.com trusty/restricted Translation-en_US
        Ign http://us.archive.ubuntu.com trusty/universe Translation-en_US
        Reading package lists... Done

    Next install gazebo-7 by:

        sudo apt-get install gazebo8
        # For developers that work on top of Gazebo, one extra package
        sudo apt-get install libgazebo8-dev

    If you see the error below:

        $ sudo apt-get install gazebo8
        Reading package lists... Done
        Building dependency tree
        Reading state information... Done
        E: Unable to locate package gazebo8

    It's possible the version of Gazebo you are looking for is not supported on the version of OS you are using.
    For example, installing gazebo8 on Ubuntu Trusty (14.04) will produce the error above.
    Hint: Take a look at "Project Status" section at [http://gazebosim.org/#status](http://gazebosim.org/#status), next to each version is the supported ubuntu versions and ROS versions.


1. Check your installation

    ***Note*** The first time `gazebo` is executed requires the download of some models and it could take some time, please be patient.

        gazebo

## Gazebo in different deb packages

Gazebo ships different Ubuntu debian packages following the [official packaging
guidelines](https://www.debian.org/doc/manuals/maint-guide/). This changes
brings an option about how to install gazebo:

 * Use Gazebo as an application: for the users that just run Gazebo simulator
   with the provided plugins and models and do not plan on developing on top of
   gazebo its own custom software. To use Gazebo 7.0, please install the package
   called ***gazebo8***.

 * Use Gazebo to develop software using Gazebo libraries: for users that
   develop plugins or any other kind of software that needs Gazebo headers and
   libraries. In this case, together with gazebo8 package, please install
   ***libgazebo8-dev***.
