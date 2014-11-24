# Install Gazebo using Ubuntu packages

## One-line install

1. Install

        wget -O /tmp/gazebo4_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo4_install.sh; sudo sh /tmp/gazebo4_install.sh

2. Run

        gazebo

## Step-by-step Install

1. Setup your computer to accept software from packages.osrfoundation.org.
    
    ***Note:*** there is a list of [available mirrors](https://bitbucket.org/osrf/gazebo/wiki/gazebo_mirrors) for this repository which could improve the download speed.

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'

1. Setup keys

        wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

1. Install Gazebo.

        sudo apt-get update
        sudo apt-get install gazebo4
        # For developers that works on top of Gazebo, one extra package
        sudo apt-get install libgazebo4-dev

1. Check your installation

    ***Note*** The first time `gazebo` is executed requires the download of some models and it could take some time, please be patient.

        gazebo

## Gazebo in different deb packages

Gazebo ships different Ubuntu debian packages following the [official packaging guidelines](https://www.debian.org/doc/manuals/maint-guide/). This changes brings an option about how to install gazebo:

 * Use Gazebo as an application: for the users that just run Gazebo simulator with the provided plugins and models and do not plan on developing on top of gazebo its own custom software. To use Gazebo 4.0, please install the package called ***gazebo4***.
 * Use Gazebo to develop software using Gazebo libraries: for users that develop plugins or any other kind of software that needs Gazebo headers and libraries. In this case, together with gazebo4 package, please install ***libgazebo4-dev***. 

# Gazebo multi physics engines support

Gazebo is able to use different physics engines to perform the simulation.

[ODE](www.ode.org) is the one used by default, but support is in also in place for 
[Bullet](http://bulletphysics.org) and [Simbody](https://simtk.org/home/simbody/)
using Ubuntu packages. For those users that wan to try [DART](http://dartsim.github.io/), 
a from source installation of gazebo is needed, so please do not use the .deb packages.

## Gazebo 4 in ROS

Please follow the [Connect to ROS](http://gazebosim.org/tutorials?cat=connect_ros) tutorials for using Gazebo with ROS.
