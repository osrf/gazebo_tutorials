# Install Gazebo using Ubuntu packages

## One-line install

1. Install

        wget -O /tmp/gazebo6_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo6_install.sh; sudo sh /tmp/gazebo6_install.sh

2. Run

        gazebo

## Step-by-step Install

1. Setup your computer to accept software from packages.osrfoundation.org.

    ***Note:*** there is a list of [available mirrors](https://github.com/osrf/gazebo/wiki/gazebo_mirrors) for this repository which could improve the download speed.

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

1. Setup keys

        wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

1. Install Gazebo.

        sudo apt-get update
        sudo apt-get install gazebo6
        # For developers that work on top of Gazebo, one extra package
        sudo apt-get install libgazebo6-dev

1. Check your installation

    ***Note*** The first time `gazebo` is executed requires the download of some models and it could take some time, please be patient.

        gazebo

## Gazebo in different deb packages

Gazebo ships different Ubuntu debian packages following the [official packaging guidelines](https://www.debian.org/doc/manuals/maint-guide/). This changes brings an option about how to install gazebo:

 * Use Gazebo as an application: for the users that just run Gazebo simulator with the provided plugins and models and do not plan on developing on top of gazebo its own custom software. To use Gazebo 6.0, please install the package called ***gazebo6***.
 * Use Gazebo to develop software using Gazebo libraries: for users that develop plugins or any other kind of software that needs Gazebo headers and libraries. In this case, together with gazebo6 package, please install ***libgazebo6-dev***.
