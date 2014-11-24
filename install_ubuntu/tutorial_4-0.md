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
