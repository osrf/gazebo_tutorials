# Run your own copy of Gazebo

On the [previous tutorial](/tutorials?tut=guided_a1)
we covered where to find the source code for Gazebo and other dependencies.

This tutorial will go through the process of getting your own copy of the
code running.

## Fork it!

We previously showed that Gazebo's source code lives on a git
[repository](https://github.com/osrf/gazebo) on GitHub. But even though
everyone in the world is able to see and copy that code, only the Gazebo core
team has write access to it.

In order to modify the code, you'll need to get your own copy, which is called
a "fork". You can fork Gazebo as follows:

1. Click on [this link](https://github.com/osrf/gazebo/fork)
1. You can choose a custom name for the repository, but here we will leave the
default value `gazebo`.
1. After you finish the fork process, you should have a copy of Gazebo on
`https://github.com/<yourname>/gazebo`.

> **Note**: Throughout these tutorials, substitute `<yourname>` with your
GitHub account username.

## Clone

Great, now you have a copy of the code, but it's not very convenient to
interact with it through the browser. You want to have it in your computer.
You will use the git command line tool to pull that code from the internet
to your computer as follows:

1. Make sure you have git installed:

        sudo apt update
        sudo apt install git

1. It's a good idea to create a directory to hold the source code, so:

        mkdir ~/code
        cd ~/code

1. Now we use git to "clone" our fork. What the clone command does is
copy all the code across all branches from the internet to your computer.
Gazebo has a large codebase, so this process may take a while depending on
your internet connection:

        git clone https://github.com/<yourname>/gazebo

1. Now you should have a local copy of Gazebo under `~/code/gazebo`. Let's
move to that folder and list its contents:

        cd ~/code/gazebo
        ls

1. You should see something like this:

    [[file:files/tut2_1.png|800px]]

## Branches

Gazebo's code is organized into different branches with different purposes.

1. Let's take a look at all existing branches using the git command
"branches":

        cd ~/code/gazebo
        git branch -a

1. You'll see a long list which looks something like this:

        remotes/origin/2.1_abi_compat
        remotes/origin/2014_copyright
        remotes/origin/90_windows_patch_test
        remotes/origin/DoNotLaunchMarkerManagerInServer
        remotes/origin/FixODERevoluteJointInitialization
        ...

1. Branches prefixed with `remotes/origin` come from the official OSRF repository.

Most of the branches in Gazebo are branches where the core team is working
on fixing bugs or adding new features. But a few branches have special meaning,
these are:

* `master`: This is the bleeding edge code where all new features are being
developed. You're automatically on this branch when you clone Gazebo. This
is where new features and code that is incompatible with previous releases
(i.e. breaks API/ABI) will go. Since Gazebo 11 was the last release, the
`master` branch no longer exists.

* `gazebo<N>`: Here, `N` is a number representing a Gazebo release. For example,
the code for the latest release of Gazebo 7 is found on branch `gazebo7`.

## Build

Cool, now we have all the code, let's build our own copy of Gazebo!

> **Note**: This tutorial goes over the most basic installation. If you need
some special configuration, check out the full
[install from source tutorial](/tutorials?tut=install_from_source&cat=install).

1. Setup your computer to accept software from packages.osrfoundation.org.

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

1. Setup keys and update

        wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
        sudo apt-get update

1. Install dependencies

        wget https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
        ROS_DISTRO=dummy . /tmp/dependencies.sh
        sudo apt-get install $(sed 's:\\ ::g' <<< $BASE_DEPENDENCIES) $(sed 's:\\ ::g' <<< $GAZEBO_BASE_DEPENDENCIES)

1. Make sure you're at the source code root directory:

        cd ~/code/gazebo

1. Make a build directory and go there

        mkdir build
        cd build

1. Configure and build. This will take a while (easily more than one hour),
leave it running and go watch some cool
[Gazebo videos](https://www.youtube.com/results?search_query=gazebo+simulator).

        cmake ..
        make -j4

1. Once that's done, install Gazebo:

        sudo make install

1. Now you can try your installation:

        gazebo --verbose

## Check the installation

If you've installed Gazebo on your system before, you might be asking
how do you know if you're running your own copy of Gazebo, or the one you
had previously installed. A quick trick to figure that out is to:

1. Check where you're running Gazebo from:

        which gazebo

1. This will give you something like:

        /usr/local/bin/gazebo

1. Now check where you're installing Gazebo to. You can do this by re-running
install and looking for the install location, for example:

        cd ~/code/gazebo/build
        sudo make install | grep /gazebo$

1. You'll see something like:

        -- Up-to-date: /usr/local/bin/gazebo

1. If the paths from both commands match, you're running your own copy!

