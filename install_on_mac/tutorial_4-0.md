# Install gazebo on Mac (using homebrew)

## Step-by-step Install

Gazebo and several of its dependencies can be compiled on OS X with 
[Homebrew](http://brew.sh) using the 
[osrf/simulation tap](https://github.com/osrf/homebrew-simulation). 
Here are the instructions:

1. Install [homebrew](http://brew.sh)
	ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

2. Install [XQuartz](http://xquartz.macosforge.org/landing/), which provides 
X11 support and is required by Gazebo and OGRE

3. For 10.8 and earlier, install [Xcode command-line tools](http://stackoverflow.com/questions/9329243/xcode-4-4-and-later-install-command-line-tools)
by downloading them from Apple. For 10.9 and later, they should prompt you to
install them when you install Homebrew in step 1.

4. Run the following commands:

        brew tap osrf/simulation
        brew install gazebo4
        gazebo

## Optional dependencies
The gazebo formula has two optional dependencies: the [Bullet](https://code.google.com/p/bullet/) and [Simbody](https://github.com/simbody/simbody) physics engines. To install with these physics engines:

        brew install gazebo4 --with-bullet --with-simbody

## Versions
The formula currently installs version 4.0 of gazebo.
Version 1.9 can be installed using the gazebo formula, gazebo 2.2 using gazebo2, and gazebo 3 using gazebo3.
To install the latest version of gazebo's default branch:

        brew install gazebo4 --HEAD
