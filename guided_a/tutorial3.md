# Workflow

On the previous tutorials you learned how to get your own copy of Gazebo
running on your computer. On this tutorial, we will go into a bit more
detail on how to navigate the code and make changes to it.

## Code structure

Let's understand a bit more how Gazebo's source code is organized. First,
move to the directory where you cloned the repository:

    cd ~/code/gazebo

These are some important contents of this directory which you should know about:

* `gazebo`: this directory is where you find all the header files (`.hh`) and
implementation files (`.cc`) which together form Gazebo. Unit tests for each class
also go here (`*_TEST.*`).
Inside this directory, we can see other directories, each one corresponding to a
library:

    * `physics`: has all classes related to physics. There are four
                 subdirectories here, one for each physics engine:
                 `ode`, `bullet`, `dart`, `simbody`.

    * `sensors`: has all classes related to sensors, like cameras and IMUs.

    * `rendering`: has all classes related to the 3D rendering, like scene,
                   visuals, materials...

    * `gui`: has all classes related to the GUI (Graphical User Interface),
             like menus and buttons.

    * `transport`: has all classes related to the transport layer.

    * `msgs`: has the description for all messages to be used with the
              transport layer.

    * `common, utils`: in these folders you find classes which are shared by
                       one or more of the other libraries.

* `test`: this is where all integration, regression and performance tests go.
Supporting data for the tests, such as worlds, meshes and plugins, also go here.

* `plugins`: this directory contains several plugins which are installed and
distributed with Gazebo.

* `worlds`: contains world files which are installed and distributed with
Gazebo. Note that there isn't an equivalent directory for models, as models are
hosted on a separate repository,
[gazebo_models](https://github.com/osrf/gazebo_models/).

* `examples`: contains example plugins and stand-alone programs which are not
installed with Gazebo, but can be used as references in tutorials.

* `cmake`: contains files used by cmake to build Gazebo. In particular, the
`SearchForStuff.cmake` file looks for dependencies.

* `doc`: contains files responsible for generating documentation web pages
from doxygen comments in C++ header files.

## Workflow example

You've previously built and run Gazebo-classic locally, but you've never made any changes
to the source code. Let's go through a simple example of how you'd change something
in the source code for the first time.

We will talk later about how to find an appropriate bug or feature to address.
For this example, let's pretend that we want to change the label which says
`Real Time Factor` on the interface to say `RTF` instead.

[[file:files/tut3_1.png|800px]]


1. Before making changes to the code, it's always a good idea to make sure you're
running the latest Gazebo-classic code. Let's move to the branch we want to target. Let's
say we want our change to be available from Gazebo-classic 9 onwards, so we target
the `gazebo9` branch. Use the `git checkout` command to update your workspace to
that branch:

        cd ~/code/gazebo
        git checkout gazebo9

1. Now let's "pull" the latest changes from the OSRF repository using the
`git pull` command:

        git pull https://github.com/osrf/gazebo

1. Now your local branch is in sync with the official repository. Let's build
and install Gazebo-classic before making any changes:

        cd build
        cmake ..
        make -j4
        sudo make install

1. Check that Gazebo-classic runs fine (it's a good idea to run Gazebo-classic in verbose mode
   to check if any errors happened):

        gazebo --verbose

1. If everything is in order, now you can start thinking about making the change.
It's a good idea to open a new terminal to browse through files, and keep the
terminal for building open for quick access. So go ahead and open a new terminal
and move to the source file folder:

        cd ~/code/gazebo

1. A good idea when jumping into a new codebase for the first time is to use the
`grep` tool to search for strings in files. Here, we can search for the string
we want to change, `Real Time Factor`. Let's use `grep` with the `-r` flag to
search recursively through all subdirectories within `gazebo` and the `-n` flag
to display line numbers:

        grep "Real Time Factor" -nr gazebo

1. You'll get a result similar to the following:

        gazebo/gui/TimeWidget.cc:137:  this->dataPtr->realTimeFactorLabel = new QLabel(tr("Real Time Factor:"));

1. The result is within `gazebo/gui`. That makes sense because we're
trying to modify something on the graphical interface. It also makes sense
that the line we're looking for is within `TimeWidget`, because it is
displayed within a widget that tells time. So let's open that file:

        gedit gazebo/gui/TimeWidget.cc

1. Then go to the line number found on the search (in this case 137), change
`Real Time Factor` to `RTF` and then save the file.

1. Now back at our build terminal, let's re-run commands to build and install:

        sudo make install

1. If everything went well, let's open Gazebo-classic from any terminal and check that the
label has been successfully changed:

        gazebo --verbose

[[file:files/tut3_2.png|800px]]

Congratulations, you're making changes to Gazebo!
