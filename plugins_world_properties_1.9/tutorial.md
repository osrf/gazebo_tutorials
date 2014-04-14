#Tutorial: Programmatic World Control#

This plugin example programmatically modifies the gravity.

##Prerequisites: 

 * [Plugin overview tutorial](tutorials/1.9/plugins/overview).

## Setup

1. Create a working directory

        mkdir ~/world_edit; cd ~/world_edit

1. Create a file called `~/world_edit/world_edit.world`

        gedit world_edit.world

Add the following contents to it:

~~~
<?xml version ='1.0'?>
<sdf version ='1.4'>
  <world name='default'>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <plugin filename="libworld_edit.so" name="world_edit"/>
  </world>
</sdf>
~~~


## Code ##

1.  Create a file called `~/world_edit/world_edit.cc`:

~~~
gedit world_edit.cc
~~~

Add the following content to it:

~~~
#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  class WorldEdit : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->GetName());

      // Create a publisher on the ~/physics topic
      transport::PublisherPtr physicsPub =
        node->Advertise<msgs::Physics>("~/physics");

      msgs::Physics physicsMsg;
      physicsMsg.set_type(msgs::Physics::ODE);

      // Set the step time
      physicsMsg.set_max_step_size(0.01);

      // Change gravity
      msgs::Set(physicsMsg.mutable_gravity(), math::Vector3(0.01, 0, 0.1));
      physicsPub->Publish(physicsMsg);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldEdit)
}
~~~

### The Code Explained ###

~~~
// Create a new transport node
transport::NodePtr node(new transport::Node());

// Initialize the node with the world name
node->Init(_parent->GetName());
~~~

We create a new node pointer, and initialize it to using the world name. The world name allows the node to communicate with one specific world.

~~~
// Create a publisher on the ~/physics topic
transport::PublisherPtr physicsPub =
node->Advertise<msgs::Physics>("~/physics");
~~~

A publishers is created for sending physics messages on the "~/physics" topic.

~~~
msgs::Physics physicsMsg;
physicsMsg.set_type(msgs::Physics::ODE);

// Set the step time
physicsMsg.set_dt(0.01);

// Change gravity
msgs::Set(physicsMsg.mutable_gravity(), math::Vector3(0.01, 0, 0.01));
physicsPub->Publish(physicsMsg);
~~~

A physics message is created, and the step time and gravity are altered. This message is then published to the "~/physics" topic.

## Build ##
Create a CMake script called `~/world_edit/CMakeLists.txt`:


~~~
gedit CMakeLists.txt
~~~

Copy the following content into it:

~~~
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(world_edit SHARED world_edit.cc)
target_link_libraries(world_edit ${GAZEBO_libraries})
~~~

Create a `build` directory

~~~
mkdir build; cd build
~~~

Compile the plugin

~~~
cmake ../; make
~~~

## Run Tutorial ##

First you need to add the folder to the `GAZEBO_PLUGIN_PATH` environment variable:

~~~
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/world_edit/build/
~~~

Then in a terminal

~~~
cd ~/world_edit/build
gazebo ../world_edit.world
~~~

You should see an empty world.

Now add a box to the world using the Box icon located above the render window. The box should float up and away from the camera.
