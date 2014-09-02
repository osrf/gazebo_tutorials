# Introduction 

Gazebo topics communicate through Google protobuf messages. There is an
extensive [list of message types provided by Gazebo(http://osrf-distributions.s3.amazonaws.com/gazebo/msg-api/dev/classes.html), for use with subscribing and publishing Gazebo
topics. However, there are many situations where you want to build your own.

This tutorial is an example of how to create your own custom messages, and how to subscribe and publish them in a Gazebo plug-in.

## About this code

At the end of this project you should have a plug-in that can create a 2D
collision map of a Gazebo World. The plug-in will rasterize a world in
Gazebo use a RayShape to do ray intersection below this grid. It will
publish the created image to a custom topic and output to a file. The source
code for this plug-in lives at
[BitBucket](https://bitbucket.org/brawner/collision_map_creator_plugin).
Feel free to submit issues or pull-requests for improvements.

You can download the source code using mercurial:

<pre>
hg clone https://bitbucket.org/brawner/collision_map_creator_plugin
</pre>

or copy and paste the code into files as instructed below.

## Create your package

Create a package directory, and a msgs directory within.

<pre>
mkdir -p ~/collision_map_creator/msgs
</pre>

## Writing your own message

Writing your own message is great if it's too difficult to cram the
subscriber/publisher message into one of Gazebo's already available message
types. Furthermore, you can combine many different message types if you want
to make complicated messages. Gazebo contains a library of messages already.
The installed messages can be found in
/usr/include/gazebo-4.0/gazebo/msgs/proto for debian installs. This tutorial 
makes use of the
[vector2d.proto message](https://bitbucket.org/osrf/gazebo/src/8b0c2de0886a61a862036f7e7fe1d4b7b8b4651c/gazebo/msgs/vector2d.proto?at=gazebo_1.9).

Create our custom message by copying the following code into a file called `collision_map_request.proto` in `~/collision_map_creator/msgs/`

### Code ###

<pre>
gedit ~/collision_map_creator/msgs/collision_map_request.proto
</pre>

and paste in the following code:

~~~
package collision_map_creator_msgs.msgs;
import "vector2d.proto";

message CollisionMapRequest
{
  required gazebo.msgs.Vector2d   upperLeft  = 1;
  required gazebo.msgs.Vector2d   upperRight = 2;
  required gazebo.msgs.Vector2d   lowerRight = 3;
  required gazebo.msgs.Vector2d   lowerLeft  = 4;
  required double                 height     = 5;
  required double                 resolution = 6;
  optional string                 filename   = 7 [default = ""];
  optional int32                  threshold  = 8 [default = 255];
}
~~~

### Code explained ###

First we declare the namespace this message lives in, with the 'package' declaration.

~~~
package collision_map_creator_msgs.msgs;
~~~

We make use of gazebo's vector2d, so we must include that (the location of the messages will be taken care of in CMakeLists.txt).

~~~
import "vector2d.proto";
~~~

This is the actual message construct.

~~~
message CollisionMapRequest
{
}
~~~

Declare each message field with: 

<pre>
["optional"/"required"] [field type] [field name] = [enum];
</pre>

Each enum must be a unique number. Note also that the full package name must be used to specify external messages (gazebo.msgs in our case).
<pre>
  required gazebo.msgs.Vector2d   upperLeft  = 1;
  required gazebo.msgs.Vector2d   upperRight = 2;
  required gazebo.msgs.Vector2d   lowerRight = 3;
  required gazebo.msgs.Vector2d   lowerLeft  = 4;
  required double                 height     = 5;
  required double                 resolution = 6;
</pre>

Messages can contain optional fields. Optional fields can contain default values if they aren't specified. This is an example of how to do that.
<pre>
  optional string filename   = 7 [default = ""];
  optional int32  threshold  = 8 [default = 255];
</pre>

## CMakeLists for custom messages

Create a file called `CMakeLists.txt` and put that in `collision_map_creator/msgs/`

### Code ###

Open:

<pre>
gedit ~/collision_map_creator/msgs/CMakeLists.txt
</pre>

and paste in the following:

~~~
find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  collision_map_request.proto
  ${PROTOBUF_IMPORT_DIRS}/vector2d.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(collision_map_creator_msgs SHARED ${PROTO_SRCS})
target_link_libraries(collision_map_creator_msgs ${PROTOBUF_LIBRARY})
~~~

### Code explained

Tells CMake to include the Protobuf package

~~~
find_package(Protobuf REQUIRED)
~~~

This section finds where the messages are installed. The message files are stored in `/usr/include/gazebo-4.0/gazebo/msgs/proto` for debian installs, but the user may have installed elsewhere. This code searches for the `gazebo-X.X` folder and sets the `PROTOBUF_IMPORT_DIRS` variable accordingly. The `PROTOBUF_IMPORT_DIRS` specifies the location of extra message files.

~~~
set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()
~~~

Creates a list called `msgs` with the vector2d.proto message, its dependencies `time.proto` and `header.proto`, and the `collision_map_request.proto` message.

~~~
set (msgs
  collision_map_request.proto
  ${PROTOBUF_IMPORT_DIRS}/vector2d.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)
~~~

Build the necessary C++ headers and source files for our messages.

~~~
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
~~~

Add this message library so that we can link to it from other projects. The
library name needs to be the same as the library name specified in the other
`CMakeLists.txt` further below.

~~~
add_library(collision_map_creator_msgs SHARED ${PROTO_SRCS})
target_link_libraries(collision_map_creator_msgs ${PROTOBUF_LIBRARY})
~~~

## Collision Map Creator Plugin

This is the code for the custom Gazebo world plugin.

### Code

Copy the following code into a file called `collision_map_creator.cc` and put in the `~/collision_map_creator/` directory

<pre>
gedit ~/collision_map_creator/collision_map_creator.cc
</pre>

~~~
#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "collision_map_request.pb.h"


namespace gazebo
{
typedef const boost::shared_ptr<const collision_map_creator_msgs::msgs::CollisionMapRequest> CollisionMapRequestPtr;
class CollisionMapCreator : public WorldPlugin
{
    transport::NodePtr node;
    transport::PublisherPtr imagePub;
    transport::SubscriberPtr commandSubscriber;
    physics::WorldPtr world;

public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        node = transport::NodePtr(new transport::Node());
        world = _parent;
        // Initialize the node with the world name
        node->Init(world->GetName());
        std::cout << "Subscribing to: " << "~/collision_map/command" << std::endl;
        commandSubscriber = node->Subscribe("~/collision_map/command", &CollisionMapCreator::create, this);
        imagePub = node->Advertise<msgs::Image>("~/collision_map/image");
    }

public: void create(CollisionMapRequestPtr &msg)
    {
        std::cout << "Received message" << std::endl;

        std::cout << "Creating collision map with corners at (" <<
                     msg->upperleft().x() << ", " << msg->upperleft().y() << "), (" <<
                     msg->upperright().x() << ", " << msg->upperright().y() << "), (" <<
                     msg->lowerright().x() << ", " << msg->lowerright().y() << "), (" <<
                     msg->lowerleft().x() << ", " << msg->lowerleft().y() << ") with collision projected from z = " <<
                     msg->height() << "\nResolution = " << msg->resolution() << " m\n" <<
                     "Occupied spaces will be filled with: " << msg->threshold() << std::endl;
        // double z = msg->height();
        double dX_vertical = msg->upperleft().x() - msg->lowerleft().x();
        double dY_vertical = msg->upperleft().y() - msg->lowerleft().y();
        double mag_vertical = sqrt(dX_vertical * dX_vertical + dY_vertical * dY_vertical);
        dX_vertical = msg->resolution() * dX_vertical / mag_vertical;
        dY_vertical = msg->resolution() * dY_vertical / mag_vertical;

        // double step_s = msg->resolution();

        double dX_horizontal = msg->upperright().x() - msg->upperleft().x();
        double dY_horizontal = msg->upperright().y() - msg->upperleft().y();
        double mag_horizontal = sqrt(dX_horizontal * dX_horizontal + dY_horizontal * dY_horizontal);
        dX_horizontal = msg->resolution() * dX_horizontal / mag_horizontal;
        dY_horizontal = msg->resolution() * dY_horizontal / mag_horizontal;



        int count_vertical = mag_vertical / msg->resolution();
        int count_horizontal = mag_horizontal / msg->resolution();

        if (count_vertical == 0 || count_horizontal == 0)
        {
            std::cout << "Image has a zero dimensions, check coordinates" << std::endl;
            return;
        }
        double x,y;

        boost::gil::gray8_pixel_t fill(255-msg->threshold());
        boost::gil::gray8_pixel_t blank(255);
        boost::gil::gray8_image_t image(count_horizontal, count_vertical);

        double dist;
        std::string entityName;
        math::Vector3 start, end;
        start.z = msg->height();
        end.z = 0.001;

        gazebo::physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
        engine->InitForThread();
        gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
              engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

        std::cout << "Rasterizing model and checking collisions" << std::endl;
        boost::gil::fill_pixels(image._view, blank);

        for (int i = 0; i < count_vertical; ++i)
        {
            std::cout << "Percent complete: " << i * 100.0 / count_vertical << std::endl;
            x = i * dX_vertical + msg->lowerleft().x();
            y = i * dY_vertical + msg->lowerleft().y();
            for (int j = 0; j < count_horizontal; ++j)
            {
                x += dX_horizontal;
                y += dY_horizontal;

                start.x = end.x= x;
                start.y = end.y = y;
                ray->SetPoints(start, end);
                ray->GetIntersection(dist, entityName);
                if (!entityName.empty())
                {
                    image._view(i,j) = fill;
                   // std::cout << ".";
                }
            }
            //std::cout << "|" << std::endl;
        }
        //std::cout << std::endl;

        std::cout << "Completed calculations, writing to image" << std::endl;
        if (!msg->filename().empty())
        {
            boost::gil::gray8_view_t view = image._view;
            boost::gil::png_write_view(msg->filename(), view);
        }
    }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CollisionMapCreator)
}
~~~

### Code Explained


The necessary system headers to include

~~~
#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
~~~

These are the necessary Gazebo headers we'll need.

~~~
#include "gazebo.hh"
#include "common/common.hh"
#include "math/Vector3.hh"
#include "transport/transport.hh"
#include "physics/physics.hh"
#include "sdf/sdf.hh"
#include "msgs/msgs.hh"
~~~

In order to make use of our collision_map_request message, we need to include it's generated header file.

~~~
#include "collision_map_request.pb.h"
~~~

These are used to write a .png file.

~~~
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
~~~

The plugin lives in the gazebo namespace

~~~
namespace gazebo
{
~~~

This is the object that will be sent to our callback method. It's convenient to create a typedef for it.

~~~
typedef const boost::shared_ptr<const collision_map_creator_msgs::msgs::CollisionMapRequest> CollisionMapRequestPtr;
~~~

Creating a derived class of the Gazebo WorldPlugin. It needs to hold on to the NodePtr, PublisherPtr, SubcriberPtr and WorldPtr, so they must be added as class variables.

~~~
class CollisionMapCreator : public WorldPlugin
{
    transport::NodePtr node;
    transport::PublisherPtr imagePub;
    transport::SubscriberPtr commandSubscriber;
    physics::WorldPtr world;
~~~

The World Plugin's Load method. It sets up the node, the world, the subscriber and the publisher.

~~~
public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        node = transport::NodePtr(new transport::Node());
        world = _parent;
        // Initialize the node with the world name
        node->Init(world->GetName());
        std::cout << "Subscribing to: " << "~/collision_map/command" << std::endl;
~~~

The node->Subscribe is called with the topic, callback function, and a pointer to this plugin object. The node->Advertise, requires a template argument of the message type, and a parameter of the topic to advertise.

~~~
        commandSubscriber = node->Subscribe("~/collision_map/command", &CollisionMapCreator::create, this);
        imagePub = node->Advertise<msgs::Image>("~/collision_map/image");
~~~

This is the callback method. It contains the typedef we used earlier. The callback is passed a reference, so be sure to use the &msg.

~~~
public: void create(CollisionMapRequestPtr &msg)
~~~

The CollisionMapRequestPtr is a boost shared_ptr, so to get the actual fields you need to use a structure de-reference ('->') instead of the structure reference ('.') like I have below. To get the fields of an object, you need to call its get function (e.g. field())

~~~
        double z = msg->height();
~~~

But wait you ask, why can you directly reference a subfield? That's because they aren't shared_ptrs inside, the message we were sent is.

~~~
        double dX_vertical = msg->upperleft().x() - msg->lowerleft().x();
        double dY_vertical = msg->upperleft().y() - msg->lowerleft().y();
        double mag_vertical = sqrt(dX_vertical * dX_vertical + dY_vertical * dY_vertical);
        dX_vertical = msg->resolution() * dX_vertical / mag_vertical;
        dY_vertical = msg->resolution() * dY_vertical / mag_vertical;
~~~


This is an example of how to call the used physics engine to do a single ray trace. This code was pulled from gazebo::physics::World::GetEntityBelowPoint(Vector3)

~~~
        gazebo::physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
        engine->InitForThread();
        gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
              engine->CreateShape("ray", gazebo::physics::CollisionPtr()));
~~~


Rasterizing over the grid to determine where there are objects and where there is not.

~~~
        for (int i = 0; i < count_vertical; ++i)
        {
...
            for (int j = 0; j < count_horizontal; ++j)
            {
...
            }
...
        }
~~~

This creates an Image msg and fills the necessary fields. Notice that the setter methods for each field is formatted like 'set_field(something)'

~~~
        msgs::Image image;
        image.set_width(count_horizontal);
        image.set_height(count_vertical);
        image.set_pixel_format(0);
        image.set_step(count_horizontal);
        image.set_data(data);

        imagePub->Publish(image);
~~~

Register the plugin with the simulator.

~~~
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CollisionMapCreator)
}
~~~

## Request Publisher Executable

This executable lives outside of gazebo, but it shows how to pull in the
required libraries from Gazebo and publish to gazebo topics with a custom
built message type. There are some extra steps that must be taken that
aren't explained in plugin tutorials.

### Code

Copy the following code into a file called `request_publisher.cc` and put in
the `~/collision_map_creator/` directory

<pre>
gedit ~/collision_map_creator/request_publisher.cc
</pre>

~~~
#include <iostream>
#include <math.h>
#include <deque>
#include <sdf/sdf.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"

#include "collision_map_request.pb.h"
#include "vector2d.pb.h"

using namespace std;

bool createVectorArray(const char * vectorString,
                       deque<gazebo::msgs::Vector2d*> corners)
{
    deque<gazebo::msgs::Vector2d*>::iterator it;

    string cornersStr = vectorString;
    size_t opening=0;
    size_t closing=0;
    for (it = corners.begin(); it != corners.end(); ++it)
    {
        opening = cornersStr.find('(', closing);
        closing = cornersStr.find(')', opening);
        if (opening == string::npos || closing == string::npos)
        {
            std::cout << "Poorly formed string: " << cornersStr << std::endl;
            std::cout << "( found at: " << opening << " ) found at: " << closing << std::endl;
            return false;
        }
        string oneCornerStr = cornersStr.substr(opening + 1, closing - opening - 1);
        size_t commaLoc = oneCornerStr.find(",");
        string x = oneCornerStr.substr(0,commaLoc);
        string y = oneCornerStr.substr(commaLoc + 1, oneCornerStr.length() - commaLoc);
        (*it)->set_x(atof(x.c_str()));
        (*it)->set_y(atof(y.c_str()));
    }
    return true;
}

int main(int argc, char * argv[])
{
    if (argc > 4)
    {
        collision_map_creator_msgs::msgs::CollisionMapRequest request;
        deque<gazebo::msgs::Vector2d*> corners;

        corners.push_back(request.mutable_upperleft());
        corners.push_back(request.mutable_upperright());
        corners.push_back(request.mutable_lowerright());
        corners.push_back(request.mutable_lowerleft());

        if (!createVectorArray(argv[1],corners))
        {
            return -1;
        }

        request.set_height(atof(argv[2]));
        request.set_resolution(atof(argv[3]));
        request.set_filename(argv[4]);

        if (argc == 6)
        {
            request.set_threshold(atoi(argv[6]));
        }

        gazebo::transport::init();
        gazebo::transport::run();
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init("default");

        std::cout << "Request: " <<
                     " UL.x: " << request.upperleft().x() <<
                     " UL.y: " << request.upperleft().y() <<
                     " UR.x: " << request.upperright().x() <<
                     " UR.y: " << request.upperright().y() <<
                     " LR.x: " << request.lowerright().x() <<
                     " LR.y: " << request.lowerright().y() <<
                     " LL.x: " << request.lowerleft().x() <<
                     " LL.y: " << request.lowerleft().y() <<
                     " Height: " << request.height() <<
                     " Resolution: " << request.resolution() <<
                     " Filename: " << request.filename() <<
                     " Threshold: " << request.threshold() << std::endl;

        gazebo::transport::PublisherPtr imagePub =
                node->Advertise<collision_map_creator_msgs::msgs::CollisionMapRequest>(
                                                            "~/collision_map/command");
        imagePub->WaitForConnection();
        imagePub->Publish(request);

        gazebo::transport::fini();
        return 0;
    }
    return -1;
}
~~~

### Code Explained ###

Including necessary standard c++ headers

~~~
#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
~~~

These are the necessary gazebo headers for our executable.

~~~
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "collision_map_request.pb.h"
~~~

This is the header associated with our custom message types.

~~~
#include "collision_map_request.pb.h"
~~~

This function parses a string of coordinates as defined by "(x1,y1)(x2,y2)(x3,y3)(x4,y4)" into a deque of Vector2d messages.

~~~
bool createVectorArray(const char * vectorString, 
                       deque<gazebo::msgs::Vector2d*> corners)
~~~

Initialize the deque iterator.

~~~
    deque<gazebo::msgs::Vector2d*>::iterator it;
~~~

Iterate through the corners deque

~~~
...
    for (it = corners.begin(); it != corners.end(); ++it)
    {
...
    }
~~~

For this corner Vector2d, we set the X and Y by converting the found string for each x and y into a float.

~~~
        (*it)->set_x(atof(x.c_str()));
        (*it)->set_y(atof(y.c_str()));
~~~

The main function, which is called when the executable is run.

~~~
int main(int argc, char * argv[])
{
...
}
~~~

This executable requires a number of arguments, only proceed if we at least four.

~~~
    if (argc > 4)
    {
...
    }
~~~ 

Create a CollisionMapRequest message. We need to fill this message before sending it. Also, we initialize a deque of Vector2d messages, so we can pass them to the createVectorArray function.

~~~
        collision_map_creator_msgs::msgs::CollisionMapRequest request;
        deque<gazebo::msgs::Vector2d*> corners;
~~~


If a message field is simple, (double, int32, string) it can be set directly by its associated 'set_field()' method. If the field is a message type itself, it must be accessed through its mutable_field() method. Because the Vector2d is itself a separate message, we must first get the pointer from the mutable function call, like we have here. 

~~~
        corners.push_back(request.mutable_upperleft());
        corners.push_back(request.mutable_upperright());
        corners.push_back(request.mutable_lowerright());
        corners.push_back(request.mutable_lowerleft());
~~~

Pass the deque of Vector2d pointers and the first argv string to the createVectorArray. If it fails, then the input was probably formatted incorrectly. Exit the program in this case.

~~~
        if (!createVectorArray(argv[1],corners))
        {
            return -1;
        }
~~~

For the simple fields, we can set their values directly by calling their set_field methods.

~~~
        request.set_height(atof(argv[2]));
        request.set_resolution(atof(argv[3]));
        request.set_filename(argv[4]);
~~~

For our optional threshold field, we set that if it was specified in the command arguments.

~~~
        if (argc == 6)
        {
            request.set_threshold(atoi(argv[6]));
        }
~~~

This is a super important part of the main method. This initializes the gazebo transport system. For plugins, Gazebo already takes care of this, but for our own executable we have to do it ourself. Also notice we're not using the gazebo namespace so we must be explicit.

~~~
        gazebo::transport::init();
        gazebo::transport::run();
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init("default");
~~~

This initializes the Request Publisher on the topic "~/collision_map/command".

~~~
        gazebo::transport::PublisherPtr imagePub =
                node->Advertise<collision_map_creator_msgs::msgs::CollisionMapRequest>(
                                                            "~/collision_map/command");
~~~

To publish our message, we must first wait for the publisher to be connected to the master. Then we can directly publish our message.

~~~
        imagePub->WaitForConnection();
        imagePub->Publish(request);
~~~

Before exiting, the program must call transport::fini() to tear it all down.

~~~
        gazebo::transport::fini();
~~~


## CMakeLists for Plugin and Executable

Create a `CMakeLists.txt` in `~/collision_map_creator/` directory with the
following code

### Code 

<pre>
gedit ~/collision_map_creator/CMakeLists.txt
</pre>

~~~
# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )
link_directories(${GAZEBO_LIBRARY_DIRS} ${SDF_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_executable (request_publisher request_publisher.cc)
target_link_libraries(request_publisher collision_map_creator_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(request_publisher collision_map_creator_msgs)

add_library(collision_map_creator SHARED collision_map_creator.cc )
target_link_libraries(collision_map_creator collision_map_creator_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(collision_map_creator collision_map_creator_msgs)
~~~

### Code Explained

Boost is required for in our callback function in the plug-in. Not to mention, it's also used for the image writer.

~~~
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
~~~

The msgs directory needs to be added to the include directories because we use them in both the executable and the plugin.

~~~
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )
~~~

Besides the standard GAZEBO_LIBRARY_DIRS, we also need to link against the msgs directory. The add_subdirectory directive tells CMake to look in it for a CMakeLists.txt file.

~~~
link_directories(${GAZEBO_LIBRARY_DIRS} ${SDF_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)
~~~

Add the executable request_publisher to the makefile. We must link it against the collision_map_creator_msgs, the GAZEBO_LIBRARIES and Boost_LIBRARIES. The add_dependencies directive tells CMake to build collision_map_creator_msgs first.

~~~
add_executable (request_publisher request_publisher.cc)
target_link_libraries(request_publisher collision_map_creator_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(request_publisher collision_map_creator_msgs)
~~~

This builds our plugin, and is very similar to your standard WorldPlugin CMakeLists.txt, except we must link it against the collision_map_creator_msgs and add that as a dependency as well.

~~~
add_library(collision_map_creator SHARED collision_map_creator.cc )
target_link_libraries(collision_map_creator collision_map_creator_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(collision_map_creator collision_map_creator_msgs)
~~~

## Build and Deploy ##

To build this project, create a build directory, run CMake and then make.

<pre>
cd ~/collision_map_creator
mkdir build
cd build
cmake ../
make
</pre>

This plugin needs to be added to the gazebo plugin path. You can either change your environment variable to refer to this build directory,

<pre>
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:collision_map_creator/build
</pre>

or you can copy the plugin to your plugin directory.

<pre>
sudo cp libcollision_map_creator.so /usr/lib/gazebo-1.9/plugins/
</pre>

## Running

Assuming everything went fine, and that's probably a rough assumption
(seriously this one was long), you need create a Gazebo world to reference
your plugin. Create a world file

<pre>
gedit ~/collision_map_creator/map_creator.world
</pre>

and paste in the following:

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

    <include>
      <uri>model://willowgarage</uri>
    </include>

    <plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>
  </world>
</sdf>
~~~

Run Gazebo with this world

<pre>
gazebo ~/collision_map_creator/map_creator.world
</pre>

In a separate terminal, run the executable and tell it to build a 20m x 20m map with a 1cm resolution.

<pre>
~/collision_map_creator/build/request_publisher "(-10,10)(10,10)(10,-10)(-10,-10)" 10 0.01 ~/map.png
</pre>

Your executable terminal should show it connecting to Gazebo and display the
request message. You should see your gazebo terminal display some messages
and run a percent complete stat until it finishes. Your map.png should look
like this below.

[[file:files/map.png|800px]]

