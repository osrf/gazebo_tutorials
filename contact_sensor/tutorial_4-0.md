# Introduction

This tutorial demonstrates the process of creating a contact sensor, and
getting the contact data via a plugin or a message. A contact sensor detects
collisions between two object and reports the location of the contact
associated forces.

# Setup Tutorial

Start by creating a work directory

~~~
mkdir ~/gazebo_contact_tutorial; cd ~/gazebo_contact_tutorial
~~~

Next, make an SDF world file with a box that has a contact sensor.

~~~
gedit contact.world
~~~

Copy the following code into `contact.world`

~~~
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

     <include>
      <uri>model://sun</uri>
    </include>

    <model name="box">
      <link name="link">
        <pose>0 0 0.5 0 0 0</pose>

        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>

        <sensor name='my_contact' type='contact'>
          <contact>
            <collision>box_collision</collision>
          </contact>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
~~~

The contact sensor is attaced to a link within the box model. It will report collisions between the `box_collision` object and any other object in the world.

# Print Contact Values

Run the `contact.world` using Gazebo:

~~~
gazebo contact.world
~~~

In a separate terminal list the topics published by Gazebo

~~~
gz topic -l
~~~

The output should look like:

~~~
/gazebo/default/pose/info
/gazebo/default/gui
/gazebo/default/log/status
/gazebo/default/response
/gazebo/default/world_stats
/gazebo/default/selection
/gazebo/default/model/info
/gazebo/default/light
/gazebo/default/physics/contacts
/gazebo/default/visual
/gazebo/default/request
/gazebo/default/joint
/gazebo/default/sensor
/gazebo/default/box/link/my_contact
/gazebo/world/modify
/gazebo/default/diagnostics
/gazebo/default/factory
/gazebo/default/model/modify
/gazebo/default/scene
/gazebo/default/physics
/gazebo/default/world_control
/gazebo/server/control
~~~

The topic we are interested in is called `/gazebo/default/box/link/my_contact`. The `my_contact` contact sensor publishes on this topic.

Print the value of the contact sensors to the screen:

~~~
gz topic -e /gazebo/default/box/link/my_contact
~~~

The above command will dump the all the contacts to the terminal. You can stop this at anytime using `ctrl-c`.

NOTE: If this doesn't work, you may need to add

~~~
<update_rate> 5 </update_rate>
~~~

Between the `</contact>` and `</sensor>` tags in order to get output on the terminal. The rate "5" can be changed to a different frequency if desired.

# Contact Sensor Plugin

It is also possible to create a plugin for the contact sensor. This plugin can get the collision data, manipulate it, and output it to an arbitrary destination (for example a ROS topic).

*    **Note** This section of the tutorial requires you to compile a Gazebo plugin. For Gazebo version 3.0 and above, you will need to have the Gazebo dev packages installed. If you install Gazebo from source then you should already have the necessary files. If you install the Gazebo binary deb version, then you'll need to install a couple of additional packages.

    ~~~
    sudo apt-get install libgazebo[MAJOR VERSION NUMBER]-dev libsdformat2-dev
    ~~~
    
    where for example `MAJOR VERSION NUMBER` for gazebo4 is `4`.


Start by modifying the `contact.world` SDF file. Add the following line directly below `<sensor name='my_contact' type='contact'>`:

~~~
gedit contact.world
~~~

~~~
<plugin name="my_plugin" filename="libcontact.so"/>
~~~

This line tells Gazebo to load the `libcontact.so` sensor plugin. Which we will now define.

Create a header file for the plugin, call it `ContactPlugin.hh`:

~~~
gedit ContactPlugin.hh
~~~

and paste in the following content:

~~~
#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class ContactPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ContactPlugin();

    /// \brief Destructor.
    public: virtual ~ContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that recieves the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;
  };
}
#endif
~~~

Create a source file called `ContactPlugin.cc`:

~~~
gedit ContactPlugin.cc
~~~

and paste in the  following content:

~~~
#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->GetContacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std::cout << "Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";

    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
      std::cout << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
    }
  }
}
~~~

### The Code Explained

The following code from the `Load` function gets pointer to the contact sensor through the `_sensor` parameter. We then test to make sure the pointer is valid, and create a connection to the contact sensor's `updated` event. The last line guarantees that the sensor is initialized.

~~~
  // Get the parent sensor.
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
~~~

The `OnUpdate` function is called whenever the contact sensor is updated. In this function we print out the contact values.

~~~
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->GetContacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std::cout << "Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";

    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
      std::cout << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
    }
  }
}
~~~

## Compiling the code

Create a `CMakeLists.txt` file:

~~~
cd ~/gazebo_contact_tutorial; gedit CMakeLists.txt
~~~

Copy in the following code and save the file:

~~~
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(contact SHARED ContactPlugin.cc)
target_link_libraries(contact ${GAZEBO_libraries})
~~~

Next, create a build directory and make the plugin:

~~~
mkdir build; cd build; cmake ../; make
~~~

## Running the code

Enter the build directory

~~~
cd ~/gazebo_contact_tutorial/build
~~~

Run `gzserver`, first modifying your `LD_LIBRARY_PATH` so that the library loader can find your library (by default it will only look in certain system locations):

~~~
export LD_LIBRARY_PATH=~/gazebo_contact_tutorial/build:$LD_LIBRARY_PATH
gzserver ../contact.world
~~~
