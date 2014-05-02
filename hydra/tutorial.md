# Tutorial: Using the Razer Hydra

Gazebo supports the [Razer Hydra controller](http://en.wikipedia.org/wiki/Razer_Hydra). You will be able to use this motion and orientation detection controller to interact with your models in Gazebo.

## Razer Hydra configuration.

Create a file called `90-hydra.rules` with the following content:

~~~
ATTRS{idProduct}=="0300",ATTRS{idVendor}=="1532",ATTR{bInterfaceNumber}=="00",TAG="hydra-tracker"
SUBSYSTEM=="hidraw",TAGS=="hydra-tracker", MODE="0666", SYMLINK+="hydra"
~~~

We need to be able to access to the controller without root access.
~~~
sudo cp 90-hydra.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
~~~

## Gazebo compilation with Razer Hydra support.

We need to install the optional `libusb` dependency.

~~~
sudo apt-get install libusb-1.0-0-dev
~~~

Once Hydra is configured and the extra dependency satisfied, you should be able to compile Gazebo from source with Hydra support.

Follow [this](http://gazebosim.org/tutorials/?tut=install) instructions to compile Gazebo. During the execution of the `cmake` command, you should see this message confirming that the Oculus SDK is found:

~~~
-- Looking for libusb-1.0 - found. Razer Hydra support enabled.
~~~

## Using Hydra within Gazebo.

Using Hydra in Gazebo requires two steps. The first step is to load the Hydra plugin in your world file.

```
<plugin name="hydra" filename="libHydraPlugin.so">
  <pivot>0.04 0 0</pivot>
  <grab>0.12 0 0</grab>
</plugin>
```

This plugin will automatically publish messages on the topic `~/hydra`.

The second step is to write a plugin that subscribes to the hydra topic and make something interesting. For this
tutorial, we are going to move a sphere by using the right joystick of Hydra. A `HydraDemoPlugin` is available in Gazebo in the `plugins/` directory.

Plugin code:
~~~
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "HydraDemoPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(HydraDemoPlugin)

/////////////////////////////////////////////////
HydraDemoPlugin::HydraDemoPlugin()
{
}

/////////////////////////////////////////////////
HydraDemoPlugin::~HydraDemoPlugin()
{
}

/////////////////////////////////////////////////
void HydraDemoPlugin::OnHydra(ConstHydraPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->msgMutex);
  this->hydraMsgPtr = _msg;
}

/////////////////////////////////////////////////
void HydraDemoPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // Get the world name.
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Subscribe to Hydra updates by registering OnHydra() callback.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());
  this->hydraSub = this->node->Subscribe("~/hydra",
      &HydraDemoPlugin::OnHydra, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&HydraDemoPlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void HydraDemoPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  boost::mutex::scoped_lock lock(this->msgMutex);

  // Return if we don't have messages yet
  if (!this->hydraMsgPtr)
    return;

  // Read the value of the right joystick.
  double joyX = this->hydraMsgPtr->right().joy_x();
  double joyY = this->hydraMsgPtr->right().joy_y();

  // Move the sphere.
  this->model->SetLinearVel(math::Vector3(-joyX * 0.2, joyY * 0.2, 0));

  // Remove the message that has been processed.
  this->hydraMsgPtr.reset();
}
~~~

Before launching Gazebo, we need to include our model plugin in a world file.
Here is the complete world file of the tutorial (also available under
`worlds/hydra_demo.world`).

~~~
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">

    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Load the plugin for Razer Hydra -->
    <plugin name="hydra" filename="libHydraPlugin.so">
      <pivot>0.04 0 0</pivot>
      <grab>0.12 0 0</grab>
    </plugin>

    <!-- A sphere controlled by Hydra-->
    <model name="sphere">
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
        </visual>
      </link>

      <plugin name='sphere_controller' filename='libHydraDemoPlugin.so'>
      </plugin>

    </model>

</world>
</sdf>
~~~

It is time to run Gazebo and use the Hydra's right joystick to move the sphere.
Do not forget to plug your Hydra and them:

~~~
gazebo worlds/hydra_demo.world
~~~