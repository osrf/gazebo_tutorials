# Overview

The Random Velocity Plugin allows you to assign a random velocity to a link in the world.

You can set the maximum and minimum permissible values of the x,y and z velocity components, and the velocity magnitude.
Additionally, you can set the time period after which the velocity can change itself.
Once you have set the velocity magnitude, the direction is set on its own (by random selection).


# Prerequisites
[Hello World Plugin](http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin)

# Example
1. Change your current working directory to the location of the worlds folder in your Gazebo source (use the path to your Gazebo source).
2. Open random_velocity.world in your default editor.

~~~
$ cd path/worlds
$ gedit random_velocity.world
~~~

You will observe code like this : [gazebo/worlds/random_velocity.world](https://bitbucket.org/osrf/gazebo/src/gazebo6/worlds/random_velocity.world)

## Code Explained

The above code is just example usage of the Random Velocity Plugin.

First, we create a model and a link. In this case, our model is a cube of unit dimensions, which follows all kinematics rules.
The surface is frictionless, since the coefficient of friction (mu) is set to zero.
<include from='/<model/' to='/</link>/' src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/worlds/random_velocity.world' />

Now comes the actual usage of the plugin.
<include from='/<plugin name/' to='/</plugin>/' src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/worlds/random_velocity.world' />

1. Plugin name is "random" and its corresponding shared object file(.so) is ``libRandomVelocityPlugin.so``.
   .so files are dynamically linked at runtime.

2. You can apply an initial velocity to the link, using the initial\_velocity tag.

3. Velocity\_factor is the magnitude of new velocities that will be generated after each time period equal to the update time.
   Direction will be random but magnitude will remain constant.

4. Clamping indicates that a range is set. Maximum velocity in the y direction cannot exceed the max\_y value, and
   minumum velocity cannot be less than the min\_y value.

The default value for scale is 1, update time is 10, and the ``(min\_i, max\_i)`` is (-1.79769e+308,1.79769e+308).
1.79769e+308 is the maximum allowed value of float by ``Ignition math libraries``.

You can run the plugin using:

~~~
gazebo worlds/random_velocity.world
~~~

You can play with the values of ``initial velocity``, ``velocity_factor``,``update period`` etc., and observe how these values affect the simulation.

## Plugin Source

The source code for this plugin is available on [bitbucket](https://bitbucket.org/osrf/gazebo/src/gazebo6/plugins/).

If you have installed Gazebo from source then you can find this file where you downloaded the repository.

~~~
$ cd /YourPath/gazebo/plugins/RandomVelocityPlugin.cc
~~~

If you have installed from debian, then you can only locate header(.hh) and not (.cc).
You can try finding the location using,

~~~
$ locate RandomVelocityPlugin.hh
~~~

RandomVelocityPlugin.hh contains a commented example of usage (as included above) and function declarations of the functions defined in RandomVelocityPlugin.cc, which looks like [this](https://bitbucket.org/osrf/gazebo/src/gazebo6/plugins/RandomVelocityPlugin.cc).

## Understanding the source

<include from='/#include/' to='/"RandomVelocityPlugin.hh"/' src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/plugins/RandomVelocityPlugin.cc' />

RandomVelocityPluginPrivate.cc contains the private data pointer, in accordance with the [PIMPL idiom](http://gazebosim.org/tutorials?tut=contrib_code&cat=development#Style) implementation (opaque pointers).
The default initial values of all variables are set in it only.
All other #includes are necessary for various parts of code. For example:

1. <ignition/math/Rand.hh> for "ignition::math::Rand::DblUniform(-1, 1)"
		
2. <gazebo/common/Assert.hh> for "GZ_ASSERT"

3. <gazebo/physics/Model.hh> for "physics::ModelPtr _model"

~~~
using namespace gazebo;
~~~

To avoid writing gazebo repeatedly, before using Gazebo routines, structures or object classes.

~~~
GZ_REGISTER_MODEL_PLUGIN(RandomVelocityPlugin)
~~~

In the [hello world](http://gazebosim.org/tutorials/?tut=plugins_hello_world#HelloWorldPlugin!) tutorial we learned that plugins must be registered with the simulator using the GZ_REGISTER_WORLD_PLUGIN macro.

~~~
RandomVelocityPlugin::RandomVelocityPlugin()
  : dataPtr(new RandomVelocityPluginPrivate)
{
}

/////////////////////////////////////////////////
RandomVelocityPlugin::~RandomVelocityPlugin()
{
  delete this->dataPtr;
}
~~~

The first function is a constructor function that initializes the data objects using Private class RandomVelocityPluginPrivate.
The second function is a destructor function that deletes the pointer pointing to current/[this](http://gazebosim.org/tutorials?tut=contrib_code&cat=development#Style) instance's data.

<include from='/ void RandomVelocityPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)' to="will not fuction.\n";
    return;/' src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/plugins/RandomVelocityPlugin.cc' />

Similar to what we did in Hello World, we create function RandomVelocityPlugin::Load()
Parameters passed in are:

1. [physics::ModelPtr](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/namespacegazebo_1_1physics.html#ab9c6a161b32573a45586f808c39afe72): A model is a collection of links, joints, and plugins.

2. [sdf::ElementPtr](http://osrf-distributions.s3.amazonaws.com/gazebo/api/1.3.1/classsdf_1_1Element.html)

The function checks that ModelPtr is not null and sdf::ElementPtr [has the element](http://osrf-distributions.s3.amazonaws.com/gazebo/api/1.3.0/classsdf_1_1Element.html#aee65641faa3f98cf2c62e31fd4021b0a),link.

~~~
// Get x clamping values
  if (_sdf->HasElement("min_x"))
    this->dataPtr->xRange.X(_sdf->Get<double>("min_x"));
  if (_sdf->HasElement("max_x"))
    this->dataPtr->xRange.Y(_sdf->Get<double>("max_x"));
~~~

If min_x exists for _sdf then, xRange.X is set to min\_x.
The same goes for max\_y.
x/y/zRange.X indicate min value and x/y/zRange.Y indicate max value.
Their default values are set in RandomVelocityPluginPrivate.cc.

~~~
// Set the initial velocity, if present
  if (_sdf->HasElement("initial_velocity"))
  {
    this->dataPtr->velocity =
      _sdf->Get<ignition::math::Vector3d>("initial_velocity");
  }
// Set the velocity factor
  if (_sdf->HasElement("velocity_factor"))
    this->dataPtr->velocityFactor = _sdf->Get<double>("velocity_factor");

  // Set the update period
  if (_sdf->HasElement("update_period"))
    this->dataPtr->updatePeriod = _sdf->Get<double>("update_period");
~~~

You can learn about ignition::math::Vector3d [here](https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Vector3.html).
The other two are simple setter functions.

~~~
this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RandomVelocityPlugin::Update, this, std::placeholders::_1));
~~~

This updates the connection between the simulator and the world.
An [Event](http://osrf-distributions.s3.amazonaws.com/gazebo/api/1.9.1/classgazebo_1_1event_1_1Event.html) class is used to get notifications for simulator events.
[ConnectWorldUpdateBegin](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1event_1_1Events.html#a441fb0fe08d924ab99b7255215e7502e) takes in the subscriber to this event and returns a connection.
'bind' is a C++ standard function; it generates a forward calling wrapper. Calling bind is equivalent to invoking 'Update', with arguments as
placeholders::\_1.
The std::placeholders namespace contains the placeholder objects [_1, . . . _N] where N is an implementation-defined maximum number.

~~~
void RandomVelocityPlugin::Update(const common::UpdateInfo &_info)
{
  GZ_ASSERT(this->dataPtr->link, "<link> in RandomVelocity plugin is null");

  // Short-circuit in case the link is invalid.
  if (!this->dataPtr->link)
    return;

  // Change direction when enough time has elapsed
  if (_info.simTime - this->dataPtr->prevUpdate > this->dataPtr->updatePeriod)
  {
    // Get a random velocity value.
    this->dataPtr->velocity.Set(
        ignition::math::Rand::DblUniform(-1, 1),
        ignition::math::Rand::DblUniform(-1, 1),
        ignition::math::Rand::DblUniform(-1, 1));

    // Apply scaling factor
    this->dataPtr->velocity.Normalize();
    this->dataPtr->velocity *= this->dataPtr->velocityFactor;

    // Clamp X value
    this->dataPtr->velocity.X(ignition::math::clamp(this->dataPtr->velocity.X(),
        this->dataPtr->xRange.X(), this->dataPtr->xRange.Y()));

    // Clamp Y value
    this->dataPtr->velocity.Y(ignition::math::clamp(this->dataPtr->velocity.Y(),
        this->dataPtr->yRange.X(), this->dataPtr->yRange.Y()));

    // Clamp Z value
    this->dataPtr->velocity.Z(ignition::math::clamp(this->dataPtr->velocity.Z(),
        this->dataPtr->zRange.X(), this->dataPtr->zRange.Y()));

    this->dataPtr->prevUpdate = _info.simTime;
  }

  // Apply velocity
  this->dataPtr->link->SetLinearVel(this->dataPtr->velocity);
}
~~~

This is the update function invoked above. [UpdateInfo](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1common_1_1UpdateInfo.html#details) &_info primarily contain three pieces of information:

1. Real time (realTime)

2. Current simulation time (simTime)

3. Name of the world (worldName)

Other important functions and classes used are:

1. [DblUniform](https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Rand.html#aa27cd5f2f0b6271ae8cd9a8691d8b753) : Gets a random double value from a uniform distribution.

2. [Normalize()](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1math_1_1Vector3.html#afce261908c53f06a41a81752cdbfb373) : Normalizes the vector length by returning a unit length vector.

3. [ignition::math::vector3d](https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Vector3.html)

4. [Clamp](https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/namespaceignition_1_1math.html#a8a8c9d2bdc3f41ea0e71639b59b22a48)

5. [SetLinearVelocity](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Model.html#acb848e605587a69dfc0c5342905f1e3c)

You can modify the plugin by involving angular acceleration, linear acceleration, and you can have keep them random or fixed.
You can also play with the relative velocity of two links by making it constant, and applying random velocity to one link and to see how the velocity of the other link changes to maintain that relative velocity.
Or you may involve external force factors.



