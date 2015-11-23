# Overview

Random Velocity Plugin allows you to assign random velocity to a link in a world.

You can set the maximum and minimum permissible values of x,y and z components of velocity and the magnitude of velocity.
Additionally, you can also set the time period after which velocity can change itself iteratively.
Once you have set the magnitude of velocity, direction is set on its own (by random selection).


# Prerequisites
[Hello World Plugin](http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin)

# Example
1. Change your current working directory to location of worlds folder in your gazebo source(path is the path to your Gazebo source).
2. Open random_velocity.world in your default editor.

~~~
$ cd path/worlds
$ gedit random_velocity.world
~~~

You will observe a code like this : [gazebo/worlds/random_velocity.world](https://bitbucket.org/osrf/gazebo/src/gazebo6/worlds/random_velocity.world)

## Code Explained

The above code is just an example of usage of Random Velocity Plugin.

First, we create a model and a link, in this case our model is a cube of unit dimensions, which follows all kinematics rules.
The surface is frictionless, since the coefficient of friction (mu) is set to zero.
<include from='/    <model/' to='/</link>/' src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/worlds/random_velocity.world' />

Now comes the actual usage of the plugin.
<include from='/    <plugin name/' to='/</plugin>/' src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/worlds/random_velocity.world' />

1. Plugin name is "random" and its corresponding shared object file(.so) is ``libRandomVelocityPlugin.so``.
   .so files are dynamically linked at runtime.

2. You can externally apply an initial velocity to the link, using initial\_velocity tag.

3. Velocity\_factor is the magnitude of new velocities which would be generated after each time period equal to update time.
   Direction would be random but magnitude would remain constant.

4. Clamping indicates that a range is set, maximum velocity in y direction cannot exceed the max\_y and
   minimum cannot be lesser than min\_y.

Default value for scale is 1, update time is 10 and the ``(min\_i, max\_i)`` is (-1.79769e+308,1.79769e+308).
1.79769e+308 is the maximum allowed value of float by ``Ignition math libraries``.

You can run it using

~~~
gazebo worlds/random_velocity.world
~~~

You can play with the values of ``initial velocity``, ``velocity_factor``,``update period`` etc and observe how that affects the simulation.

## Plugin Source

The source code of this plugin is available on [bitbucket](https://bitbucket.org/osrf/gazebo/src/gazebo6/plugins/).

If you have installed Gazebo from source then you can find this file where you downloaded the repository.

~~~
$ cd /YourPath/gazebo/plugins/RandomVelocityPlugin.cc
~~~

If you have installed from debian, then you can only locate header(.hh) and not (.cc).
You can try finding the location using,

~~~
$ locate RandomVelocityPlugin.hh
~~~

RandomVelocityPlugin.hh contains commented example of usage of this plugin(as we did above) and function declarations of the functions defined in RandomVelocityPlugin.cc, which looks like [this](https://bitbucket.org/osrf/gazebo/src/gazebo6/plugins/RandomVelocityPlugin.cc).

## Understanding the source

<include from='/#include/' to='/"RandomVelocityPlugin.hh"/' src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/plugins/RandomVelocityPlugin.cc' />

RandomVelocityPluginPrivate.cc contains the private data pointer, in accordance with the [PIMPL idiom](http://gazebosim.org/tutorials?tut=contrib_code&cat=development#Style) implementation(opaque pointers).
The default initial values of all variables are set in it only.
All other #includes are necessary for various parts of code for example

1. <ignition/math/Rand.hh> for "ignition::math::Rand::DblUniform(-1, 1)"

2. <gazebo/common/Assert.hh> for "GZ_ASSERT"

3. <gazebo/physics/Model.hh> for "physics::ModelPtr _model"

~~~
using namespace gazebo;
~~~

To avoid writing gazebo repeatedly,before all Gazebo API.

~~~
GZ_REGISTER_MODEL_PLUGIN(RandomVelocityPlugin)
~~~

In [hello world](http://gazebosim.org/tutorials/?tut=plugins_hello_world#HelloWorldPlugin!) tutorial we have learnt that plugin must be registered with the simulator using the GZ_REGISTER_WORLD_PLUGIN macro.

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

First one is the constructor function that, initializes the data objects using Private class RandomVelocityPluginPrivate.
Second one is the destructor function that deletes the pointer pointing to current/[this](http://gazebosim.org/tutorials?tut=contrib_code&cat=development#Style) instance's data.

<include from='/   void RandomVelocityPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)' to="will not fuction.\n";
    return;/' src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/plugins/RandomVelocityPlugin.cc' />

Similar to what we did in Hello World we create function RandomVelocityPlugin::Load()
Parameters passed in this are:

1. [physics::ModelPtr](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/namespacegazebo_1_1physics.html#ab9c6a161b32573a45586f808c39afe72): A model is a collection of links, joints, and plugins.

2. [sdf::ElementPtr](http://osrf-distributions.s3.amazonaws.com/gazebo/api/1.3.1/classsdf_1_1Element.html)

The function checks that ModelPtr is not null and sdf::ElementPtr [HasElement](http://osrf-distributions.s3.amazonaws.com/gazebo/api/1.3.0/classsdf_1_1Element.html#aee65641faa3f98cf2c62e31fd4021b0a),link.

<include from='/    // Get x clamping values/' to='this->dataPtr->xRange.Y(_sdf->Get<double>("max_x"));/' src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/plugins/RandomVelocityPlugin.cc' />

If min_x exists for _sdf then,xRange.X is set to min\_x.
similarly for max\_y.
x/y/zRange.X indicate min value and x/y/zRange.Y indicate max value.
Their default values are set in RandomVelocityPluginPrivate.cc.

<include from='/    // Set the initial velocity/' to='/_sdf->Get<double>("update_period");/' src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/plugins/RandomVelocityPlugin.cc' />

ignition::math::Vector3d can be understood from [here](https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Vector3.html).
The other two are simple setter functions.

~~~
this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RandomVelocityPlugin::Update, this, std::placeholders::_1));
~~~

This updates the connection of simulator with the world.
An [Event](http://osrf-distributions.s3.amazonaws.com/gazebo/api/1.9.1/classgazebo_1_1event_1_1Event.html) class is to get notifications for simulator events.
[ConnectWorldUpdateBegin](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1event_1_1Events.html#a441fb0fe08d924ab99b7255215e7502e) takes in the subscriber to this event and returns a connection.
'bind' is a c++ standard function, it generates a forward calling wrapper, calling bind is equivalent to invoking 'Update',with arguments as
placeholders::\_1.
The std::placeholders namespace contains the placeholder objects [_1, . . . _N] where N is an implementation defined maximum number.

<include from='/    void RandomVelocityPlugin/' to='/(this->dataPtr->velocity)/' src='https://bitbucket.org/osrf/gazebo/raw/gazebo6/plugins/RandomVelocityPlugin.cc' />

This is the update function invoked above, [UpdateInfo](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1common_1_1UpdateInfo.html#details) &_info primarily contain three information:

1. Real time (realTime)

2. Current simulation time (simTime)

3. Name of the world (worldName)

Other important functions and classes used are:

1. [DblUniform](https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Rand.html#aa27cd5f2f0b6271ae8cd9a8691d8b753) : Gets a random double value from a uniform distribution.

2. [Normalize()](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1math_1_1Vector3.html#afce261908c53f06a41a81752cdbfb373) : Normalizes the vector length by returning a unit lenght vector.

3. [ignition::math::vector3d](https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Vector3.html)

4. [Clamp](https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/namespaceignition_1_1math.html#a8a8c9d2bdc3f41ea0e71639b59b22a48)

5. [SetLinearVelocity](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Model.html#acb848e605587a69dfc0c5342905f1e3c)

You can try modifying the plugin by involving angular acceleration, linear acceleration and can have keep them random or fixed and see the results accordingly.
You can also play with relative velocity of two links by making it constant, and applying random velocity to one link and then see how the other functions to satisfy the constraints.
You can also involve external force factors.



