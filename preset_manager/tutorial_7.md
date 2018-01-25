This tutorial describes how to use the Preset Manager interface to store
physics parameter profiles and switch between them.

# Motivation
Gazebo has many parameters that affect the performance, accuracy, and general
behavior of physics simulation. Some are shared between the different physics
engines supported by Gazebo, like maximum step size and target real time
factor, and some are not. The physics preset manager interface offers a way to
easily switch between a set of physics parameters and save them to SDF.

# Usage

## SDF
In SDF, a physics profile is simply a `<physics>` element. As of SDF version 3,
multiple physics elements are allowed in a world file, but they must be
differentiated by the `name` attribute. When there are multiple physics elements
specified, Gazebo will choose the one with the `default` attribute set to true.
If no default physics profile is set, Gazebo will choose the first one. If
multiple default profiles are set, Gazebo will choose the first set as default.

In the following world example
([downloadable here](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/preset_manager/files/preset_example_sdf1_6.world)),
the `ode_200iters` profile is set as the default, and the `ode_70iters`
profile will be available via the C++ API or the `gz` command line tool. The following is an excerpt from the downloadable world example that shows the values for `ode_70iters` and `ode_200iters`.

<include from=' <sdf version="1.6">' to='<!-- end physics presets, models and other world properties go here --> ' src='https://bitbucket.org/osrf/gazebo_tutorials/raw/default/preset_manager/files/preset_example_sdf1_6.world'/>

## C++ API
If you are writing a plugin that switches between profiles, you can use the C++
API to access the `PresetManager` to add, remove, and switch between profiles.
You can also get and set profile parameters and generate an SDF element
representing your physics profile.

Here's an example `PresetManager` code snippet that would programmatically
construct the example world shown above in SDF:

```
physics::WorldPtr world = physics::get_world("default");
// Get the PresetManager object from the world
physics::PresetManagerPtr presetManager = world->GetPresetManager();

// Create a new profile called "ode_70iters"
// It will be populated with the default physics parameters
// Many of the PresetManager functions return a boolean for error-checking
if (!presetManager->CreateProfile("ode_70iters"))
{
  gzerr << "Couldn't create profile ode_70iters" << std::endl;
  return -1;
}

// Create another profile
presetManager->CreateProfile("ode_200iters");

// Set the current profile to "ode_70iters"
presetManager->CurrentProfile("ode_70iters");

// Set the solver type to quickstep in the current profile, checking for errors
// SetCurrentProfileParam will change the current state of the physics engine
if (!presetManager->SetCurrentProfileParam("solver", "quick"))
{
  gzerr << "Couldn't set parameter, did you pass a valid key/value pair?"
  return -1;
}

// Set the number of iterations in the current profile to 70
presetManager->SetCurrentProfileParam("iters", 70);

// Set parameters in the other profile. These changes will be stored in
// presetManager but will not change the current state of the physics engine
presetManager->SetProfileParam("ode_200iters", "solver", "quick");
presetManager->SetProfileParam("ode_200iters", "iters", 200);

boost::any iters;

// Get the number of iterations from the current profile.
presetManager->GetCurrentProfileParam(iters, "iters");
// GetProfileParam and GetCurrentProfileParam currently return a boost::any,
// which must be casted to the correct type
gzmsg << "Iterations in current preset: " << boost::any_cast<int>(iters);

// Generate an SDF Element from the profile we constructed
sdf::ElementPtr odeQuickProfileSDF =
    presetManager->GenerateSDFFromPreset("ode_70iters");
```

## Command Line Interface
A quicker and more convenient way to switch physics profiles is to use the
command line interface.

To start the `preset_example` world we made above with a non-default profile,
start gazebo with:

```
gazebo preset_example_sdf1_6.world --profile ode_70iters
```

For a shortcut, use `-o`:

```
gazebo preset_example_sdf1_6.world -o ode_70iters
```

You can also substitute `gazebo` with `gzserver` to run Gazebo headless
(without the graphical client).

While Gazebo is running, you can switch the profile in another terminal
by using `gz physics`:

```
gz physics --profile ode_200iters
gz physics -o ode_70iters
```

# Example

Download the
[world file](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/preset_manager/files/preset_example_sdf1_6.world)
`preset_example_sdf1_6.world` (same as the SDF example shown above) and the
[bash script](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/preset_manager/files/switch_profiles_sdf1_6.sh)
`switch_profiles_sdf1_6.sh`.

<include src='https://bitbucket.org/osrf/gazebo_tutorials/raw/default/preset_manager/files/switch_profiles_sdf1_6.sh'/>

The script launches Gazebo with the `ode_70iters` profile and switches between
the two world profiles 5 times, pausing for 5 seconds between each switch.

The behavior of double pendulum model in this world illustrates the differences between the two physics
profiles. A classic double pendulum consists of two links attached by a hinge joint. One of the links is
attached to a fixed point via another hinge joint. In this example, the link attached to a fixed point
is much smaller, and thus there is a large inertia ratio between the two links. The world enforces a constant
force lateral to the hinge joint (in the x-direction) by setting the x component of gravity to 1.0 meters
per second squared, causing the pendulum to swing back and forth in the XZ plane.

[[file:files/inertia_ratio_pendulum.svg]]

The `ode_70iters` profile uses the "quickstep" physics constraint solver, which is faster but sometimes less
accurate. In particular, the large inertia ratio of this model causes the constraint solver to converge slowly.
You can observe that the pendulum tends to wobble when this profile is active. This profile is also set to
run with a real time factor of 1.5, which means that simulation runs in "fast motion" while this profile is active.

The `ode_200iters` profile also uses the "quickstep" solver but increases the number of iterations,
which gives the solver more time to converge. Thus when the profile switches, you can see the pendulum's motion
stabilize to back and forth behavior in the XZ plane. This profile runs with a real time factor of 1.0, which means
that the time passes as slowly in simulation as it does in real life.

You can also see the physics parameter changes reflected in the GUI. In the left-hand panel, under the "World" tab,
click on "Physics" and look at the physics properties of the world change as the script changes profiles. You
will have to close and reopen the menu to refresh the GUI.

[[file:files/worldtab.png|600px]]

Use this script as a base for your own experimentation and profiling with the Gazebo physics
library!
