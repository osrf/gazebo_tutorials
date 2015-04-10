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

In the following example
([downloadable here](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/preset_manager/files/preset_example.world)),
the `ode_quick_70iters` profile is set as the default, and the `ode_world_30iters`
profile will be available to be switched to using the C++ API or the terminal.

<include src='https://bitbucket.org/osrf/gazebo_tutorials/raw/default/preset_manager/files/preset_example.world'/>

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

// Create a new profile called "ode_quick_30iters"
// It will be populated with the default physics parameters
// Many of the PresetManager functions return a boolean for error-checking
if (!presetManager->CreateProfile("ode_quick_30iters"))
{
  gzerr << "Couldn't create profile ode_quick_30iters" << std::endl;
  return -1;
}

// Create another profile
presetManager->CreateProfile("ode_world_70iters");

// Set the current profile to "ode_quick_30iters"
presetManager->CurrentProfile("ode_quick_30iters");

// Set the solver type to quickstep in the current profile, checking for errors
// SetCurrentProfileParam will change the current state of the physics engine
if (!presetManager->SetCurrentProfileParam("solver", "quick"))
{
  gzerr << "Couldn't set parameter, did you pass a valid key/value pair?"
  return -1;
}

// Set the number of iterations in the current profile to 30
presetManager->SetCurrentProfileParam("iters", 30);

// Set parameters in the other profile. These changes will be stored in
// presetManager but will not change the current state of the physics engine
presetManager->SetProfileParam("ode_world_70iters", "solver", "world");
presetManager->SetProfileParam("ode_world_70iters", "iters", 70);

boost::any iters;

// Get the number of iterations from the current profile.
presetManager->GetCurrentProfileParam(iters, "iters");
// GetProfileParam and GetCurrentProfileParam currently return a boost::any,
// which must be casted to the correct type
gzmsg << "Iterations in current preset: " << boost::any_cast<int>(iters);

// Generate an SDF Element from the profile we constructed
sdf::ElementPtr odeQuickProfileSDF =
    presetManager->GenerateSDFFromPreset("ode_quick_30iters");
```

## Command Line Interface
A quicker and more convenient way to switch physics profiles is to use the
command line interface.

To start the `preset_example` world we made above with a non-default profile,
start gazebo with:

```
gazebo preset_example.world --profile ode_world_70iters
```

For a shortcut, use `-o` (`-p` is a different command that plays a log file):

```
gazebo preset_world.world -o ode_world_70iters
```

You can also substitute `gazebo` with `gzserver` to run Gazebo headless
(without the graphical client).

While Gazebo is running, you can switch the profile in another terminal
by using `gz physics`:

```
gz physics --profile ode_quick_30iters
gz physics -o ode_world_70iters
```

# Example

Download the
[world file](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/preset_manager/files/preset_example.world)
`preset_example.world` (same as the SDF example shown above) and the
[bash script](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/preset_manager/files/switch_profiles.sh)
`switch_profiles.sh`.

<include src='https://bitbucket.org/osrf/gazebo_tutorials/raw/default/preset_manager/files/switch_profiles.sh'/>

The script launches Gazebo with the `ode_world_70iters` profile and switches between
the two world profiles 10 times, pausing for 5 seconds between each switch.

In the left-hand panel in the GUI, under the "World" tab, click on "Physics" and look
at the physics properties of the world change as the script changes profiles. You will have
to close and reopen the menu to refresh the GUI.

[[file:files/worldtab.png|600px]]

You can also see the real time factor in the bottom of the screen change every 5 seconds,
since `ode_quick_30iters` has an unthrottled real time update rate and `ode_world_70iters` has
the default real time update rate and step size, which will give a real time factor of 1.0.

Use this script as a base for your own experimentation and profiling with the Gazebo physics
library!
