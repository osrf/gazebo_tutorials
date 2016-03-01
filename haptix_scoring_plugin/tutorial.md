# Overview

This tutorial demonstrates building a custom test world
to create a manipulation dexterity test inspired by a paper titled
"The strength-dexterity test as a measure of dynamic pinch performance"
written by Valero-Cuevas et. al. in 2003.

We assume that you have already completed the
[installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix)
and recommend that you have completed the
[world API](http://gazebosim.org/tutorials?tut=haptix_sim_api&cat=haptix)
tutorials.

# Relevant Documentations

The events SDF in the custom world can be found here:
<include lang='xml' src='https://bitbucket.org/osrf/handsim/src/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/worlds/luke_hand_spring_test.world?at=spring_buckle_test&fileviewer=file-view-default#luke_hand_spring_test.world-4' from='/<plugin name="SimEvents/' to='plugin name="HaptixWorldPlugin' />
(Recall that the documentation for building a Gazebo world using [SDF format](http://www.sdformat.org/) can be found [here](http://gazebosim.org/tutorials?cat=build_world)).

And here is the accompanying source code that interprets the `SimEventsPlugin` data can be found in `HaptixGUIPlugin.cc`:
<include lang='c' src='https://bitbucket.org/osrf/handsim/src/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc?at=default&fileviewer=file-view-default#HaptixGUIPlugin.cc-1050' from='void HaptixGUIPlugin::ScoringUpdate()' to='void HaptixGUIPlugin::PollTracking()' /> 
Where the `HaptixGUIPlugin::ScoringUpdate` function is spawned in its own thread [here](https://bitbucket.org/osrf/handsim/src/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc?at=default&fileviewer=file-view-default#HaptixGUIPlugin.cc-718).
And the `HaptixGUIPlugin::OnSimEvents` function [here](https://bitbucket.org/osrf/handsim/src/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc?at=default&fileviewer=file-view-default#HaptixGUIPlugin.cc-1560) [subscribes to state updates](https://bitbucket.org/osrf/handsim/src/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc?at=default&fileviewer=file-view-default#HaptixGUIPlugin.cc-463) published by the `SimEventsPlugin`.

For reference, the Gazebo SimEvents API documentation can be found
[here](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1SimEventsPlugin.html).

# Start Gazebo handsim simulation

To run the example, start gazebo in terminal:

~~~
gazebo --verbose worlds/luke_hand_spring_test.world
~~~

## Example Video
<iframe width="600" height="450" src="https://www.youtube.com/embed/q-WT0C6UhHc" frameborder="0" allowfullscreen></iframe>
