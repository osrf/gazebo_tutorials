# Overview

This tutorial demonstrates building a custom test world
to create a manipulation dexterity test inspired by a paper titled
["The strength-dexterity test as a measure of dynamic pinch performance"](http://www.asbweb.org/conferences/2001/pdf/012.pdf)
written by Valero-Cuevas et. al. in 2003.

For this tutorial, we assume that you have already completed the
[installation step](http://gazebosim.org/tutorials?tut=haptix_install&cat=haptix)
and we strongly recommend that you have completed the
[world API](http://gazebosim.org/tutorials?tut=haptix_sim_api&cat=haptix)
tutorials.

# Relevant Documentations

To begin building the tutorial, a good starting point is to add the `libSimEventsPlugin.so` block to the world SDF.
In our current example, custom SDF block for the custom plugin can be found here:

<include lang='xml' src='https://bitbucket.org/osrf/handsim/raw/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/worlds/luke_hand_spring_test.world' from='/<plugin name="SimEvents/' to='/plugin name="HaptixWorldPlugin/' />

(For reference, recall that the documentation for [SDF format](http://www.sdformat.org/) can be found [here](http://gazebosim.org/tutorials?cat=build_world)).

And the accompanying source code blocks that interprets the results of the `SimEventsPlugin` data can be found in [`HaptixGUIPlugin.cc`](https://bitbucket.org/osrf/handsim/src/spring_buckle_test/src/HaptixGUIPlugin.cc).

This block:

<include lang='c' src='https://bitbucket.org/osrf/handsim/raw/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc' from='/void HaptixGUIPlugin::ScoringUpdate\(\)/' to='/void HaptixGUIPlugin::PollTracking/' /> 

updates the GUI visual to keep track of the task completion state by changing colors of three circles to the lower right hand
side of the hand diagram used for contact sensor visualization.
Note the `HaptixGUIPlugin::ScoringUpdate` function is spawned in its own thread [here](https://bitbucket.org/osrf/handsim/src/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc?at=default&fileviewer=file-view-default#HaptixGUIPlugin.cc-718).
And the `HaptixGUIPlugin::OnSimEvents` function [referenced here](https://bitbucket.org/osrf/handsim/src/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc?at=default&fileviewer=file-view-default#HaptixGUIPlugin.cc-1560) [subscribes to state updates](https://bitbucket.org/osrf/handsim/src/d8e8f7e996266a85dc15e63687d1c8ee15e4bab7/src/HaptixGUIPlugin.cc?at=default&fileviewer=file-view-default#HaptixGUIPlugin.cc-463) published by the `SimEventsPlugin`.

For reference, the Gazebo SimEvents API documentation can be found
[here](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1SimEventsPlugin.html).

# Start Gazebo handsim simulation

To run the example, start gazebo in terminal:

~~~
gazebo --verbose worlds/luke_hand_spring_test.world
~~~

## Example Video

Below is an example of the custom world as tele-operated by keyboard and spacenav options:

<iframe width="600" height="450" src="https://www.youtube.com/embed/q-WT0C6UhHc" frameborder="0" allowfullscreen></iframe>
