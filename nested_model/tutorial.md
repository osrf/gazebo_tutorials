# Overview

**Prerequisites:** [Make a model](/tutorials/?tut=build_model)

This tutorial describes how you can embed a model inside another to create an
assembly of models.

# Nested Model

It was seen in the
[Make a model](/tutorials/?tut=build_model) tutorial that a
model SDF is composed of a collection of `links` and `joints`. As of SDF 1.5,
the `<model>` SDF element has been extended to support self-referencing, which
means allowing `<model>` elements to be nested. Support for loading nested
`<model>` elements has been added in Gazebo 7.

Here is a basic example of a nested model SDF:

~~~
<sdf version="1.6">
  <model name="model_00">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="link_00">
      <pose>0.0 0 0 0 0 0</pose>
      <collision name="collision_00">
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="visual_00">
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
      </visual>
    </link>

    <model name="model_01">
      <pose>1.0 0 0.0 0 0 0</pose>
      <link name="link_01">
        <pose>0.25 0 0.0 0 0 0</pose>
        <collision name="collision_01">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual_01">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </model>
</sdf>
~~~

This model SDF is composed of a link (`link_00`), and a nested model
(`model_01`) with another link (`link_01`). Since a model in Gazebo is just an
abstract container for a group of objects, loading this model in Gazebo will
result in just two rigid bodies being created in the physics engine; one for the
sphere link and the other for the nested box link. By default, they will not
self-collide just like other links within the same model. On the GUI client,
you will see a sphere and a box sitting side-by-side and should not notice
any visual difference between nested models and links.

[[file:files/nested_model.png|640px]]

# Joints

Joints can also be created between links in nested models. Here is an example
of a joint that can be added to the model SDF above:

~~~
    <joint name="joint_00" type="revolute">
      <parent>link_00</parent>
      <child>model_01::link_01</child>
      <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>
      <axis>
        <xyz>1.0 0.0 0.0</xyz>
      </axis>
    </joint>
~~~

This joint SDF element can be added to either the top level or nested `<model>`
element. A revolute joint is then created between the sphere and the box links.
Pay attention to the scoping of `<parent>` and `<child>` names; references to
nested model links need to be scoped but minus the top level model name prefix.

[[file:files/nested_model_joint.png|640px]]

# Note on the include SDF element

Another approach for nesting models is demonstrated in the
[Add a Sensor to a Robot](/tutorials?tut=add_laser) tutorial
which introduces the use of the `<include>` element.

The `<include>` element works by taking all the links from the included model
and embedding them into the parent model. The downside of this approach is that
the model representation is modified during the process, i.e. saving the world
will result in a model with all the links combined together in one `<model>`
element without preserving the `<include>` tag. This is one of the shortcomings
which the nested `<model>` element is designed to address.

On the other hand, the `<include>` element is a simple and clean solution that
only requires a reference to an SDF file for creating a model assembly.
Future work will look into extending the nested `<model>` SDF element with
this feature.
