# Adding a model to Gazebo

First, create a directory for your model. For this tutorial, we will assume that this directory is called **mymodel**. That directory should include the file **model.config**.

# Contents of **model.config**:

The **model.config** file provides information necessary to pick the proper SDF file, information on authorship of the model, and a textual description of the model. 

A sample **model.config** looks like this:

    <?xml version="1.0"?>
    <model>
      <name>Wedge juggler</name>
      <version>1.0</version>
      <sdf version="1.5">model.sdf</sdf>

      <author>
        <name>Evan Drumwright</name>
        <email>drum@gwu.edu</email>
      </author>

      <description>
        A ball-in-wedge juggler. 
      </description>
    </model>

This **model.config** file indicates that the simulator's definition of the model (i.e., visual, inertial, kinematic, and geometric properties, among others), is located in **model.sdf**, and follows [SDF standard 1.5](http://sdformat.org/spec). It is possible to define multiple versions of your model, which may be useful if you intend for your model to be used with different versions of Gazebo. For example, we now change the contents of the file above, to support three different versions of SDF:

    <?xml version="1.0"?>
    <model>
      <name>Wedge juggler</name>
      <version>1.0</version>
      <sdf version="1.5">model.sdf</sdf>
      <sdf version="1.4">model-1.4.sdf</sdf>

      <author>
        <name>Evan Drumwright</name>
        <email>drum@gwu.edu</email>
      </author>

      <description>
        A ball-in-wedge juggler. 
      </description>
    </model>


# Path for the directory

The directory (**mymodel**) should be located in the default Gazebo models path (_~/.gazebo/models_) or under the path pointed to be the `GAZEBO_MODEL_PATH` environment variable.
