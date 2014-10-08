# Adding a model to Gazebo

First, create a directory for your model. For this tutorial, we will assume that this directory is called ''mymodel''. That directory should include the following files:

    model.config
    model.sdf

# Contents of model.config:

The `model.config` file provides information necessary to pick the proper SDF file, information on authorship of the model, and a textual description of the model. 

A sample model.config looks like this:

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


# Path for the directory

The directory should be located in the default Gazebo models path (_~/.gazebo/models_) or under the `GAZEBO_MODEL_PATH`.
