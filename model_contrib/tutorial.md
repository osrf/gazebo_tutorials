# Overview

This tutorial will explain how to add a model to the
[Gazebo Model Database](http://models.gazebosim.org/).
You can read more about the database
[here](http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot).

This tutorial assumes you have created an original Gazebo model and you'd like
to share it with the community.

This tutorial also assumes that you have an account on Bitbucket, and that you have a client for [Mercurial](http://mercurial.selenic.com).

> **Note**: You don't need to add your model to the database in order to use it
on Gazebo. The database is a common place where you can find models which are
useful for the whole commuinity.

# Fork and clone the osrf/gazebo\_models repository
Go to [https://bitbucket.org/osrf/gazebo\_models](https://bitbucket.org/osrf/gazebo_models) and, from the menu on the left hand side of the screen, choose "Fork". The default options are generally fine.

After you have forked the repository, clone it. Assuming that you chose the
default name for the repository, you will clone using a command on a terminal
similar to the following:

    hg clone https://bitbucket.org/yourname/gazebo_models

where _yourname_ is your Bitbucket username.

# Creating a model

Create a directory for your model under the **gazebo\_models** directory.
For this tutorial, we will assume that this directory is called **mymodel**,
but you should give the directory an informative name about the model.

That directory must include the file **model.config** and at least one `.sdf`
file. It may include other files as well (plugins, makefiles, README's, etc.)

Also make sure you add the model directory to the
[CMakeLists.txt](https://bitbucket.org/osrf/gazebo_models/src/default/CMakeLists.txt?at=default&fileviewer=file-view-default)
file.

Finally, make sure that you have permission to distribute all the files included
in the model, and they're not copywrited material.

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


# Adding the directory (and files) to the repository

You can add all of your files to the repository by typing the following from
the root directory of `gazebo_models`:

    hg add mymodel

or, if you have some files that you do not wish to track, you can add files individually:

    hg add mymodel/model.config
    hg add mymodel/model.sdf
etc.

# Committing and pushing

Commit and push your changes to your fork on Bitbucket. Give the commit a
descriptive message, for example:

    hg commit -m "Adding mymodel to the database"
    hg push

# Final step: creating a pull request

Assuming that your Bitbucket username is _yourname_ and you used the defaults
for the fork, you would find the forked repository on:

[https://bitbucket.org/yourname/gazebo_models](https://bitbucket.org/yourname/gazebo_models)

1. From that site, pick "Create pull request" from the menu on the left side of the web page.

1. Make sure that `osrf/gazebo_models` is selected to the right of the arrow.

1. Give your pull request a meaningful title referring to your model.

1. On the description, describe any relevant information about the model and why
you think the community could benefit from it. It's also recommended to add a
picture of the model. You can see a good example of a pull request
[here](https://bitbucket.org/osrf/gazebo_models/pull-requests/241).

1. When satisfied with your other options, click "Create pull request".

1. OSRF will review your pull request and begin integrating your changes into
the model database.
