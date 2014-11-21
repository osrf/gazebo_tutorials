
This tutorial assumes that you have an account on Bitbucket, and that you have a client for [Mercurial](http://mercurial.selenic.com).

## Fork and clone the osrf/gazebo\_models repository
Go to [https://bitbucket.org/osrf/gazebo\_models](https://bitbucket.org/osrf/gazebo_models) and, from the menu on the left hand side of the screen, choose "Fork". The default options are generally fine. After you have forked the repository, clone it. Assuming that you chose the default name for the repository, you will clone using commands similar to the following: 

    code$ hg clone https://yourname@bitbucket.org/yourname/gazebo_models

where _yourname_ is your Bitbucket username.

## Creating a model
Create a directory for your model under the **gazebo\_models** directory. For this tutorial, we will assume that this directory is called **mymodel**. That directory must include the file **model.config**, and it may include other files as well (plugins, makefiles, README's, etc.)

## Contents of **model.config**:
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


## Adding the directory (and files) to the repository
You can add all of your files to the repository by typing:
   
    gazebo_models$ hg add mymodel
    
or, if you have some files that you do not wish to track, you can add files individually:

    gazebo_models$ hg add mymodel/model.config
    gazebo_models$ hg add mymodel/model.sdf
etc.

## Committing and pushing
Commit and push your changes to Bitbucket:

    gazebo_models$ hg commit
    gazebo_models$ hg push

## Final step: creating a pull request

From your Bitbucket repository **https://bitbucket.org/yourname/gazebo\_models** (assuming that your Bitbucket username is _yourname_ and you used the defaults for the fork, this is where you would find the forked repository), create a pull request. Pick "Create pull request" from the menu on the left side of the web page. Make sure that "osrf/gazebo\_models" is selected to the right of the arrow. When satisfied with your other options, click "Create pull request". OSRF will review your pull request and begin integrating your changes into the model database.  
