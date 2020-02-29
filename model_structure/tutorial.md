# Overview

Gazebo is able to dynamically load models into simulation either programmatically or through the GUI. Models exist on your computer, after they have been downloaded or created by you. This tutorial describes Gazebo's model directory structure, and the necessary files within a model directory.

Models in Gazebo define a physical entity with dynamic, kinematic, and
visual properties. In addition, a model may have one or more plugins, which
affect the model's behavior. A model can represent anything from a simple
shape to a complex robot; even the ground is a model.

Gazebo relies on a database to store and maintain models available for use
within simulation. The model database is a community-supported resource, so
please upload and maintain models that you create and use.

## The Model Database Repository

The model database is a GitHub repository found [here](https://github.com/osrf/gazebo_models).

You can  clone the repository using:

        git clone https://github.com/osrf/gazebo_models


## Model Database Structure

A model database must abide by a specific directory and file structure. The
root of a model database contains one directory for each model, and a
`database.config` file with information about the model database. Each model
directory also has a `model.config` file that contains meta data about the
model. A model directory also contains the SDF for the model and any materials,
meshes, and plugins.

The structure is as follows (in this example the database has only one model called `model_1`):

* *Database*
    * *database.config* : Meta data about the database. This is now populated automatically from CMakeLists.txt
    * *model_1* : A directory for model_1
        * *model.config* : Meta-data about model_1
        * *model.sdf* : SDF description of the model
        * *model.sdf.erb* : Ruby embedded SDF model description
        * *meshes* : A directory for all COLLADA and STL files
        * *materials* : A directory which should only contain the `textures` and `scripts` subdirectories
            * *textures* : A directory for image files (jpg, png, etc).
            * *scripts* : A directory for OGRE material scripts
        * *plugins*: A directory for plugin source and header files

### Plugins Directory
This is an optional directory that contains all of the plugins for the model.

### Meshes Directory
This is an optional directory that contains all of the COLLADA and/or STL files for the model.

### Material Directory
This is an optional directory that contains all of the textures, images, and OGRE scripts for the model. Texture images must be placed in the `textures` subdirectory, and OGRE script files in the `scripts` directory.

### Database Config
This is the `database.config` file in the root of the model database. This file contains license information for the models, a name for the database, and a list of all the valid models.

**Note: The `database.config` file is only required for online repositories. A directory full of models on your local computer does not need a `database.config` file.**

The format of this `database.config` is:

~~~
<?xml version='1.0'?>
<database>
  <name>name_of_this_database</name>
  <license>Creative Commons Attribution 3.0 Unported</license>
  <models>
    <uri>file://model_directory</uri>
  </models>
</database>
~~~

   *  <*name*>

      The name of the database. This is used by the GUI and other tools.

   *  <*license*>

      The license for the models within the database. We highly recommend the[Creative Commons Attribution 3.0 Unported](http://creativecommons.org/licenses/by/3.0) license.

   *  <*models*>

      A listing of all the model URIs within the database.

      * <*uri*>

        The URI for a model, this should be `file://model_directory_name`

### Model Config

Each model must have a `model.config` file in the model's root directory that contains meta information about the model.

The format of this `model.config` is:

~~~
<?xml version="1.0"?>

<model>
  <name>My Model Name</name>
  <version>1.0</version>
  <sdf version='1.5'>model.sdf</sdf>

  <author>
    <name>My name</name>
    <email>name@email.address</email>
  </author>

  <description>
    A description of the model
  </description>
</model>
~~~

   *  <*name*> *required*

      Name of the model.

   *  <*version*> *required*

      Version of this model.

      *Note:* This is not the version of sdf that the model uses. That information
      is kept in the `model.sdf` file.

   *  <*sdf*> *required*

      The name of a SDF or URDF file that describes this model. The `version` attribute indicates what SDF version the file uses, and is not required for URDFs. Multiple <*sdf*> elements may be used in order to support multiple SDF versions.

   *  <*author*> *required*

      *  <*name*> *required*

         Name of the model author.

      *  <*email*> *required*

         Email address of the author.

   *  <*description*> *required*

      Description of the model should include:
      >  * What the model is (e.g., robot, table, cup)
      >  * What the plugins do (functionality of the model)

   *  <*depend*> *optional*

      All the dependencies for this model. This is typically other models.

      *  <*model*> *optional*

         *  <*uri*> *required*

            URI of the model dependency.

         *  <*version*> *required*

            Version of the model.

### Model SDF

Each model requires a `model.sdf` file that contains the Simulator Description Format of the model. You can find more information on the [SDF website](http://sdformat.org).

### Model SDF.ERB

Standard SDF file which can contain ruby code embedded. This option is used to
programatically generate SDF files using [Embedded Ruby code](http://www.stuartellis.eu/articles/erb/)
templates. Please note that the ruby conversion should be done manually (`erb
model.sdf.erb > model.sdf`) and the final `model.sdf` file must be uploaded
together with the `model.sdf.erb` (this one only for reference).

Examples of `sdf.erb` files are available in the
[gazebo_models repository](https://github.com/osrf/gazebo_models/src)
(some of them use the deprecated suffix `.rsdf`). An easy ERB file is the
[flocking.world.erb](https://github.com/osrf/gazebo/src/b54961341ffb938a9f99c9976aed50a771c95216/worlds/flocking.world.erb?at=default)
which uses a simple loop.
