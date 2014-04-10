#Tutorial: Add or modify a tutorial# 

This tutorial describes how to modify an existing tutorial or add a new tutorial.

### Tutorial source code

All tutorials exist in a Bitbucket repository:
[https://bitbucket.org/osrf/gazebo_tutorials](https://bitbucket.org/osrf/gazebo_tutorials).

This repository contains a `manifest.xml` that holds meta-information about
the tutorials, and a set of directories that hold the tutorial contents.

All changes to the set of tutorials will use the [pull-request](https://bitbucket.org/osrf/gazebo_tutorials/pull-request/new) feature on the [Bitbucket](https://bitbucket.org/osrf/gazebo_tutorials) site.

### Modify a tutorial

Follow these steps to change an existing tutorial.

1. [Fork](https://bitbucket.org/osrf/gazebo_tutorials/fork) the gazebo_tutorials repository.

1. Edit your fork to include the necessary modifications.

1. Create a [pull-request](https://bitbucket.org/osrf/gazebo_tutorials/pull-request/new) back to the gazebo_tutorials repository.

1. Your pull-request will be reviewed, during which time you may be asked to make changes.

1. Once accepted, your pull-request will be merged into the repository. Your changes should appear on the main gazebo website in a few minutes.

### Create a new tutorial

The steps for creating a new tutorial are similar to those for modifying a tutorial.

1. [Fork](https://bitbucket.org/osrf/gazebo_tutorials/fork) the gazebo_tutorials repository.

1. Add a new top-level directory to hold the contents of your tutorial.

1. Populate the new directory with a text file formated using markdown and any other supporting material.

1. Add a new `<tutorial>...</tutorial>` block to `manifest.xml`

1. Create a [pull-request](https://bitbucket.org/osrf/gazebo_tutorials/pull-request/new) back to the gazebo_tutorials repository.

1. Your pull-request will be reviewed, during which time you may be asked to make changes.

1. Once accepted, your pull-request will be merged into the repository. Your changes should appear on the main gazebo website in a few minutes.

