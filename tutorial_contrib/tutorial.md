#Tutorial: Add or modify a tutorial# 

This tutorial describes how to modify an existing tutorial or add a new tutorial.

### Tutorial source code

All tutorials exist in a Bitbucket repository:
[https://bitbucket.org/osrf/gazebo_tutorials](https://bitbucket.org/osrf/gazebo_tutorials).

This repository contains a `manifest.xml` that holds meta-information about
the tutorials, and a set of directories that hold the tutorial contents.

All changes to the set of tutorials will use the
[pull-request](https://bitbucket.org/osrf/gazebo_tutorials/pull-request/new)
feature on the [Bitbucket](https://bitbucket.org/osrf/gazebo_tutorials) site.

### Modify a tutorial

Follow these steps to change an existing tutorial.

1. Click the Edit button located at the top-right of the tutorial you want to modify.

1. This will bring you to the tutorial source on Bitbucket.

1. Click the Edit button on the file (see [bitbucket help page](https://confluence.atlassian.com/display/BITBUCKET/Edit+online)).

1. Make your changes to the file using Markdown syntax.

1. Clock Commit at the bottom-right of the screen.

1. Type a commit message to describe your changes and click commit and make a pull request.

### Create a new tutorial

The steps for creating a new tutorial are similar to those for modifying a tutorial.

1. [Fork](https://bitbucket.org/osrf/gazebo_tutorials/fork) the gazebo_tutorials repository.

1. Add a new top-level directory to hold the contents of your tutorial.

1. Populate the new directory with a text file formated using markdown and any other supporting material.

1. Add a new `<tutorial>...</tutorial>` block to `manifest.xml`

    1. Make sure the `ref` attribute of the `<tutorial>` is unique and contains no special characters.

    1. The `<description>` should be very short.

    1. Set the minimum and maximum version of gazebo for which the specified markdown file is applicable. For example:

        1. `<markdown version="1.9">...`: Only version 1.9

        1. `<markdown version="1.9-">...`: All versions 1.9 and above

        1. `<markdown version="1.9-2.0">...`: All versions between 1.9 and 2.0, inclusive.

    1. It is possible to have multiple markdown files for different versions of Gazebo. Just add multiple `<markdown>` tags that reference different markdown files.
      

1. Create a [pull-request](https://bitbucket.org/osrf/gazebo_tutorials/pull-request/new) back to the gazebo_tutorials repository.

1. Your pull-request will be reviewed, during which time you may be asked to make changes.

1. Once accepted, your pull-request will be merged into the repository. Your changes should appear on the main gazebo website in a few minutes.

