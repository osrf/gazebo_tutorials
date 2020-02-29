# Overview

The source code for GzWeb is located at the
[osrf/gzweb](https://github.com/osrf/gzweb) GitHub repository.

The source code is composed of two main parts: Javascript code inside `gzweb/gz3d`,
responsible for visualization, and C++ code inside `gzweb/gzbridge`,
responsible for communicating with `gzserver`.

# Javascript development

GzWeb makes use of the
[JQuery mobile](http://jquerymobile.com/) user interface system for GUI and the
[three.js](http://threejs.org/) library for 3D rendering.

## Grunt tasks

[Grunt](http://gruntjs.com/) is used for running tasks including code checking and minification:

* Code check: uses JSHint for detecting potential errors in Javascript code.

* Concatenation: Concatenates javascript files as follows:

    * `gz3d.src.js`: All files under `gz3d/src`
    * `gz3d.gui.js`: All files under `gz3d/src` and all the necessary dependencies
    * `gz3d.js`: The same as `gz3d.gui.js` but without GUI-specific files and dependencies

* Minification: Compresses the 3 files above into `.min.js` versions which are more
  efficient to use in production.

## Workflow

1. Make changes to Javascript source code at `gzweb/gz3d/src`. You may also
   edit files at `gzweb/gz3d/client/js/include`, but keep in mind that these
   files are copied from external projects.

1. The following command runs Grunt to perform code check and contatenate files:

        npm run update

1. Verify your changes:

    1. Make sure `gzserver` is running
    1. Start gzweb with `npm start`
    1. Open browser at `localhost:8080`, or just refresh page.

    If you don't see anything changed after modifying the Javascript code, you
    might need to clear your browser cache to see the changes.

1. Run tests:

        npm test

1. Generate updated documentation:

        npm run docs

# C++ development

GzWeb communicates with `gzserver` by publishing and subscribing to Gazebo topics.

## Workflow

1. Make changes to C++ source code in `gzweb/gzbridge`

1. You can compile by running:

        npm run deploy

1. Verify your changes:

    1. Make sure `gzserver` is running
    1. Start gzweb with `npm start`
    1. Open browser at `localhost:8080`, or just refresh page.

# Making a contribution

## Bug reports and feature requests

On GzWeb's [issue tracker](https://github.com/osrf/gzweb/issues),
you're able to report bugs and ask for new features. Simply create an issue and
categorize it accordingly.

## Pull requests

If you've fixed a bug or added a feature and would like your changes to be
integrated into GzWeb, you can make a
[pull request](https://github.com/osrf/gzweb/pull-requests)  to the
repository, and the changes will be reviewed and merged.

