# Overview

The source code for Gzweb is located at the [osrf/gzweb](https://bitbucket.org/osrf/gzweb) Bitbucket repository. The source code is composed of two main parts: Javascript code inside `gzweb/gz3d`, responsible for visualization, and C++ code inside `gzweb/gzbridge`, responsible for communicating with `gzserver`. 

# Javascript development

Gzweb makes use of the [JQuery mobile](http://jquerymobile.com/) user interface system for GUI and the [three.js](http://threejs.org/) library for 3D rendering.

## Grunt setup

[Grunt](http://gruntjs.com/) is used for running tasks including code checking and minification:

* Minification: Compresses all `*.js` code located at  `gzweb/gz3d/src` into `gz3d.js` and `gz3d.min.js` files that can be included into other projects.
* Code check: uses JSHint for detecting potential errors in Javascript code.

To install required Grunt packages:

    cd gz3d/utils && npm install

## Work Flow:

1. Make changes to Javascript source code at `gzweb/gz3d/src`. You may also edit files at `gzweb/gz3d/client/js/include`, but keep in mind most of these were taken from somewhere else.

1. Code check and minify Javascript files and copy them to the right directory, running:

        ./updateGZ3D.sh

1. Verify your changes: start gzweb with `./start_gzweb.sh` and open browser to `localhost:8080`, or just refresh page. If you don't see anything changed after modifying the Javascript code, you might need to clear your browser cache to see the changes.

# C++ development

Gzweb communicates with `gzserver` by publishing and subscribing to Gazebo topics.

## Work Flow:

1. Make changes to C++ source code in `gzweb/gzbridge`

1. You can compile by running:

        ./deploy.sh

1. Verify your changes: start gzweb with `./start_gzweb.sh` and open browser to `localhost:8080`, or just refresh page.

## Making a contribution

# Bug reports and feature requests

On Gzweb's [issue tracker](https://bitbucket.org/osrf/gzweb/issues?status=new&status=open), you're able to report bugs and ask for new features. Simply create an issue and categorize it accordingly.

# Pull requests

If you've fixed a bug or added a feature and would like your changes to be integrated into Gzweb, you can make a [pull request](https://bitbucket.org/osrf/gzweb/pull-requests)  to the repository, and the changes will be reviewed and merged.
