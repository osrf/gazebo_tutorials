# Overview

Gzweb is usually installed on an Ubuntu server. Once the server is set up and running,
clients can interact with the simulation simply by accessing the server's URL
on a web browser.

# Dependencies

You'll need a full [Gazebo install](http://gazebosim.org/install) on your server,
including development packages.

GzWeb currently works with NodeJS versions >= 4, so you should be able to use the
version shipped with Ubuntu Xenial or higher.

> For Trusty, you can follow instructions from
  [here](https://github.com/nodesource/distributions) for example to upgrade the
  Node version.

To install all dependencies, including Gazebo's official Ubuntu packages and
Node on Ubuntu Xenial or higher:

~~~
sudo apt install gazebo libgazebo-dev libjansson-dev nodejs npm nodejs-legacy libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
~~~

# Build GzWeb

1. Clone the repository, for example, into a directory in your home folder:

        cd ~; hg clone https://bitbucket.org/osrf/gzweb

1. Enter the Gzweb repository and switch to the 1.4.0 release branch:

        cd ~/gzweb
        hg up gzweb_1.4.0

1. The first time you build, you'll need to gather all the Gazebo models which
   you want to simulate in the right directory ('http/client/assets') and prepare
   them for the web.

   Before running the deploy script, it's important to source the Gazebo
   `setup.sh` file:

    > If you installed gazebo via deb packages:

        source /usr/share/gazebo/setup.sh

    > If you did a source install then:

        source <YOUR_GAZEBO_PATH>/share/gazebo/setup.sh

1. Run the deploy script, this downloads models from the web and may take a couple of minutes.

        ./deploy.sh -m

    > Note: the `-m` flag tells the deploy script to grab all the models from the
     [model database](https://bitbucket.org/osrf/gazebo_models/) and any other
     models in your `GAZEBO_MODEL_PATH`. For all subsequent builds, the `-m` flag
     will not be needed.

## Options

* To skip downloading models from the model database and grab only local models
  in your Gazebo model path, do:

        ./deploy.sh -m local

* To generate thumbnails for all the models , run the script with the `-t` flag, i.e.:

        ./deploy.sh -t

    > Note: This spins up a `gzserver` with a camera for capturing screenshots
    of models. So make sure there is rendering support and no background gzerver
    process running (or set a different `GAZEBO_MASTER_URI` in the terminal).

* If you'll use GzWeb on mobile devices, you can create coarse versions of all
  models, which are lighter to load (50% of original quality). If generated,
  these meshes will automatically be used on mobile devices. If you've already
  ran `./deploy.sh -m`, run just:

          ./deploy.sh -c

      Or you can run both flags at the same time to generate coarse versions as
      you create the database:

          ./deploy.sh -m -c

      You also have the option to pick specific models and how much percent to
      coarsen, running:

          ./coarse_meshes.sh [percent] [path]

      Here, `[percent]` is the edges ratio with respect to the original mesh
      (0 to 100), and `[path]` is the path of a model. For example:

          ./coarse_meshes.sh 20 http/client/assets/bowl/

# Running gzserver, GzWeb server, and WebGL client

1. On the server machine, start `gazebo` or `gzserver` first:

        gzserver

1. On another terminal, from your GzWeb directory, run the following command to
   start an HTTP server to serve static content (website assets and models) and
   a websocket server that sends periodic updates:

        npm start

1. Open a browser that has WebGL and websocket support (i.e. most modern browsers)
   and point it to the IP address and port where the HTTP server is started, by
   default it's on port 8080, e.g.

        http://localhost:8080

    > You can use the `-p` option to choose an arbitrary port, for example:
      `npm start -p 1234`

1. To stop the GzWeb server, just press `Ctrl+C` in the terminal.

# Troubleshooting

 * **Q: When installing node package modules, I see errors along the lines of:**

        npm ERR! Error: failed to fetch from registry: node-gyp

    A: Try setting the npm registry first then install the modules again.

        npm config set registry http://registry.npmjs.org/

 * **Q: When installing websocket, I see errors along the lines of:**

        sh: 1: node: not found
        npm ERR! error installing websocket@1.0.8
        npm WARN This failure might be due to the use of legacy binary "node"

    **Or along the lines of:**

        /usr/bin/env: node: No such file or directory
        There are node-gyp build errors, exiting.

    A: In Debian systems, the binary file "node" has been renamed to "nodejs" to avoid a name conflict. Try adding a symlink to the correct name:

        sudo ln -s /usr/bin/nodejs /usr/bin/node

    You may also find that your repository is too old and you should just install recent versions of node and npm directly.

 * **Q: When running `./deploy.sh`, it is missing a file in `gz3d/build`:**

    A: You will need to install Grunt and run it appropriately, as described in the [development](http://gazebosim.org/tutorials?tut=gzweb_development) section.

 * **Q: When running `./deploy.sh`, I see errors along the lines of:**

        gyp ERR! configure error

    A: There might be a conflict between the gyp version installed and the gyp version in node-gyp. Try removing gyp:

        sudo apt-get remove gyp

 * **Q: When running `./deploy.sh`, I have problems finding GTS, like this:**

        ~/gzweb/tools/gzcoarse.cc:18:17: fatal error: gts.h : no such file or directory, #include <gts.h>

    A: It seems that your Gazebo installation didn't install GTS headers. Try installing them manually:

        sudo apt-get install libgts-dev
