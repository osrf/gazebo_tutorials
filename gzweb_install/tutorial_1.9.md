# Overview

Gzweb is installed on the server-side. Once the server is set up and running, clients can interact with the simulation simply by accessing the server's URL on a web browser.

# Dependencies

Gzweb is a graphical interface which communicates with gzserver. To use
 gzserver, install either [Gazebo](http://gazebosim.org/install) or [DRCSim](/tutorials?tut=drcsim_install&cat=drcsim).

 1. Make sure your system has the right NodeJS version (0.10.x). While this is
 not the latest version, it is the version that ships with Ubuntu Trusty.

    >**Note:** For Ubuntu Precise or older distributions, the NodeJS version that comes with it may not work with gzweb. In that case, set up your system to grab and install the latest NodeJS debs:

    ~~~~
    dpkg -l nodejs
    ~~~

    >If the above command returns a version < 0.10 or couldn't find the nodejs package use:

    ~~~
    curl -sL https://deb.nodesource.com/setup | sudo bash -
    ~~~

    >We will install the nodejs package in the following step.

 1. Next, install the dependencies from a terminal:

    ~~~
    sudo apt-get install libjansson-dev nodejs npm libboost-dev imagemagick libtinyxml-dev git cmake build-essential
    ~~~


# Clone the repository and build

 1. Clone the repository into a directory in your home folder:

    ~~~
    cd ~; git clone https://github.com/osrf/gzweb
    ~~~

 1. Enter the Gzweb repository and switch to a release branch:

    ~~~
    cd ~/gzweb
    # Note for Gazebo versions < 7, please use the gzweb_1.2.0 branch
    git checkout gzweb_1.3.0
    ~~~

 1. The first time you build, you'll need to gather all the Gazebo models in the right directory and prepare them for the web. Before running the deploy script, you'll need to source the Gazebo setup.sh file:

    >If you installed gazebo via deb packages:

    ~~~
    source /usr/share/gazebo/setup.sh
    ~~~

    >If you did a source install then:

    ~~~
    source <YOUR_GAZEBO_PATH>/share/gazebo/setup.sh
    ~~~

 1. If you have drcsim then source:

    ~~~
    source /usr/share/drcsim/setup.sh
    ~~~

 1. Run the deploy script, this downloads models from the web and may take a couple of minutes.

    ~~~
    ./deploy.sh -m
    ~~~

    >Note: the `-m` flag tells the deploy script to grab models from the model database and any other models in your Gazebo paths. For all subsequent builds, the `-m` flag will not be needed. The process will also try to generate thumbnails, see note on thumbnails below.

## Options

* To skip downloading models from the model database and grab only local models in your Gazebo model path, do:

        ./deploy.sh -m local

* To generate thumbnails manually, run the script with the `-t` flag, i.e.:

        ./deploy.sh -t

    >Note: This spins up a gzserver with a camera for capturing screenshots of models. So make sure there is rendering support and no background gzerver process running (or set a different `GAZEBO_MASTER_URI` in the terminal).

* If you'll use gzweb on mobile devices, you can create coarse versions of all models, which are lighter to load (50% of original quality). If generated, these meshes will automatically be used on mobile devices. If you've already ran `./deploy.sh -m`, run just:

        ./deploy.sh -c

    Or you can run both flags at the same time to generate coarse versions as you create the database:

        ./deploy.sh -m -c

* You also have the option to pick specific models and how much percent to coarsen, running:

        ./coarse_meshes.sh [percent] [path]

    Here, `[percent]` is the edges ratio with respect to the original mesh (0 to 100), and `[path]` is the path of the models. For example:

        ./coarse_meshes.sh 20 http/client/assets/bowl/

# Running gzserver, gzweb server, and WebGL client

1. Start gazebo or gzserver first:

    ~~~
    gzserver
    ~~~

1. On another terminal, from your gzweb directory, start the http and websocket servers:

    ~~~
    ./start_gzweb.sh
    ~~~

1. Open a browser (Chrome works well) that has WebGL and websocket support and point it to the ip address and port where the http server is started, by default it's on port 8080, e.g.

    ~~~
    http://localhost:8080
    ~~~

    > You can use the `-p` option to choose an arbitrary port, for example: `./start_gzweb.sh -p 1234`

1. To stop Gzweb server, from your gzweb directory, run:

    ~~~
    ./stop_gzweb.sh
    ~~~

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

    A: You will need to install Grunt and run it appropriately, as described in the [development](/tutorials?tut=gzweb_development) section.

 * **Q: When running `./deploy.sh`, I see errors along the lines of:**

        gyp ERR! configure error

    A: There might be a conflict between the gyp version installed and the gyp version in node-gyp. Try removing gyp:

        sudo apt-get remove gyp

 * **Q: When running `./deploy.sh`, I have problems finding GTS, like this:**

        ~/gzweb/tools/gzcoarse.cc:18:17: fatal error: gts.h : no such file or directory, #include <gts.h>

    A: It seems that your Gazebo installation didn't install GTS headers. Try installing them manually:

        sudo apt-get install libgts-dev
