# Overview

Gzweb is installed on the server-side. Once the server is set up and running, clients can interact with the simulation simply by accessing the server's URL on a web browser.

# Dependencies

Gzweb is a graphical interface which communicates with gzserver. To use gzserver, install either [Gazebo](http://gazebosim.org/install) or [DRCSim](http://gazebosim.org/tutorials?tut=drcsim_install&cat=drcsim).

Make sure your system supports a more recent version of NodeJS (>=0.10) then install the dependencies from the terminal:

    sudo apt-get install libjansson-dev nodejs npm libboost-dev imagemagick libtinyxml-dev mercurial

>**Note:** For Ubuntu Precise or older distributions, the NodeJS version that comes with it may not work with gzweb. In that case, set up your system to grab and install the latest NodeJS debs:

>     curl -sL https://deb.nodesource.com/setup | sudo bash -
>     sudo apt-get install nodejs

>If NodeJS installs without errors then install the rest of the dependencies (leaving out npm as that should be installed with NodeJS):

>     sudo apt-get install libjansson-dev nodejs libboost-dev make cmake mercurial g++ libtinyxml-dev imagemagick

After all dependencies have been installed, install NodeJS modules:

    npm config set registry http://registry.npmjs.org/
    sudo npm install -g node-gyp
    sudo npm install -g http-server


# Clone the repository and build

Clone the repository into a directory in your home folder:

    cd ~; hg clone https://bitbucket.org/osrf/gzweb

Enter the Gzweb repository and switch to the 1.2.0 branch:

    cd ~/gzweb
    hg up gzweb_1.2.0

The first time you build, you'll need to gather all the Gazebo models in the right directory and prepare them for the web. Before running the deploy script, you'll need to source the Gazebo setup.sh file:

    . /usr/share/gazebo/setup.sh

If you have drcsim then source:

    . /usr/share/drcsim/setup.sh

Run the deploy script, this downloads models from the web and may take a couple of minutes.

    ./deploy.sh -m

Note: the `-m` flag tells the deploy script to grab models from the model database and any other models in your Gazebo paths. For all subsequent builds, the `-m` flag will not be needed.

## Options

* To skip downloading models from the model database and grab only local models in your Gazebo model path, do:

        ./deploy.sh -m local

* To generate thumbnails manually, run the script with the `-t` flag, i.e.:

        ./deploy.sh -t

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

        gzserver

1. On another terminal, from your gzweb directory, start the http and websocket servers:

        ./start_gzweb.sh

1. Open a browser (Chrome works well) that has WebGL and websocket support and point it to the ip address and port where the http server is started, by default it's on port 8080, e.g.

        http://localhost:8080

To stop Gzweb server, from your gzweb directory, run:

    ./stop_gzweb.sh

# Troubleshooting

 * **Q: When installing node package modules, I see errors along the lines of:**

        npm ERR! Error: failed to fetch from registry: node-gyp

    A: Try setting the npm registry first then install the modules again.

        npm config set registry http://registry.npmjs.org/

 * **Q: When installing websocket, I see errors along the lines of:**

        sh: 1: node: not found
        npm ERR! error installing websocket@1.0.8
        npm WARN This failure might be due to the use of legacy binary "node"

    A: In Debian systems, the binary file "node" has been renamed to "nodejs" to avoid a name conflict. Try adding a symlink to the correct name:

        sudo ln -s /usr/bin/nodejs /usr/bin/node

    You may also find that your repository is too old and you should just install recent versions of node and npm directly.

 * **Q: When running `./deploy.sh`, it is missing a file in `gz3d/build`:**

    A: You will need to install grunt and run it appropriately, as described in the following section.

 * **Q: When running `./deploy.sh`, I see errors along the lines of:**

        gyp ERR! configure error

    A: There might be a conflict between the gyp version installed and the gyp version in node-gyp. Try removing gyp:

        sudo apt-get remove gyp

