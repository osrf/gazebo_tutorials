# Overview

GazeboJs provides a scripting interface to the Gazebo-classic simulator. Specifically, it provides a javascript client for the simulator, using NodeJs and built on Google's V8 script engine.

GazeboJs is a C++ addon to NodeJs that is loaded inside node process at runtime (using the `require` javascript function). Once loaded, it provides javascript functions that communicate with the Gazebo-classic simultation server (the gzserver process) over the network, using the Gazebo-classic transport library.
This is the same mechanism that the Gazebo-classic simulation client (the gzclient process) uses to communicate with the simulation server.
The source code for this project can be found here: <https://bitbucket.org/osrf/gazebojs>

[[file:files/gazebojs_overview.png|640px]]

This page explains how to install the GazeboJs Node bindings to Gazebo.

### Ubuntu Linux

This tutorial shows how to download, install and compile Gazebojs on a computer where Gazebo-classic and its  development libraries are installed.

#### Setup

Install the osrf repository and install the development libraries, dependending on the version of Gazebo-classic you are using (for example, libgazebo4-dev for Gazebo-classic 4. see http://gazebosim.org/tutorials?cat=install). The dev Debian package contains the Gazebo-classic header files that are necessary for the gazebojs installation. This is because the NodeJs Gazebo-classic modules are automatically compiled on your machine when the 'npm install gazebojs' is invoked (see below).
Like Gazebo, gazebojs module uses semantic versioning, so the major version of gazebojs should be the same as the major version of Gazebo-classic you are using. You can specify rules about the version of gazebojs you want to use in the `package.json` file (see https://www.npmjs.org/doc/files/package.json.html).

Make sure you have curl

    sudo apt-get install curl

Install nodejs and npm

    curl -sL https://deb.nodesource.com/setup_4.x | sudo -E bash -
    sudo apt-get install -y nodejs

You may have to install nodejs-legacy if the node command does not exist:

    sudo apt-get install nodejs-legacy

If you already have nodejs, make sure that its version >=0.12.x:

    node --version

If you have a problem with installation or are using a different OS,
take a look at the NodeJs
[documentation](https://nodejs.org/en/download/package-manager/).

> **Tip**: In case you want to install more than one version of nodejs on your machine you can use nvm
>
>    ~~~
>    curl https://raw.githubusercontent.com/creationix/nvm/v0.30.2/install.sh | bash
>    source ~/.profile
>    nvm install [version number e.g: v5]
>    nvm use [version number e.g: v5]
>    ~~~

Install jansson (JSON library)

    sudo apt-get install libjansson-dev

Make sure that these packages can be found in your pkg-config path. You can check this by executing this command without any error:

    pkg-config --cflags gazebo jansson protobuf


#### Running

Now that everything is installed, here are the steps to test it:


Create a NodeJs project directory

    mkdir gz_node_inst
    cd gz_node_inst


Install Gazebojs

     npm install gazebojs || echo "Installation failed!"

This operation should download and compile the latest gazebojs. There is a C++
compilation phase where a NodeJs module is created. There should now be a
`gz_node_inst/node_modules` directory and you should not see the installation
failed message.


Test your installation:

Launch Gazebo-classic in a separate terminal and verify that the simulation is running (Sim Time increases):

    gazebo


Use the 'node' command (from the same directory where you invoked the npm
command) to invoke the NodeJs REPL console

    node

Type in the following commands to load the gazeboJs module, create a
simulation client and pause the running simulation

> **Note**: 'undefined' messages are NodeJS console valid responses to your
command

    > var gazebojs = require('gazebojs')
    undefined
    > var sim = new gazebojs.Gazebo()
    undefined
    > sim.pause()
    undefined

You should see the simulation stop in Gazebo.

