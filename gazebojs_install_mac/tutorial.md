# Overview

GazeboJs provides a scripting interface to the Gazebo simulator. Specifically, it provides a javascript client for the simulator, using NodeJs and built on Google's V8 script engine.

GazeboJs is a C++ addon to NodeJs that is loaded inside node process at runtime (using the `require` javascript function). Once loaded, it provides javascript functions that communicate with the Gazebo simultation server (the gzserver process) over the network, using the Gazebo transport library.
This is the same mechanism that the Gazebo simulation client (the gzclient process) uses to communicate with the simulation server.
The source code for this project can be found here: <https://bitbucket.org/osrf/gazebojs>

[[file:files/gazebojs_overview.png|640px]]

This page explains how to install the GazeboJs Node bindings to Gazebo.




### Mac OSX (using homebrew)

This tutorial shows how to download, install and compile Gazebojs on a computer where Gazebo and its  development libraries are installed.
Please refer to the [Install Gazebo on Mac (using homebrew)](http://gazebosim.org/tutorials?tut=install_on_mac&cat=install).

#### Setup

The homebrew install of Gazebo also install the corresponding development libraries (for example, libgazeboi6-dev for Gazebo 6). The dev libraries also contain the Gazebo header files that are necessary for the gazebojs installation. This is because the NodeJs Gazebo modules are automatically compiled on your machine when the 'npm install gazebojs' is invoked (see below).
Like Gazebo, gazebojs module uses semantic versioning, so the major version of gazebojs should be the same as the major version of Gazebo you are using. You can specify rules about the version of gazebojs you want to use in the `package.json` file (see https://www.npmjs.org/doc/files/package.json.html).

Install nodejs and npm

    brew search node

This might give you the following results

    homebrew/versions/node012
    homebrew/versions/node010
    homebrew/versions/node08
    homebrew/versions/node06
    homebrew/versions/node04
    node nodebrew leafnode nodenv

Gazebo 6 is compatible with node 0.10 (which is the version of node that is bundled with Ubuntu 14.04). It works well with node v0.10.40 and npm 2.14.4 and Gazebo 6.4. Npm is the package manager for node, and it installs with node:

    brew install homebrew/versions/node010

To check your installed version:

    npm --version
    node --version
    gazebo --version


Install the required libraries

    brew install jpeg
    brew install pkg-config
    brew install jansson

Make sure that these packages can be found in your pkg-config path. This is because gazebojs uses pkg-config t.  You can check this by executing this command without any error:

    pkg-config --cflags gazebo jansson protobuf || echo "error :-("


#### Running

Now that everything is installed, here are the steps to test it:


Create a NodeJs project directory

    mkdir gz_node_inst
    cd gz_node_inst
    npm init

Npm will prompt you for details about your project. You can add information or press ENTER a few times. This step creates a package.json directory. Having a package.json file in your local directory will also ensure that npm will install all its packages in your local directory.

Install Gazebojs

     npm install gazebojs --save

This operation should download and compile the latest gazebojs. There is a C++
compilation phase where a NodeJs module is created. There should now be a
`node_modules` directory created in your project directory.


Test your installation:

Launch Gazebo in a separate terminal and verify that the simulation is running (when simulation is running, the Sim Time increases in the status bar in the bottom of the screen):

    gazebo


Use the 'node' command (from the same directory where you invoked the npm
command) to invoke the NodeJs REPL console

    node

Type in the following commands ('undefined' messages are NodeJS console valid
responses to your command) to load the gazeboJs module, create a
simulation client and pause the running simulation

    > var gazebojs = require('gazebojs')
    undefined
    > var sim = new gazebojs.Gazebo()
    undefined
    > sim.pause()
    undefined

You should see the simulation stop in Gazebo.

#### Troubleshooting


Problem: Npm error about Incompatible version

    npm http 304 https://registry.npmjs.org/which
    npm ERR! Error: No compatible version found: mkdirp@'^0.5.0'
    npm ERR! Valid install targets:
    npm ERR! ["0.0.1","0.0.2","0.0.3","0.0.4","0.0.5","0.0.6","0.0.7","0.1.0","0.2.0","0.2.1","0.2.2","0.3.0","0.3.1","0.3.2","0.3.3","0.3.4","0.3.5","0.4.0","0.4.1","0.4.2","0.5.0","0.5.1"]
    npm ERR!     at installTargetsError (/usr/local/lib/node_modules/npm/lib/cache.js:685:10)
    npm ERR!     at /usr/local/lib/node_modules/npm/lib/cache.js:607:10
    npm ERR!     at saved (/usr/local/lib/node_modules/npm/node_modules/npm-registry-client/lib/get.js:138:7)
    npm ERR!     at FSReqWrap.oncomplete (fs.js:82:15)


Solution: check that your npm version is above 1.4.3

    http://apple.stackexchange.com/questions/171530/how-do-i-downgrade-node-or-install-a-specific-previous-version-using-homebrew


Problem: missing jpeg


    > node-gyp rebuild

      CXX(target) Release/obj.target/canvas/src/Canvas.o
    In file included from ../src/Canvas.cc:19:
    ../src/JPEGStream.h:11:10: fatal error: 'jpeglib.h' file not found
    #include <jpeglib.h>
             ^
    1 error generated.
    make: *** [Release/obj.target/canvas/src/Canvas.o] Error 1
    gyp ERR! build error
    gyp ERR! stack Error: `make` failed with exit code: 2
    gyp ERR! stack     at ChildProcess.onExit (/usr/local/lib/node_modules/npm/node_modules/node-gyp/lib/build.js:267:23)
    gyp ERR! stack     at ChildProcess.EventEmitter.emit (events.js:98:17)
    gyp ERR! stack     at Process.ChildProcess._handle.onexit (child_process.js:789:12)
    gyp ERR! System Darwin 13.0.0

Solution:

    brew uninstall jpeg && brew install jpeg


Jpeg is also part of the xcode command line tools. You can try:

    xcode-select --install

It is good to try to use a new bash terminal after jpeg installation, in case the path to the library has not been set for the compiler/linker.

   https://github.com/Automattic/node-canvas/issues/348

Problem: linker error, ld: library not found for -lgcc_s.10.5

    > bcrypt@0.8.5 install /Users/Kyle/Documents/Software/mean/mean_scotch/node_modules/bcrypt
    > node-gyp rebuild

      CXX(target) Release/obj.target/bcrypt_lib/src/blowfish.o
      CXX(target) Release/obj.target/bcrypt_lib/src/bcrypt.o
      CXX(target) Release/obj.target/bcrypt_lib/src/bcrypt_node.o
      SOLINK_MODULE(target) Release/bcrypt_lib.node
    ld: library not found for -lgcc_s.10.5
    collect2: error: ld returned 1 exit status
    make: *** [Release/bcrypt_lib.node] Error 1
    gyp ERR! build error
    gyp ERR! stack Error: `make` failed with exit code: 2

Solution: add the missing libraries in your '/usr/local/lib' directory

    cd /usr/lib sudo ln -s ../lib/libSystem.B.dylib libgcc_s.10.5.dylib

Another (better) solution is to (re) install Xcode 7.

    https://github.com/nodejs/node/issues/2933


