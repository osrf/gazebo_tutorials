# Introduction

In order to address the requirements and resolve the current issues with Gazebo,
we propose a distributed architecture.
Gazebo will be broken into libraries for physics simulation,
rendering, user interface, communication, and sensor generation.
Three different processes will be provided:
`physics_sim`, `sensor_gen`, `gui`, and a `master` for coordination.

## Communication Between Processes

Communication between each process will use a combination of google::protobufs and sockets.
A simulated world will publish body pose updates,
and sensor generation and GUI will consume these messages to produce output.

This mechanism will allow for introspection of a running simulation,
and provide a convenient mechanism to control aspects
of Gazebo.

## System 

### Gazebo Master

This is essentially a topic name server.
It provides namelookup, and topic management.
A single master can handle multiple physics simulations,
sensor generators, and GUIs.

### Communication Library

 * **Dependencies:** Protobuf and boost::ASIO
 * **External API:**
 * **Internal API:** None
 * **Advertised Topics:** None
 * **Subscribed Topics:** None
 
 This library is used by almost all subsequent libraries.
 It acts as the communication and transport mechanism for Gazebo.
 It currently supports only publish/subscribe, but it's possible to use RPC with minimal effort.
 
### Physics Library
 * **Dependencies:** Dynamics engine, and Collision Library
 * **External API:** Provides a simple and generic interface to physics simulation
 * **Internal API:** Defines a fundamental interface to the physics library for 3rd party dynamic engines.
 
 The physics library can use any dynamic engine that conforms to the internal API (TBD).
 It also presents a simple external interface in order to establish a work physics simulation.
 
### Collision Library

 * **Dependencies:** 3rd party collision engine
 * **External API:** TBD
 * **Internal API:** Generic interface for collision engines
 * **Advertised Topics:** None
 * **Subscribed Topics:** None

 This is an abstraction library to handle different collision engines,
 and provide a simple external interface to the user.

### Rendering Library

 * **Dependencies:** OGRE
 * **External API:** Allows for loading, initialization, and scene creation
 * **Internal API:** None, we are going to use only OGRE.

 The rendering library provides a simple interface to both the GUI and sensor generation.
 We are currently sticking with OGRE since we don't have a better alternative.
 It will be possible to write plugins for the rendering engine.

### Sensor Generation

 * **Dependencies:** Rendering Library, Collision Library
 * **External API:** Provide functionality to initialize and run a set of sensors
 * **Internal API:** TBD
 
 The sensor generation library implements all the various types of sensors,
 listens to world state updates from a physics simulator and
 produces output specified by the instantiated sensors.
 
### GUI

 * **Dependencies:** Rendering Library, wxWidgets
 * **External API:** None
 * **Internal API:** None

 The primary function of the GUI involves displaying the current state
 of the simulation and providing a convenient means for user input.
 There is no need for an internal or external API since we are sticking with wxWidgets.

### Plugins
 The physics, sensor, and rendering libraries will support plugins.
 These plugins will provide users with access to the respective libraries
 without using the communication system.
 
