# Introduction

Gazebo uses a distributed architecture
with separate libraries for physics simulation,
rendering, user interface, communication, and sensor generation.
Additionally, gazebo provides two executable programs for running simulations,
a server (gzserver) for simulating the physics, rendering, and sensors
and a client (gzclient) that provides a graphical interface to
visualize and interact with the simulation.
The client and server communicate using the gazebo communication library.

## Communication Between Processes

The communication library currently uses the open source
Google Protobuf for the message serialization
and boost::ASIO for the transport mechanism.
It supports the publish/subscribe communication paradigm.
For example, a simulated world publishes body pose updates,
and sensor generation and GUI will consume these messages to produce output.

This mechanism allow for introspection of a running simulation,
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
 It currently supports only publish/subscribe,
 but it is possible to use RPC with minimal effort.
 
### Physics Library
 * **Dependencies:** Dynamics engine, and Collision Library
 * **External API:** Provides a simple and generic interface to physics simulation
 * **Internal API:** Defines a fundamental interface to the physics library for 3rd party dynamic engines.
 
 The physics library provides a simple and generic interface to
 fundamental simulation components, including rigid bodies,
 collision shapes, and joints for representing articulation
 constraints.
 This interface has been integrated with four open-source
 physics engines: Open Dynamics Engine (ODE), Bullet,
 Simbody, and Dynamics, Animation, and Robotics Toolkit (DART).
 A model described in the Simulation Description Format (SDF)
 using XML can be loaded by each of these physics engines.
 This provides access to different algorithm implementations
 and simulation features.

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

 The rendering library uses OGRE to provide a simple interface
 for rendering 3D scenes to both the GUI and sensor libraries.
 It includes lighting, textures, and sky simulation.
 It is possible to write plugins for the rendering engine.

### Sensor Generation

 * **Dependencies:** Rendering Library, Collision Library
 * **External API:** Provide functionality to initialize and run a set of sensors
 * **Internal API:** TBD
 
 The sensor generation library implements all the various types of sensors,
 listens to world state updates from a physics simulator and
 produces output specified by the instantiated sensors.
 
### GUI

 * **Dependencies:** Rendering Library, QT
 * **External API:** None
 * **Internal API:** None

 The GUI library uses QT to visualize the simulation and allow user interaction.
 The user may control the flow of time by pausing or changing time step size
 via GUI widgets.
 The user may also modify the scene by adding, modifying, or removing models.
 Additionally there are some tools for visualizing and
 logging simulated sensor data.

### Plugins
 The physics, sensor, and rendering libraries support plugins.
 These plugins provide users with access to the respective libraries
 without using the communication system.
 
