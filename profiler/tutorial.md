# Profiler

Gazebo 11 uses the [Ignition Common Profiler](https://ignitionrobotics.org/api/common/3.5/profiler.html) to check and visualize the performance of the different threads, functions and methods.

Gazebo 9 uses an internal fork of Ignition Common's profiler.

## How to run and see the profiler results for Gazebo?

To use the compiler, Gazebo must be compiled from source with the CMake flag
`-DENALE_PROFILER=1`, for example:

```bash
cmake .. --cmake-args -DENABLE_PROFILER=1
```

For example, if you are using `colcon` to compile Gazebo:

```bash
colcon build --cmake-args -DENABLE_PROFILER=1
```

After compiling Gazebo, launch the world you want to profile. For example,

```
gazebo profiler.world
```

There's a convenient launcher script (Linux and macOS) for starting Remotery.

* On Gazebo 11:

    ```
    ign_remotery_vis
    ```

* On Gazebo 9:

    ```
    gz_remotery_vis
    ```

The script should open the profiler output in a browser.

Profile data for each process goes to a different port, which can be chosen on
the top of the page:

* `gzserver` to port 1500
* `gzclient` to port 1501

You should see plots corresponding to different theads, for example:

* `gzserver`
* `[Ode,Bullet,Symbody,Dart]Physics`
* `SensorManager`

The following image shows the sensor manager timeline. In this case we
have 5 different sensors: `gps`, `imu`, `magnetometer`, `altimeter` and `contact`.
You can see what is the duration of the sensor manager loop (50ms) and
how long it takes to update all the sensors (around 0.177ms), the rest
of the time the thead will be sleeping.

You can see the update time of each sensor, all the sensors have two
subtasks: generate data and then publish this data (fill the data
structure and publish it).

[[file:files/SensorManagerLoop.png|600px]]

In the following two images you can see the evolution of the `OdePhysics`
and `gzserver` loops.

[[file:files/Remotery_OdePhysics.png|600px]]

[[file:files/gzserverLoop.png|600px]]

## How can I add the profiler to custom Gazebo plugins?

Using Gazebo 11, you can add profiler points to any Gazebo plugin. This
is not possible on Gazebo 9.

If you want to add the profiler to your own Gazebo plugins you need to
follow these steps:

* Include the header

    ```cpp
     #include <ignition/common/Profiler.hh>
    ```

* Link against the profiler library

    ```cpp
    target_link_libraries(<your plugin name>
        ...
        ${IGNITION-COMMON_LIBRARIES}
    )
    ```

* Add profiler macros to the update method or other periodic methods:

    ```cpp
    IGN_PROFILE("WindPlugin::OnUpdate");
    IGN_PROFILE_BEGIN("Update");
    ...
    IGN_PROFILE_END();
    ```

### Sensor classes

Classes that inherit from `gazebo::sensors::Sensor` already have the macros
in the `UpdateImpl` method. This method is called periodically and it's in
charge of generating the data and publishing it.

### Plugins

Classes that inherit from `gazebo::*Plugin` can connect to the periodic world
update event with `gazebo::event::Events::ConnectWorldUpdateBegin`. See the
`WindPlugin` for example, this will call the `WindPlugin::OnUpdate` method for
every simulation iteration:

```cpp
this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WindPlugin::OnUpdate, this));
```

That callback is a good place to put the macros above.
