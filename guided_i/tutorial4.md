# Re-cap

We have created an SDF model of the Velodyne HDL-32 LiDAR that has visual
meshes and generates data with a noise model.

In this section, we will learn how to contribute the model to the online database.

# Contribute the model to the online-database

Contributing our model to Gazebo's online-databse benefits you and every
other user of Gazebo. When the model is hosted on the database, Gazebo will
automatically download it when requested. This means you don't have to
manage which computers have the model and which do not. Additionally, other
people can use your model.

1. Fork the `gazebo_models` database by visiting [https://bitbucket.org/osrf/gazebo_models/fork](https://bitbucket.org/osrf/gazebo_models/fork).

1. Clone your fork of model database.

    ```
    cd
    hg clone URL_OF_YOUR_FORK
    ```

1. Look at the directories in your cloned repository to make sure your model
   does not already exist.

1. Copy the model from `~/.gazebo/models` to the cloned repository.

    ```
    cp -r ~/.gazebo/models/velodyne-hdl32 ~/gazebo_models
    ```

1. Make a new branch, which will make the pull-request process a bit easier.

    ```
    cd ~/gazebo_models
    hg branch velodyne_tutorial_do_not_merge
    ```

1. Add, commit, and push your model.

    1. ```hg add velodyne*```

    1. ```hg commit -m "Added a Velodyne HDL-32 LiDAR"```

    1. ```hg push --new-branch```

1. Create a pull-request back to the main `gazebo_models` repository.

    1. Open the `URL_OF_YOUR_FORK` in a webbrowser

    1. Select `Create pull request` on the left

    1. Enter a title and description, and select the `Create Pull Request`
       button.

1. Two approvals of your pull request are required before it will be merged
   into the main `gazebo_models` repository. Please respond to any comments
   quickly in order to expedite the process.

# Next Up

The next tutorial in this series will add a plugin to the Velodyne sensor.
This plugin will control the rotation of the sensor's upper portion.

[Control plugin](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5)
