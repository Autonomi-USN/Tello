<h1 align="center">
    <img alt="Tello Edu" ttle="Tello Edu" src="https://www.eduporium.com/media/catalog/product/cache/344839f5026348e9ff213e0be9a4da00/t/e/tello_edu_front.png" />
    <p>Tello ROS2 Wrapper</p>
</h1>

<p align="center">
  <a href="#about-the-project">About the Project</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#packages-description">Packages description</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#how-to-connect-with-drone">How to Connect with Drone</a><br />
  <a href="#dependencies">Dependencies</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#how-to-run">How to Run</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#troubleshooting">Troubleshooting</a>&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
  <a href="#possible-bugs">References</a>
</p>

<br />

## About the Project

This project has as objective to create a simple ROS Wrapper for the [DJI Tello Edu](https://djioslo.no/produkt/tello/tello-edu/) drone, enabling the programmers to control the drone through ROS framework. The packages are based on a Python framework.

## Packages description

### tello_interfaces

Messages, services and actions related to the Tello operations.

### tello_bridge

The main package, it generates the node that process commands and logs from Tello.

## How to Connect with Drone

To connect the drone with the device where the ROS wrapper will run, just follow the next steps:

- Turn on the Drone
- Connect with the Drone wifi, with format TELLO-XXXXXX. The 6 digits code can be found on the back side of the drone
- Enjoy :)

## Dependencies

Some python packages are needed to be able to run this wrapper. To install it, just use the following command

```
$ pip install opencv-python av
```

## How to Run

The first step after installing all the dependencies in the previous section is to clone the repository and build the packages.

### Set-up the environment

First, download/clone this repository, then copy the packages inside the _tello_ folder into the source of your ROS workspace.

If a ROS workspace created was not created yet, just run the following commands:

```
$ cd ~
$ mkdir -p ~/ros2_ws/src
```

Now paste the packages inside ~/ros2_ws/src. After that, use rosdep to install all the ROS dependencies.

```
$ cd ~/ros2_ws/
$ rosdep install --from-paths src --ignore-src -r -y
```

Now we can build.

### Build

```
$ cd ~/ros2_ws/
$ colcon build
```

### Run Wrapper

```
$ cd ~/ros2_ws/
$ source install/setup.bash
$ ros2 run tello_bridge tello_bridge_node
```

After that, **/tello_bridge_node** is created. This node publishes all the telemetry data in the **/tello_data** topic.

This node also have a subscriber, named **/cmd_vel**, used to move the robot. you need to publish a Twist message, with the values of translation and rotation of the drone.

## Troubleshooting

### Take off / land command is not working

Battery may be too low
Wifi may be disconnected

## References

Subscribes:
/cmd_vel

Publishes:
/tello_data
/tello_image
