# naoqi_driver

This module establishes a ROS bridge with NAOqi. It publishes several sensor data as well as the robot position.

It also enables ROS to call parts of the NAOqi API.
It was tested with a pepper robot.

## What it does

The __naoqi_driver__ module is in charge of providing some
bridge capabilities between ROS and NAOqiOS.

## Dependencies

To run, the driver requires the [`naoqi_libqi`](https://github.com/ros-naoqi/libqi), [`naoqi_libqicore`](https://github.com/ros-naoqi/libqicore) and [`naoqi_bridge_msgs`](https://github.com/ros-naoqi/naoqi_bridge_msgs) packages. Those can be installed using apt-get (if they have been released for your ROS distro) or from source. Additionally, [`pepper_meshes`](https://github.com/ros-naoqi/pepper_meshes) and/or [`nao_meshes`](https://github.com/ros-naoqi/nao_meshes) can be useful if you try to display the robot in RViz.

## How it works

The __naoqi_driver__ module is a NAOqi module that also acts
as a ROS node. Ros will automaticaly generate a roscore IP for the robot.
Usually, you will start your __roscore__ on your local desktop.

Once connected, normal ROS communication is happening between
your robot, running NAOqi OS, and your desktop, running ROS.

## Launch

Before launching, you may want to shutdown the autonomous life of the robot with the following process:

```sh
ssh nao@<naoip>
qicli call ALAutonomousLife.setState disabled
qicli call ALMotion.wakeUp
```

The driver can be launched using the following command:

Be aware that username and password arguments are only
required for robots running NAOqi 2.9 or greater.

```sh
source <catkin_ws>/devel/setup.bash
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<ip> nao_port:=<port> roscore_ip := <ip> network_interface:=<interface> username:=<name> password:=<passwd>
```

Warning: `naoqi_driver` for melodic and greater have to be used for robots
running NAOqi 2.9 and greater.

## Check that the node is running correctly

Check that naoqi_driver is connected with :

```sh
rosnode info /naoqi_driver
```

Check that you can move the head by publishing on `/joint_angles`:

```sh
rostopic pub /joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed "{header: {seq: 0, stamp: now, frame_id: ''}, joint_names: ['HeadYaw', 'HeadPitch'], joint_angles: [0.5,0.1], speed: 0.1, relative: 0}"
```

You can see the published message with `rostopic echo /joint_angles`

Check that you can move the robot by publishing on cmd_vel to make the robot move:

```sh
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.8"
```

## Build status

ROS Distro| Binary Status | Source Status | Github Build |
|-------------------|-------------------|-------------------|-------------------|
Noetic | [![Build Status](https://build.ros.org/job/Nbin_uf64__naoqi_driver__ubuntu_focal_amd64__binary/badge/icon)](https://build.ros.org/job/Nbin_uf64__naoqi_driver__ubuntu_focal_amd64__binary/) | [![Build Status](https://build.ros.org/job/Nsrc_uF__naoqi_driver__ubuntu_focal__source/badge/icon)](https://build.ros.org/job/Nsrc_uF__naoqi_driver__ubuntu_focal__source/) | [![ros-noetic-focal](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/noetic_focal.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/noetic_focal.yml)
Melodic | [![Build Status](https://build.ros.org/job/Mbin_ub64__naoqi_driver__ubuntu_bionic_amd64__binary/badge/icon)](https://build.ros.org/job/Mbin_ub64__naoqi_driver__ubuntu_bionic_amd64__binary/) | [![Build Status](https://build.ros.org/job/Msrc_uB__naoqi_driver__ubuntu_bionic__source/badge/icon)](https://build.ros.org/job/Msrc_uB__naoqi_driver__ubuntu_bionic__source/) | [![ros-melodic-bionic](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/melodic_bionic.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/melodic_bionic.yml)
Kinetic | ![passing](https://raw.githubusercontent.com/jenkinsci/embeddable-build-status-plugin/7c7eedc7617851f07a1f09629c33fee11cff50ab/src/doc/flat_unconfigured.svg) | ![passing](https://raw.githubusercontent.com/jenkinsci/embeddable-build-status-plugin/7c7eedc7617851f07a1f09629c33fee11cff50ab/src/doc/flat_unconfigured.svg) | [![ros-kinetic-xenial](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/kinetic_xenial.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/kinetic_xenial.yml) |

## Further information

For further information, you can consult the documentation (OUTDATED) [here](http://ros-naoqi.github.io/naoqi_driver/) or build it:

```sh
cd doc
doxygen Doxyfile
sphinx-build -b html ./source/ ./build/
```
