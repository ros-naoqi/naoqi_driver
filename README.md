# naoqi_driver

This module establishes a ROS bridge with naoqi. It publishes several sensor data as well as the robot position.

It also enables ROS to call parts of the NAOqi API.

## What it does

The __naoqi_driver__ module is in charge of providing some
bridge capabilities between ROS and NAOqiOS.

## How it works

The __naoqi_driver__ module is a NAOqi module that also acts
as a ROS node. As there is no __roscore__ on the robot, it
needs to be given the IP of the __roscore__ in order to be
registered as a node in the ROS processing graph. Usually,
you will start your __roscore__ on your local desktop.

Once connected, normal ROS communication is happening between
your robot, running NAOqi OS, and your desktop, running ROS.


For further information, you can consult the documentation (OUTDATED) [here](http://ros-naoqi.github.io/naoqi_driver/) or build it:

```sh
cd doc
doxygen Doxyfile
sphinx-build -b html ./source/ ./build/
```

## Dependencies
To run, the driver requires the `naoqi_libqi`, `naoqi_libqicore` and `naoqi_bridge_msgs` packages. Those can be installed using apt-get (if they have been released for your ROS distro) or from source. Additionally, `pepper_meshes` and/or `nao_meshes` can be useful if you try to display the robot in RViz.

## Launch
The driver can be launched using the following command:
```sh
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<ip> nao_port:=<port> network_interface:=<interface> username:=<name> password:=<passwd>
```
Note that the username and password arguments are only required for robots running naoqi 2.9 or greater 

:warning: naoqi_driver for melodic and greater have to be used for robots running naoqi 2.9 and greater

## Build status

ROS Distro| Binary Status | Source Status | Github Build |
|-------------------|-------------------|-------------------|-------------------|
Noetic | [![Build Status](https://build.ros.org/job/Nbin_uf64__naoqi_driver__ubuntu_focal_amd64__binary/badge/icon)](https://build.ros.org/job/Nbin_uf64__naoqi_driver__ubuntu_focal_amd64__binary/) | [![Build Status](https://build.ros.org/job/Nsrc_uF__naoqi_driver__ubuntu_focal__source/badge/icon)](https://build.ros.org/job/Nsrc_uF__naoqi_driver__ubuntu_focal__source/) | [![ros-noetic-focal](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/noetic_focal.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/noetic_focal.yml)
Melodic | [![Build Status](https://build.ros.org/job/Mbin_ub64__naoqi_driver__ubuntu_bionic_amd64__binary/badge/icon)](https://build.ros.org/job/Mbin_ub64__naoqi_driver__ubuntu_bionic_amd64__binary/) | [![Build Status](https://build.ros.org/job/Msrc_uB__naoqi_driver__ubuntu_bionic__source/badge/icon)](https://build.ros.org/job/Msrc_uB__naoqi_driver__ubuntu_bionic__source/) | [![ros-melodic-bionic](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/melodic_bionic.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/melodic_bionic.yml)
Kinetic | ![passing](https://raw.githubusercontent.com/jenkinsci/embeddable-build-status-plugin/7c7eedc7617851f07a1f09629c33fee11cff50ab/src/doc/flat_unconfigured.svg) | ![passing](https://raw.githubusercontent.com/jenkinsci/embeddable-build-status-plugin/7c7eedc7617851f07a1f09629c33fee11cff50ab/src/doc/flat_unconfigured.svg) | [![ros-kinetic-xenial](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/kinetic_xenial.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver/actions/workflows/kinetic_xenial.yml) |


