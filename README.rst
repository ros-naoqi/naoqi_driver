Description
===========

This is a NAOqi module that bridges with ROS. It publishes
several sensor data as well as the robot position.

On the other hand it enables ROS to call parts of the
NAOqi API.

What it does
============

The **BridgeService** module is in charge of providing some bridge capabilities between ROS and NAOqiOS.

How it works
============

The **BridgeService** module is a NAOqi module that also acts as a ROS node. As there is no **roscore** on the robot, it needs to be given the IP of the **roscore** in order to be registered as a node in the ROS processing graph. Usually, you will start your **roscore** on your local desktop.

Once connected, normal ROS communication is happening between your robot, running NAOqi OS, and your desktop, running ROS.

Performances and limitations
============================

This module is in C++ and gets data straight from the lowest levels of NAOqi hence ensuring low latency and CPU usage. For the usual usage of using /tf and TODO, we observe a latency of TODO and a CPU usage increase of TODO.

When there are no subscribers connected to a specific topic, no CPU is used on the robot.

This module tries to be as close to the ROS standard by exposing several standard ROS topics (like /tf) as well as custom ones. Those topics are detailed on the Topics page.

It also exposes the following higher level NAOqi API services:

* /cmd_vel as detailed here http://wiki.ros.org/android_teleop
