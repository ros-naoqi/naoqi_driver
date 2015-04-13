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

Getting Started
===============

Start the **BridgeService** module
----------------------------------

This module is provided in a binary package `here <https://gitlab.aldebaran.lan/ros/bridgeservicepackage/tree/master>`_

See instructions on installation `here <https://sites.google.com/a/aldebaran-robotics.com/ros/home/2-installation>`_ (Section *C++ Bridge*)

Using the **BridgeService** module
----------------------------------

To start using this bridge, you need to communicate your external roscore IP (see instructions `here <https://sites.google.com/a/aldebaran-robotics.com/ros/home/start-core-bridge>`_ )

API
+++

* startPublishing():

  start/enable publishing all registered publisher
  
* stopPublishing():

  stop/disable publishing all registered publisher

* addMemoryConverters(filePath):

  add some new converters for memory keys. This call requires as argument the path to a JSON file structured as the following one.
  memKeys and topic must be present and filled. Frequency is optional, and if not there, the default value is 10 Hz.

::

  {
      "memKeys": [
                  "KeyName1",
                  "KeyName2"
                 ],
      "topic": "topicName",
      "frequency": 10
  }
