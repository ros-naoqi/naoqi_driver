.. _main menu:

Description
===========

This is a NAOqi module that bridges with ROS. It is written
in C++ and gets data straight from the lowest levels of
NAOqi hence ensuring low latency and CPU usage. It publishes
several sensor data as well as the robot position and tries
to be as close as possible to the ROS standard by exposing
several standard ROS topics (like /tf or /cmd_vel) as well as
custom ones.

Table of content
================

1. :ref:`How to install it`
2. :ref:`Getting started`
3. :ref:`API`
4. :ref:`Topic list`
5. :ref:`Troubleshooting`
6. :ref:`Next step`
7. :ref:`Extra use-cases`