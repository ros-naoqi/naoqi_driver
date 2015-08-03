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

.. toctree::
  :numbered:

  install
  start
  api
  topics
  trouble
  next
  other
