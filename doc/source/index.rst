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

1. `How to install it <install.rst>`_
2. `Getting started <start.rst>`_
3. `API <api.rst>`_
4. `Topic list <topics.rst>`_
5. `Troubleshooting <trouble.rst>`_
6. `Next step <next.rst>`_
7. `Extra use-cases <other_usage.rst>`_