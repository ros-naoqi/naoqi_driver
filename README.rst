Description
===========

This is a NAOqi module that bridges with ROS. It publishes
several sensor data as well as the robot position.

On the other hand it enables ROS to call parts of the
NAOqi API.

It is necessary to include ROS into your toolchain.
You can check out the toolchain packages under:
git@gitlab.aldebaran.lan:kknese/ros-toolchain.git
(!) As for now, there are pre-compiled packages for atom and linux64

TODO
====
- write down instructions on how to deploy that module in that README.rst file
- write down instructions on how to use it in an official qidoc .rst file. This package will be official so it needs to be documented from the very beginning
- mention, in the docs, the problem with NTP on the robot: you need to synchronize it before
- check how DCM access is done with B-Human: https://github.com/bhuman/BHumanCodeRelease/blob/master/Src/libbhuman/bhuman.cpp#L768
