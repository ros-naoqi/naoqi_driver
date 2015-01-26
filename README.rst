Description
===========

This is a NAOqi module that bridges with ROS. It publishes
several sensor data as well as the robot position.

On the other hand it enables ROS to call parts of the
NAOqi API.

TODO
====

- get the toolchain in https://gitlab.aldebaran.lan/kknese/ but not in this repo
- write down instructions on how to install that toolchain and compile that module in the README.rst file
- write down instructions on how to deploy that module in that README.rst file
- write down instructions on how to use it in an official qidoc .rst file. This package will be official so it needs to be documented from the very beginning
- mention, in the docs, the problem with NTP on the robot: you need to synchronize it before
- check how DCM access is done with B-Human: https://github.com/bhuman/BHumanCodeRelease/blob/master/Src/libbhuman/bhuman.cpp#L768
