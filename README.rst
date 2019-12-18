Description
===========

This is a naoqi driver module that bridges with ROS. It publishes
several sensor data as well as the robot position.

On the other hand it enables ROS to call parts of the
NAOqi API.

What it does
============

The **naoqi_driver** module is in charge of providing some
bridge capabilities between ROS and NAOqiOS.

How it works
============

The **naoqi_driver** module is a NAOqi module that also acts
as a ROS node. As there is no **roscore** on the robot, it
needs to be given the IP of the **roscore** in order to be
registered as a node in the ROS processing graph. Usually,
you will start your **roscore** on your local desktop.

Once connected, normal ROS communication is happening between
your robot, running NAOqi OS, and your desktop, running ROS.


For further information, you can go `here <http://ros-naoqi.github.io/naoqi_driver/>`_ or build the doc:

.. code-block:: sh

  cd doc
  doxygen Doxyfile
  sphinx-build -b html ./source/ ./build/


Travis - Continuous Integration
===============================

.. |indigo| image:: https://travis-matrix-badges.herokuapp.com/repos/ros-naoqi/naoqi_driver/branches/master/1
    :alt: Indigo with Ubuntu Trusty
    :target: https://travis-ci.org/ros-naoqi/naoqi_driver/

.. |kinetic| image:: https://travis-matrix-badges.herokuapp.com/repos/ros-naoqi/naoqi_driver/branches/master/2
    :alt: Kinetic with Ubuntu Xenial
    :target: https://travis-ci.org/ros-naoqi/naoqi_driver/

.. |melodic| image:: https://travis-matrix-badges.herokuapp.com/repos/ros-naoqi/naoqi_driver/branches/master/3
    :alt: Melodic with Ubuntu Bionic
    :target: https://travis-ci.org/ros-naoqi/naoqi_driver/

.. |melodic-stretch| image:: https://travis-matrix-badges.herokuapp.com/repos/ros-naoqi/naoqi_driver/branches/master/4
    :alt: Melodic with Debian Stretch
    :target: https://travis-ci.org/ros-naoqi/naoqi_driver/

+-----------------+---------------------+
|   ROS Release   |       status        |
+=================+=====================+
| Melodic-stretch |  |melodic-stretch|  |
+-----------------+---------------------+
| Melodic         |     |melodic|       |
+-----------------+---------------------+
| Kinetic         |     |kinetic|       |
+-----------------+---------------------+
| Indigo          |     |indigo|        |
+-----------------+---------------------+
