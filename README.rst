Description
===========

This is a NAOqi module that bridges with ROS. It publishes
several sensor data as well as the robot position.

On the other hand it enables ROS to call parts of the
NAOqi API.

What it does
============

The **ALRosBridge** module is in charge of providing some
bridge capabilities between ROS and NAOqiOS.

How it works
============

The **ALRosBridge** module is a NAOqi module that also acts
as a ROS node. As there is no **roscore** on the robot, it
needs to be given the IP of the **roscore** in order to be
registered as a node in the ROS processing graph. Usually,
you will start your **roscore** on your local desktop.

Once connected, normal ROS communication is happening between
your robot, running NAOqi OS, and your desktop, running ROS.


For further information, you can go `here <./doc/source/index.rst>`_ or build the doc:

.. code-block:: sh

  cd doc
  doxygen Doxyfile
  sphinx-build -b html ./source/ ./build/
