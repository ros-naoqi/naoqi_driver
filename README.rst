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


Build status
============

.. |i_log| image:: https://raw.github.com/ros/ros_tutorials/indigo-devel/turtlesim/images/indigo.png
    :width: 40
    :height: 45

.. |j_log| image:: https://raw.github.com/ros/ros_tutorials/jade-devel/turtlesim/images/jade.png
    :width: 40
    :height: 45

.. |k_log| image:: https://raw.github.com/ros/ros_tutorials/kinetic-devel/turtlesim/images/kinetic.png
    :width: 40
    :height: 45

.. |l_log| image:: https://raw.githubusercontent.com/ros/ros_tutorials/lunar-devel/turtlesim/images/lunar.png
    :width: 40
    :height: 45

.. |indigo| image:: https://travis-matrix-badges.herokuapp.com/repos/ros-naoqi/naoqi_driver/branches/readme_test/1
    :alt: Indigo with Ubuntu Trusty
    :target: https://travis-ci.org/ros-naoqi/naoqi_driver/

.. |jade| image:: https://travis-matrix-badges.herokuapp.com/repos/ros-naoqi/naoqi_driver/branches/readme_test/2
    :alt: Jade with Ubuntu Trusty
    :target: https://travis-ci.org/ros-naoqi/naoqi_driver/

.. |kinetic| image:: https://travis-matrix-badges.herokuapp.com/repos/ros-naoqi/naoqi_driver/branches/readme_test/3
    :alt: Kinetic with Ubuntu Xenial
    :target: https://travis-ci.org/ros-naoqi/naoqi_driver/

.. |lunar| image:: https://travis-matrix-badges.herokuapp.com/repos/ros-naoqi/naoqi_driver/branches/readme_test/4
    :alt: Lunar with Ubuntu Xenial
    :target: https://travis-ci.org/ros-naoqi/naoqi_driver/

+----------------+---------------+---------------+
| ROS Release    | Ubuntu Trusty | Ubuntu Xenial |
+================+===============+===============+
||l_log| Lunar   | N/A           | |lunar|       |
+----------------+---------------+---------------+
||k_log| Kinetic | N/A           | |kinetic|     |
+----------------+---------------+---------------+
||j_log| Jade    | |jade|        | N/A           |
+----------------+---------------+---------------+
||i_log| Indigo  | |indigo|      | N/A           |
+----------------+---------------+---------------+
