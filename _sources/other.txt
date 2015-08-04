.. _other:

Additional Resources
====================

ROS
---
For any ROS question, please refer to the official doc at http://wiki.ros.org .


Android Teleop
--------------

First, install the teleop app on your Android after installing rosjava and android_apps at http://wiki.ros.org/rosjava
(or ask Karsten for the ``.apk``)

When starting your roscore and this naoqi driver, make sure you export your ``ROS_IP`` and ``ROS_MASTER_URI`` to your IP.

Then start motion on your robot:

.. code-block:: sh

  $ qicli call ALMotion.wakeUp
  $ qicli call ALRobotPosture.goToPosture Stand 1

You can now control your robot with the app.


Go back to the :ref:`index <main menu>`.
