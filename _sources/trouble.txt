.. _trouble:

Troubleshooting
===============

The robot cannot connect to the roscore
---------------------------------------

Try out the following solutions:

- make sure the roscore is completely started (wait for the "started core service" message)
- check you can ping the roscore IP from the robot
- check the IP you are giving: make sure it is the IP where the roscode run, check that you did not forget http:// and port number


ROS gets delayed data
---------------------

This is due to a difference of time between your robot and your desktop.
In order to synchronize the two, you need to update the NTP server on both:

.. code-block:: sh

  TODO


Go back to the :ref:`index <main menu>`.