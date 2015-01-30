What it does
------------

The **ALROS** module is in charge of providing some bridge capabilities between `ROS <http://ros.org/>`_ and NAOqiOS.

How it works
------------

The **ALROS** module is a NAOqi module that also acts as a ROS node. As there is no **roscore** on the robot, it needs to be given the IP of the **roscore** in order to be registered as a node in the ROS processing graph. Usually, you will start your roscore on your local desktop.

Once connected, normal ROS communication is happening between your robot, running NAOqi OS, and your desktop, running ROS.

Performance and limitations
---------------------------

This module is in C++ and gets data straight from the lowest levels of NAOqi hence ensuring low latency and CPU usage. For the usual usage of using ``/tf`` and TODO, we observe a latency of ``TODO`` and a CPU usage increase of TODO.

When there are no subscribers connected to a specific topic, no CPU is used on the robot.

This module tries to be as close to the ROS standard. The following topics are being published:

- ``/tf  [tf2_msgs/TFMessage]``: http://wiki.ros.org/tf#Published_Topics
- ``/joint_states [sensor_msgs/JointState]``: http://wiki.ros.org/joint_state_publisher#Published_Topics
- ``/rosout [rosgraph_msgs/Log]``: http://wiki.ros.org/rosout#rosout_topic
- ``/alrosconverter/string_pub [std_msgs/String]`` TODO
- ``/alrosconverter/int_pub [std_msgs/Int32]`` TODO
- TODO

The following services are also exposed to bridge with the NAOqi API:

- ``/naoqi/set_life_status`` to turn the life behavior on or off
- ``/naoqi/move_to``
- ``/naoqi/set_position_to``
- ``/naoqi/start_asr`` and ``/naoqi/stop_asr``
- ``/naoqi/speak`` (TODO: look at audio common)

Getting Started
---------------

Starting ROS on your machine
++++++++++++++++++++++++++++

First of all, you need to have ROS installed on your machine. Please follow the instructions at http://wiki.ros.org/ROS/Installation .

First, get yourself in a ROS workspace by sourcing your ``setup.sh``:

.. code-block:: sh

  source /opt/ros/you_installed_rosdistro/setup.sh

You can then start the **roscore** as explained at http://wiki.ros.org/roscore

.. code-block:: sh

  $ roscore

And you should then see the following output:

.. code-block:: sh

  started roslaunch server http://MY_MACHINE:53971/
  ros_comm version 1.11.10
  
  
  SUMMARY
  =======
  
  PARAMETERS
   * /rosdistro: indigo
   * /rosversion: 1.11.10
  
  NODES
  
  auto-starting new master
  process[master]: started with pid [13303]
  ROS_MASTER_URI=http://ALD-1270-LA:11311/
  
  setting /run_id to 7b0eaf26-a718-11e4-a3f8-f01faf464728
  process[rosout-1]: started with pid [13316]
  started core service [/rosout]

You also need to launch your robot description:

.. code-block:: sh

  $ roslaunch nao_description nao_desc_generated.launch

Starting the **ALROS** module
+++++++++++++++++++++++++++++

In the future, this module will be started by default.

.. code-block:: sh

  $ cd kk
  $ source toolchain/install/setup.sh
  $ ./bin/naoqi-bin

In a different terminal:

.. code-block:: sh

  $ ./bin/alros_bin


Triggering the **ALROS** module
+++++++++++++++++++++++++++++++

In order to get the module to connect to your roscore, you should send it your IP.
Let us assume your IP is ``10.0.132.105`` and your port ``11311``.

If oyu are on your desktop:

.. code-block:: sh

  $ rosrun local_naoqi_module local_executable http://10.0.132.105:11311

You can also perform that action from your robot:

.. code-block:: sh

  $ qicli call BridgeService.setMasterURI http://10.0.132.105:11311

Using the **ALROS** module
++++++++++++++++++++++++++

On your desktop, you can then use ROS as you would normally do:

.. code-block:: sh

  $ source /opt/ros/your_installed_rosdistro/setup.sh
  $ rostopic info alrosconverter

And you will get the following output:

.. code-block:: sh

  $ rosnode info /alrosconverter 
  --------------------------------------------------------------------------------
  Node [/alrosconverter]
  Publications: 
   * /alrosconverter/string_pub [std_msgs/String]
   * /joint_states [sensor_msgs/JointState]
   * /rosout [rosgraph_msgs/Log]
   * /tf [tf2_msgs/TFMessage]
   * /alrosconverter/int_pub [std_msgs/Int32]
  
  Subscriptions: None
  
  Services: 
   * /alrosconverter/get_loggers
   * /alrosconverter/set_logger_level
  
  
  contacting node http://10.0.132.89:44869/ ...
  Pid: 9678
  Connections:
   * topic: /rosout
      * to: /rosout
      * direction: outbound
      * transport: TCPROS
    
Troubleshooting
---------------

The robot cannot connect to the roscore
+++++++++++++++++++++++++++++++++++++++

Try out the following solutions:

- make sure you are on a local network
- check the IP you are giving: make sure it has the format TODO
- check you can ping the roscore IP from the robot

ROS gets delayed data
+++++++++++++++++++++

This is due to a difference of time between your robot and your desktop.
In order to synchronize the two, you need to update the NTP server on both:

.. code-block:: sh

  TODO


Additional Resources
--------------------

ROS
+++
For any ROS question, please refer to the official doc at http://wiki.ros.org .

Compiling
+++++++++

Those instructions are internal to Aldebaran for now.

To compile the module, you first need to get ROS in your toolchain. Get the toolchain file from https://gitlab.aldebaran.lan/kknese/ros-toolchain/tree/master .

Then execute the proper instruction to add it to your toolchain, e.g.:

.. code-block:: sh

  $ qitoolchain add-package -c atom ros toolchain_install_atom.tar.gz

TODO: fix compilation error ROS

Get the code from gitlab:

.. code-block:: sh

  $ git clone git@gitlab.aldebaran.lan:kknese/alrosconverter.git
  $ qisrc add ./alrosconverter
  $ qibuild configure -c atom alrosconverter
  $ qibuild make -c atom alrosconverter
