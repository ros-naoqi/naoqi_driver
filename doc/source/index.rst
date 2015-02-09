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

This module tries to be as close to the ROS standard by exposing several standard ROS topics (like ``/tf``) as well as custom ones. Those topics are detailed on the :ref:`topics` page.

It also exposes the following higher level NAOqi API services:

- TODO
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

You then need to launch your robot description:

.. code-block:: sh

  $ roslaunch nao_description nao_desc_generated.launch

which also spawns a **roscore** as explained at http://wiki.ros.org/roscore.

Starting the **ALROS** module
+++++++++++++++++++++++++++++

In the future, this module will be started by default. But for now, you need to start it yourself. On your robot create
you have to create a folder to work in:

.. code-block:: sh

  $ mkdir ~/ros

Then copy the file at ``https://gitlab.aldebaran.lan/kknese/ros-toolchain/tree/master/atom`` on your robot and unzip it:

.. code-block:: sh

  $ unzip toolchain_install_atom.zip -d ./toolchain_install/

Deploy the ROS bridge in ``~/ros``. And in a different terminal, launch that executable to register your module:

.. code-block:: sh

  $ source toolchain_install/setup.bash
  $ ./bin/alros_bin

Triggering the **ALROS** module
+++++++++++++++++++++++++++++++

In order to get the module to connect to your roscore, you should send it your IP.
Let us assume your IP is ``10.0.132.105`` and your port ``11311``.

If you are on your desktop:

.. code-block:: sh

  $ TODO rosrun local_naoqi_module local_executable http://10.0.132.105:11311

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

Get the code from gitlab:

.. code-block:: sh

  $ git clone git@gitlab.aldebaran.lan:kknese/alrosconverter.git
  $ qisrc add ./alrosconverter
  $ qibuild configure -c atom alrosconverter
  $ qibuild make -c atom alrosconverter
