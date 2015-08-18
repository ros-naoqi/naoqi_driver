.. _start:

Getting Started
===============

Before starting the module
--------------------------

To start using this bridge, you need to start a roscore locally on our desktop computer and then, to communicate your external roscore IP on your robot.
Start a ROScore (on your desktop machine) ::

  $ ROS_IP=<yourDesktopIP> roscore

The ROS_IP parameter is optional in case your local network supports a fully functional DNS. You can easily verify this when you can ping your desktop computer from your robot. In case this fails, set the ROS_IP parameter so that we can later tell the robot how to find and connect to this roscore.


Start the naoqi driver module
--------------------------------

Please execute the following steps on your local computer, not from the robot

.. toggle_table::
  :arg1: catkin
  :arg2: qibuild

.. toggle:: catkin

  In case you built via catkin, you can alternatively start the naoqi driver via rosrun ::

  $ rosrun naoqi_driver naoqi_driver_node --qi-url=tcp://<yourRobotIP>:9559 --roscore_ip <roscore_ip> --network_interface <network_interface>

  The second parameter is available from version 1.2 (or compilation from source). Starting from version 1.2 (or compilation from source) you can alternatively start the bridge via a launch file ::

  $ roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<yourRobotIP> roscore_ip:=<roscore_ip> network_interface:=<eth0|wlan0|tethering|vpn...>

.. toggle:: qibuild

  In case you have a qibuild workspace, you can run the roscore by directly executing the binary ::

  $ path/to/naoqi_driver_node --qi-url=tcp://<yourRobotIP>:9559 --roscore_ip <roscore_ip> --network_interface <network_interface>


The roscore IP is the IP of the computer where the roscore is running. This command is optional. If you don't specify, any ROS communication is disabled until you call setMasterURI. The second parameter ``network_interface`` specifies the network interface the bridge is connected. By default, this is set to ``eth0``. Changing this becomes important to establish a correct network connection between robot and computer, when you are connected via a different network device, such as wlan0 or vpn0 etc. In case you are not certain which device to use, verify with ``ifconfig`` and verify which network device has the correct IP.

Set the roscore IP manually
---------------------------

In case you started the naoqi driver without ROS communication (saying without the roscore_ip parameter), you can do so while the bridge is running.
For this, open a SSH connection onto your robot and execute the following command **on** the robot ::

  $ qicli call ROS-Driver.setMasterURI http://<yourDesktopIP>:11311

If you connect to the robot not through ethernet, but an alternative interface such as wlan0, tethering or VPN, you need to specify the network interface ::

  $ qicli call ROS-Driver.setMasterURINet http://<yourDesktopIP>:11311 <network_interface>

For instance, if the robot use tethering (and ``tether`` shows when doing ``ifconfig``), use ::

  $ qicli call ROS-Driver.setMasterURINet http://<yourDesktopIP>:11311 tether

Check that it is running correctly
----------------------------------

For an easy check that all components are started and correctly running, you should see the following topics being available on your roscore.

.. toggle_table::
  :arg1: NAO
  :arg2: Pepper

.. toggle:: NAO

  For NAO, the following topics should be available ::

    $ rosnode info /nao_robot
    Node [/nao_robot]
    Publications:
    * /tf [tf2_msgs/TFMessage]
    * /nao_robot/sonar/left [sensor_msgs/Range]
    * /nao_robot/camera/front/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /nao_robot/camera/bottom/camera_info [sensor_msgs/CameraInfo]
    * /nao_robot/camera/bottom/image_raw [sensor_msgs/Image]
    * /joint_states [sensor_msgs/JointState]
    * /rosout [rosgraph_msgs/Log]
    * /nao_robot/camera/bottom/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
    * /nao_robot/info [naoqi_bridge_msgs/StringStamped]
    * /nao_robot/camera/bottom/image_raw/compressed [sensor_msgs/CompressedImage]
    * /nao_robot/camera/front/image_raw [sensor_msgs/Image]
    * /nao_robot/camera/front/camera_info [sensor_msgs/CameraInfo]
    * /nao_robot/camera/bottom/image_raw/theora [theora_image_transport/Packet]
    * /nao_robot/camera/front/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
    * /nao_robot/camera/bottom/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
    * /nao_robot/audio [naoqi_bridge_msgs/AudioBuffer]
    * /nao_robot/camera/bottom/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /nao_robot/imu/torso [sensor_msgs/Imu]
    * /nao_robot/camera/bottom/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /nao_robot/camera/front/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
    * /nao_robot/camera/front/image_raw/theora [theora_image_transport/Packet]
    * /nao_robot/camera/front/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /nao_robot/camera/front/image_raw/compressed [sensor_msgs/CompressedImage]
    * /nao_robot/sonar/right [sensor_msgs/Range]
    * /diagnostics_agg [diagnostic_msgs/DiagnosticArray]

    Subscriptions:
    * /move_base_simple/goal [geometry_msgs/PoseStamped]
    * /cmd_vel [unknown type]

    Services:
    * /nao_robot/camera/front/image_raw/theora/set_parameters
    * /nao_robot/camera/bottom/image_raw/theora/set_parameters
    * /nao_robot/set_logger_level
    * /nao_robot/camera/front/image_raw/compressed/set_parameters
    * /nao_robot/camera/bottom/image_raw/compressed/set_parameters
    * /nao_robot/get_loggers
    * /nao_robot/camera/bottom/image_raw/compressedDepth/set_parameters
    * /nao_robot/camera/front/image_raw/compressedDepth/set_parameters


.. toggle:: Pepper

  For Pepper, you should see the following topics::

    $ rosnode info /pepper_robot
    Node [/pepper_robot]
    Publications:
    * /pepper_robot/imu/base [sensor_msgs/Imu]
    * /pepper_robot/camera/ir/image_raw/theora [theora_image_transport/Packet]
    * /tf [tf2_msgs/TFMessage]
    * /pepper_robot/camera/bottom/image_raw/theora [theora_image_transport/Packet]
    * /pepper_robot/camera/ir/camera_info [sensor_msgs/CameraInfo]
    * /pepper_robot/camera/depth/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /pepper_robot/camera/ir/image_raw/compressed [sensor_msgs/CompressedImage]
    * /pepper_robot/camera/depth/image_raw [sensor_msgs/Image]
    * /pepper_robot/camera/bottom/image_raw/compressed [sensor_msgs/CompressedImage]
    * /pepper_robot/camera/front/image_raw/theora [theora_image_transport/Packet]
    * /pepper_robot/audio [naoqi_bridge_msgs/AudioBuffer]
    * /pepper_robot/camera/bottom/camera_info [sensor_msgs/CameraInfo]
    * /pepper_robot/camera/depth/image_raw/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /pepper_robot/camera/front/camera_info [sensor_msgs/CameraInfo]
    * /joint_states [sensor_msgs/JointState]
    * /rosout [rosgraph_msgs/Log]
    * /pepper_robot/camera/depth/image_raw/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
    * /pepper_robot/camera/bottom/image_raw [sensor_msgs/Image]
    * /pepper_robot/sonar/front [sensor_msgs/Range]
    * /pepper_robot/camera/depth/camera_info [sensor_msgs/CameraInfo]
    * /pepper_robot/camera/front/image_raw [sensor_msgs/Image]
    * /pepper_robot/camera/bottom/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /pepper_robot/camera/front/image_raw/compressed [sensor_msgs/CompressedImage]
    * /pepper_robot/camera/depth/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
    * /pepper_robot/laser [sensor_msgs/LaserScan]
    * /pepper_robot/camera/front/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /pepper_robot/imu/torso [sensor_msgs/Imu]
    * /pepper_robot/camera/ir/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /pepper_robot/camera/depth/image_raw/theora [theora_image_transport/Packet]
    * /pepper_robot/camera/ir/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /pepper_robot/camera/front/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /pepper_robot/camera/bottom/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
    * /pepper_robot/info [naoqi_bridge_msgs/StringStamped]
    * /pepper_robot/camera/front/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
    * /pepper_robot/camera/depth/image_raw/compressed [sensor_msgs/CompressedImage]
    * /pepper_robot/camera/bottom/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
    * /pepper_robot/camera/bottom/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /pepper_robot/camera/ir/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
    * /pepper_robot/camera/ir/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
    * /pepper_robot/sonar/back [sensor_msgs/Range]
    * /pepper_robot/camera/depth/image_raw/compressedDepth [sensor_msgs/CompressedImage]
    * /pepper_robot/camera/front/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
    * /pepper_robot/camera/ir/image_raw [sensor_msgs/Image]
    * /pepper_robot/camera/depth/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
    * /pepper_robot/camera/depth/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
    * /diagnostics_agg [diagnostic_msgs/DiagnosticArray]

    Subscriptions:
    * /move_base_simple/goal [geometry_msgs/PoseStamped]
    * /cmd_vel [unknown type]

    Services:
    * /pepper_robot/get_loggers
    * /pepper_robot/camera/front/image_raw/theora/set_parameters
    * /pepper_robot/camera/bottom/image_raw/compressed/set_parameters
    * /pepper_robot/camera/depth/image_raw/compressed/set_parameters
    * /pepper_robot/camera/bottom/image_raw/theora/set_parameters
    * /pepper_robot/camera/depth/image_raw/theora/set_parameters
    * /pepper_robot/camera/front/image_raw/compressedDepth/set_parameters
    * /pepper_robot/camera/bottom/image_raw/compressedDepth/set_parameters
    * /pepper_robot/set_logger_level
    * /pepper_robot/camera/front/image_raw/compressed/set_parameters
    * /pepper_robot/camera/depth/image_raw/compressedDepth/set_parameters
    * /pepper_robot/camera/ir/image_raw/compressedDepth/set_parameters
    * /pepper_robot/camera/ir/image_raw/theora/set_parameters
    * /pepper_robot/camera/ir/image_raw/compressed/set_parameters

This module also provides an API on the robot side to:
 * Publish data
 * Record data
 * Send command to naoqi modules

You can find more about it on the :ref:`API page <api>`, or you can go back to the :ref:`index <main menu>`.
