.. _start:

Getting Started
===============

Before starting the module
--------------------------

To start using this bridge, you need to start a roscore locally on our desktop computer and then, to communicate your external roscore IP on your robot.
Start a ROScore (on your desktop machine) ::

  $ ROS_IP=<yourDesktopIP> roscore

The ROS_IP parameter is optional in case your local network supports a fully functional DNS. You can easily verify this when you can ping your desktop computer from your robot. In case this fails, set the ROS_IP parameter so that we can later tell the robot how to find and connect to this roscore.


Start the ALRosBridge module
--------------------------------

Please execute the following steps on your local computer, not from the robot

.. toggle_table::
  :arg1: ROS
  :arg2: NAOqi

.. toggle:: ROS

  In case you built via catkin, you can alternatively start the bridge via rosrun ::

  $ rosrun naoqi_rosbridge alrosbridge_bin --qi-url=<yourRobotIP> <roscore_ip> <network_interface>

  The second parameter is available from version 1.2 (or compilation from source). Starting from version 1.2 (or compilation from source) you can alternatively start the bridge via a launch file ::

  $ roslaunch naoqi_rosbridge naoqi_driver.launch nao_ip:=<yourRobotIP> roscore_ip:=<roscore_ip> network_interface:=<eth0|wlan0|tethering|vpn...>

.. toggle:: NAOqi

  In case you have a qibuild workspace, you can run the roscore by directly executing the binary ::

  $ path/to/alrosbridge_bin --qi-url=<yourRobotIP> <roscore_ip> <network_interface>


The roscore IP is the IP of the computer where the roscore is running. This command is optional. If you don't specify, any ROS communication is disabled until you call setMasterURI. The second parameter ``network_interface`` specifies the network interface the robot is connected to. This is important to establish a correct network connection between robot and computer. In case you are not certain which device to use, verify with ``ifconfig`` on the robot and verify which network device has the correct IP.

Set the roscore IP manually
---------------------------

In case you started the bridge without ROS communication (saying without the roscore_ip parameter), you can do so while the bridge is running.
For this, open a SSH connection onto your robot and execute the following command **on** the robot ::

  $ qicli call ALRosBridge.setMasterURI http://<yourDesktopIP>:11311

If the robot is not connected through ethernet, but an alternative interface such as wlan0, tethering or VPN, you need to specify the network interface ::

  $ qicli call ALRosBridge.setMasterURINet http://<yourDesktopIP>:11311 <network_interface>

For instance, if the robot use tethering (and ``tether`` shows when doing ``ifconfig``), use ::

  $ qicli call ALRosBridge.setMasterURINet http://<yourDesktopIP>:11311 tether

**Functionalities**:

On your desktop, you can use ROS as you would normally do. For instance ::

  $ rosnode info /alrosbridge
  --------------------------------------------------------------------------------
  Node [/alrosbridge]
  Publications:
   * /alrosbridge/sonar/right [sensor_msgs/Range]
   * /alrosbridge/camera/front/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
   * /alrosbridge/camera/front/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
   * /alrosbridge/camera/front/image_raw [sensor_msgs/Image]
   * /alrosbridge/camera/front/image_raw/theora [theora_image_transport/Packet]
   * /alrosbridge/audio [naoqi_msgs/AudioBuffer]
   * /alrosbridge/info [std_msgs/String]
   * /joint_states [sensor_msgs/JointState]
   * /rosout [rosgraph_msgs/Log]
   * /tf [tf2_msgs/TFMessage]
   * /alrosbridge/camera/front/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
   * /alrosbridge/camera/front/camera_info [sensor_msgs/CameraInfo]
   * /alrosbridge/camera/front/image_raw/compressed [sensor_msgs/CompressedImage]
   * /alrosbridge/imu_torso [sensor_msgs/Imu]
   * /alrosbridge/sonar/left [sensor_msgs/Range]
   * /alrosbridge/camera/front/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
   * /diagnostics_agg [diagnostic_msgs/DiagnosticArray]

  Subscriptions:
   * /move_base_simple/goal [geometry_msgs/PoseStamped]
   * /cmd_vel [unknown type]

  Services:
   * /alrosbridge/camera/front/image_raw/compressed/set_parameters
   * /alrosbridge/camera/front/image_raw/compressedDepth/set_parameters
   * /alrosbridge/set_logger_level
   * /alrosbridge/get_loggers
   * /alrosbridge/camera/front/image_raw/theora/set_parameters


  contacting node http://10.0.128.70:43458/ ...
  Pid: 21490
  Connections:
   * topic: /rosout
      * to: /rosout
      * direction: outbound
      * transport: TCPROS


This module also provides an API on the robot side to:
 * Publish data
 * Record data
 * Send command to naoqi modules

You can find more about it on the :ref:`API page <api>`, or you can go back to the :ref:`index <main menu>`.
