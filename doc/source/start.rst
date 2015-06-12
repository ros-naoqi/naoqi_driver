.. _start:

Getting Started
===============


Start the **ALRosBridge** module
--------------------------------

**Run from your computer:** ::

  $ path/to/alrosbridge-bin --qi-url=<yourRobotIP> <roscore_ip>

The roscore IP is the IP of the computer where roscore is running. This command is optional. If you don't specify, any ROS communication is disabled until you call setMasterURI.

Use the **ALRosBridge** module
----------------------------------

To start using this bridge, you need to start a roscore locally on our desktop computer and then, to communicate your external roscore IP on your robot.

**Start a ROScore** (on your desktop machine) ::

  $ ROS_IP=<yourDesktopIP> roscore

The ROS_IP parameter is needed so that we can later tell the robot how to find and connect to this roscore.

**Set the master IP** (on the robot) ::

  $ qicli call ALRosBridge.setMasterURI http://<yourDesktopIP>:11311

If the robot is not connected through ethernet, you need to specify the network interface ::

  $ qicli call BridgeService.setMasterURINet http://<yourDesktopIP>:11311 <network_interface>

For instance, if the robot use tethering (and ``tether`` shows when doing ``ifconfig``), use ::

  $ qicli call BridgeService.setMasterURINet http://<yourDesktopIP>:11311 tether

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
