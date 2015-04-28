.. _topic:

Topics
======

Exhaustive list
---------------

Here you can find the exhaustive list of the topics published by default.

* Nao

.. code-block:: sh

  $ rosnode info /alrosbridge                                                                            [15-04-24 12:37]
  --------------------------------------------------------------------------------
  Node [/alrosbridge]
  Publications: 
   * /alrosbridge/sonar/right [sensor_msgs/Range]
   * /alrosbridge/camera/front/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
   * /alrosbridge/camera/front/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
   * /alrosbridge/camera/front/image_raw [sensor_msgs/Image]
   * /alrosbridge/camera/front/image_raw/theora [theora_image_transport/Packet]
   * /alrosbridge/audio [naoqi_msgs/AudioBuffer]
   * /alrosbridge/info [naoqi_bridge_msgs/StringStamped]
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
   * /move_base_simple/goal [unknown type]
   * /cmd_vel [unknown type]
  
  Services: 
   * /alrosbridge/camera/front/image_raw/compressed/set_parameters
   * /alrosbridge/get_loggers
   * /alrosbridge/camera/front/image_raw/compressedDepth/set_parameters
   * /alrosbridge/set_logger_level
   * /alrosbridge/camera/front/image_raw/theora/set_parameters


* Pepper

.. code-block:: sh

  $ rosnode info /alrosbridge                                                                            [15-04-24 12:37]
  --------------------------------------------------------------------------------
  Node [/alrosbridge]
  Publications: 
   * /alrosbridge/camera/front/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
   * /alrosbridge/camera/front/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
   * /alrosbridge/imu_base [sensor_msgs/Imu]
   * /alrosbridge/camera/depth/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
   * /alrosbridge/camera/front/image_raw [sensor_msgs/Image]
   * /alrosbridge/camera/depth/image_raw [sensor_msgs/Image]
   * /alrosbridge/sonar/front [sensor_msgs/Range]
   * /alrosbridge/camera/front/image_raw/theora [theora_image_transport/Packet]
   * /alrosbridge/audio [naoqi_msgs/AudioBuffer]
   * /alrosbridge/info [naoqi_bridge_msgs/StringStamped]
   * /alrosbridge/sonar/back [sensor_msgs/Range]
   * /alrosbridge/camera/depth/camera_info [sensor_msgs/CameraInfo]
   * /alrosbridge/camera/depth/image_raw/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
   * /alrosbridge/camera/depth/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
   * /alrosbridge/camera/depth/image_raw/compressed [sensor_msgs/CompressedImage]
   * /alrosbridge/camera/depth/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
   * /alrosbridge/camera/depth/image_raw/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
   * /joint_states [sensor_msgs/JointState]
   * /rosout [rosgraph_msgs/Log]
   * /alrosbridge/camera/depth/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
   * /tf [tf2_msgs/TFMessage]
   * /alrosbridge/camera/front/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
   * /alrosbridge/camera/front/camera_info [sensor_msgs/CameraInfo]
   * /alrosbridge/camera/front/image_raw/compressed [sensor_msgs/CompressedImage]
   * /alrosbridge/laser [sensor_msgs/LaserScan]
   * /alrosbridge/imu_torso [sensor_msgs/Imu]
   * /alrosbridge/camera/front/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
   * /alrosbridge/camera/depth/image_raw/compressedDepth [sensor_msgs/CompressedImage]
   * /alrosbridge/camera/depth/image_raw/theora [theora_image_transport/Packet]
   * /diagnostics_agg [diagnostic_msgs/DiagnosticArray]
  
  Subscriptions: 
   * /move_base_simple/goal [unknown type]
   * /cmd_vel [unknown type]
  
  Services: 
   * /alrosbridge/camera/depth/image_raw/theora/set_parameters
   * /alrosbridge/camera/front/image_raw/compressed/set_parameters
   * /alrosbridge/get_loggers
   * /alrosbridge/camera/depth/image_raw/compressed/set_parameters
   * /alrosbridge/camera/front/image_raw/compressedDepth/set_parameters
   * /alrosbridge/set_logger_level
   * /alrosbridge/camera/depth/image_raw/compressedDepth/set_parameters
   * /alrosbridge/camera/front/image_raw/theora/set_parameters


Main topics
-----------

* Camera Front

/alrosconverter/camera/front/camera_info (sensor_msgs/CameraInfo): publishes information on the front camera
/alrosconverter/camera/front/image_raw (sensor_msgs/Image): publish the images of the Top Camera obtained from ALVideoDevice

* Camera Depth (Pepper only)

/alrosconverter/camera/depth/camera_info (sensor_msgs/CameraInfo): publishes information on the depth camera
/alrosconverter/camera/depth/image_raw (sensor_msgs/Image): publish the depth images obtained from ALVideoDevice

* IMU

/alrosconverter/imu_base (sensor_msgs/Imu): publishes the IMU of Pepper base(Pepper only)
/alrosconverter/imu_torso (sensor_msgs/Imu): publishes the IMU of the robot's torso

* Joint States

/joint_states (sensor_msgs/JointState): uses the keys named Device/SubDeviceList/\*/Position/Sensor/Value at a frequency of 15Hz.

* Laser

/alrosconverter/laser (sensor_msgs/LaserScan): publishes the obstacles' positions retrieved through lasers.

* Sonar

/alrosconverter/sonar/left (sensor_msgs/Range): publishes the left sonar values of Nao (Nao only)
/alrosconverter/sonar/right (sensor_msgs/Range): publishes the right sonar values of Nao (Nao only)
/alrosconverter/sonar/front (sensor_msgs/Range): publishes the front sonar values of Pepper (Pepper only)
/alrosconverter/sonar/back (sensor_msgs/Range): publishes the back sonar values of Pepepr (Pepper only)

* TF

/tf (tf2_msgs/TFMessage): the usual tf message, using /joint_states

Go back to the :ref:`index <main menu>`.