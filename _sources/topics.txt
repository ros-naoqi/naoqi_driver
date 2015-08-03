.. _topic:

Topics
======

The following listed topics are available, depending for which robot you launched the bridge. Note that the topics differ between NAO and Pepper, since they don't have the same sensor and actuators (e.g. 3D camera). Further, this list may vary over time, since the bridge is still actively under development.

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


Main topics
-----------

* Camera Front

/<robot-prefix>/camera/front/camera_info (sensor_msgs/CameraInfo): publishes information on the front camera
/<robot-prefix>/camera/front/image_raw (sensor_msgs/Image): publish the images of the Top Camera obtained from ALVideoDevice

* Camera Depth (Pepper only)

/<robot-prefix>/camera/depth/camera_info (sensor_msgs/CameraInfo): publishes information on the depth camera
/<robot-prefix>/camera/depth/image_raw (sensor_msgs/Image): publish the depth images obtained from ALVideoDevice

* IMU

/<robot-prefix>/imu_base (sensor_msgs/Imu): publishes the IMU of Pepper base(Pepper only)
/<robot-prefix>/imu_torso (sensor_msgs/Imu): publishes the IMU of the robot's torso

* Joint States

/joint_states (sensor_msgs/JointState): uses the keys named Device/SubDeviceList/\*/Position/Sensor/Value at a frequency of 15Hz.

* Laser

/<robot-prefix>/laser (sensor_msgs/LaserScan): publishes the obstacles' positions retrieved through lasers.

* Sonar

/<robot-prefix>/sonar/left (sensor_msgs/Range): publishes the left sonar values of Nao (Nao only)
/<robot-prefix>/sonar/right (sensor_msgs/Range): publishes the right sonar values of Nao (Nao only)
/<robot-prefix>/sonar/front (sensor_msgs/Range): publishes the front sonar values of Pepper (Pepper only)
/<robot-prefix>/sonar/back (sensor_msgs/Range): publishes the back sonar values of Pepepr (Pepper only)

* TF

/tf (tf2_msgs/TFMessage): the usual tf message, using /joint_states

Go back to the :ref:`index <main menu>`.
