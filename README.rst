Description
===========

This is a NAOqi module that bridges with ROS. It publishes
several sensor data as well as the robot position.

On the other hand it enables ROS to call parts of the
NAOqi API.

What it does
============

The **ALRosBridge** module is in charge of providing some bridge capabilities between ROS and NAOqiOS.

How it works
============

The **ALRosBridge** module is a NAOqi module that also acts as a ROS node. As there is no **roscore** on the robot, it needs to be given the IP of the **roscore** in order to be registered as a node in the ROS processing graph. Usually, you will start your **roscore** on your local desktop.

Once connected, normal ROS communication is happening between your robot, running NAOqi OS, and your desktop, running ROS.

Performances and limitations
============================

This module is in C++ and gets data straight from the lowest levels of NAOqi hence ensuring low latency and CPU usage. For the usual usage of using /tf and TODO, we observe a latency of TODO and a CPU usage increase of TODO.

When there are no subscribers connected to a specific topic, no CPU is used on the robot.

This module tries to be as close to the ROS standard by exposing several standard ROS topics (like /tf) as well as custom ones. Those topics are detailed on the Topics page.

It also exposes the following higher level NAOqi API services:

* /cmd_vel as detailed here http://wiki.ros.org/android_teleop

Getting Started
===============

Start the **ALRosBridge** module
--------------------------------

**Requirement:**

* qiBuild package (see `here <https://github.com/aldebaran/qibuild>`_ )
* libqi library (see `here <https://github.com/aldebaran/libqi>`_ )

**Get the code from gitlab**::
  
  $ git clone git@gitlab.aldebaran.lan:kknese/alrosbridge.git
  $ qisrc add ./alrosbridge

**Run from your computer:**

Build with your linux64 toolchain::

  $ cd alrosbridge
  $ qibuild configure -c atom alrosbridge
  $ qibuild make -c atom alrosbridge

Register the BridgeService on your robot::

  $ source /opt/ros/<you_installed_rosdistro>/setup.sh
  $ ./build-linux64/sdk/bin/alrosbridge-bin --qi-url=tcp://<yourRobotIP>:9559

Using the **ALRosBridge** module
----------------------------------

To start using this bridge, you need to start a roscore locally on our desktop computer and then, to communicate your external roscore IP on your robot.

**Start a ROScore**::

  $ ROS_IP=<yourDesktopIP> roscore

We start the roscore with a prefixed IP of our desktop computer. This is needed so that we can later tell the robot how to find and connect to this roscore.

**Set the master IP**::

  $ qicli call ALRosBridge.setMasterURI http://<yourDesktopIP>:11311

We have to tell the robot the computer IP, where to find the roscore we started before.

**Functionalities**:

This module povides an API to:

* Publish data
* Record data
* Send command to naoqi modules

To learn more, see API section above.

API
===

-----------------

**Environment setup**

In order to get the module to connect to your roscore, you should send it your IP.

* ``void`` ALRosBridge:\:**setMasterURI** ( ``const std::string&`` **uri** )

  Set current master URI. The IP adress given is from defauth *eth0* network interface.

  *param:* **uri** - string in form of ``http://<ip>:11311``

* ``void`` ALRosBridge:\:**setMasterURINet** ( ``const std::string&`` **uri**, ``const std::string&`` **network_interface** )

  Set current master URI using a given network interface.

  *param:* **uri** - string in form of ``http://<ip>:11311``

  *param:* **network_interface** - string. For example ``tether``.

* ``const std::string&`` ALRosBridge:\:**getMasterURI** ()

  Set current master URI using a given network interface.

  *param:* **uri** - string in form of ``http://<ip>:11311``

  *param:* **network_interface** - string. For example ``tether``.

-----------------

**Converters API**

The converters are responsible for operating conversion between NAOqi messages and ROS messages, in accordance with given frequency.

* ``const std::vector< std::string >&`` ALRosBridge:\:**getAvailableConverters** ()
  
  Get all registered converters in the module.

  *return:* vector of string of all converter's topic name

* ``void`` ALRosBridge:\:**registerMemoryConverter** ( ``const std::string&`` **key**, ``float`` **frequency**, ``int`` **type** )

  Register a new converter for the memory key given.

  *param:* **key** - naoqi memory key. For example ``ALMemory/KeyAdded``.

  *param:* **frequency** - frequency of the converter (in Hz)

  *param:* **type** - type identifier of the given memory data.

  ::

    Available types are:
    * 0 - None/Undefined
    * 1 - Int
    * 2 - Float
    * 3 - String
    * 4 - Bool

* ``void`` ALRosBridge:\:**addMemoryConverters** ( ``std::string`` **filePath** )

  Add some new converters for memory keys. This call requires as argument the path to a JSON file structured as the following one.
  memKeys and topic must be present and filled. Frequency is optional, and if not there, the default value is 10 Hz.

  *param:* **filePath** - path of the JSON file

  ::

    {
        "memKeys": [
                    "KeyName1",
                    "KeyName2"
                   ],
        "topic": "topicName",
        "frequency": 10
    }

-----------------

**Publishers API**

* ``void`` ALRosBridge:\:**startPublishing** ()

  Start/enable publishing all registered publisher
  
* ``void`` ALRosBridge:\:**stopPublishing** ()

  Stop/disable publishing all registered publisher

* ``const std::vector< std::string >&`` ALRosBridge:\:**getSubscribedPublishers** ()

  Get all subscribed publishers.

  *return:* vector of string of publisher's topic name

-----------------

**Recorders API**

* ``void`` ALRosBridge:\:**startRecording** ()

  Start/enable recording all registered recorder.
  
  This will record all topics in one ROSbag, named after current date & time. The ROSbag is stored in the exact path where the **ALRosBridge** module is launched (meaning that it will be stored on the robot if it's launched from here).
  
* ``void`` ALRosBridge:\:**stopRecording** ()

  Stop/disable recording all registered recorder.
  
