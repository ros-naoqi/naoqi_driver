.. _api:

API description
===============

All the following calls must be performed from the robot. 

**Environment setup API**

In order to get the module to connect to your roscore, you should send it your IP.

* ``void`` ROS-Driver:\:**setMasterURI** ( ``const std::string&`` **uri** )

  Set current master URI. The IP adress given is from defauth *eth0* network interface.

  *param:* **uri** - string in form of ``http://<ip>:11311``

* ``void`` ROS-Driver:\:**setMasterURINet** ( ``const std::string&`` **uri**, ``const std::string&`` **network_interface** )

  Set current master URI using a given network interface.

  *param:* **uri** - string in form of ``http://<ip>:11311``

  *param:* **network_interface** - string. For example ``tether``.

* ``const std::string&`` ROS-Driver:\:**getMasterURI** ()

  Get current master URI using a given network interface.

  *return:* string in form of ``http://<ip>:11311``

-----------------

**Converters API**

The converters are responsible for operating conversion between NAOqi messages and ROS messages, in accordance with given frequency.

* ``const std::vector< std::string >&`` ROS-Driver:\:**getAvailableConverters** ()

  Get all registered converters in the module.

  *return:* vector of string of all converter's topic name

* ``void`` ROS-Driver:\:**registerMemoryConverter** ( ``const std::string&`` **key**, ``float`` **frequency**, ``int`` **type** )

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

* ``void`` ROS-Driver:\:**addMemoryConverters** ( ``std::string`` **filePath** )

  Add some new converters for memory keys. This call requires as argument the path to a JSON file (stored on the robot) structured as the following one.
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

* ``void`` ROS-Driver:\:**startPublishing** ()

  Start/enable publishing all registered publisher

* ``void`` ROS-Driver:\:**stopPublishing** ()

  Stop/disable publishing all registered publisher

* ``const std::vector< std::string >&`` ROS-Driver:\:**getSubscribedPublishers** ()

  Get all subscribed publishers.

  *return:* vector of string of publisher's topic name

-----------------

**Recorders API**

* ``void`` ROS-Driver:\:**startRecording** ()

  Start/enable recording all registered recorder.

  This will record all topics in one ROSbag, named after current date & time. The ROSbag is stored in the exact path where the **ROS-Driver** module is launched (meaning that it will be stored on the robot if it's launched from here).

* ``void`` ROS-Driver:\:**stopRecording** ()

  Stop/disable recording all registered recorder.


You can now have a look to the :ref:`list of available topics <topic>`, or you can go back to the :ref:`index <main menu>`.

If you want to know more about the API, you can look at the doxygen documentation `here <./api/index.html>`_
