^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package naoqi_rosbridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.5 (2015-08-27)
------------------
* fix correct loading of urdf
* Contributors: Karsten Knese

0.5.4 (2015-08-27)
------------------
* remove useless include
* add V Rabaud as a maintainer
* Contributors: Vincent Rabaud

0.5.3 (2015-08-26)
------------------
* fix: advertise service in global ns
* Contributors: Karsten Knese

0.5.2 (2015-08-26)
------------------
* build and run dependency v004 for bridge msgs
* fill robot config data
* implement robot config service call
* change to latest robotinfo msg
* add sessionptr to service
* fill the service to get the robot info
* Merge pull request `#38 <https://github.com/ros-naoqi/naoqi_driver/issues/38>`_ from antegallya/patch-1
  Fix repo url in install.rst
* Fix repo url in install.rst
* Merge pull request `#37 <https://github.com/ros-naoqi/naoqi_driver/issues/37>`_ from antegallya/patch-1
  Fix a code-block in install.rst
* Fix a code-block in install.rst
* rename service topic to ros standard
* add license declaration
* add support for ros services
* update doc
* enhance error message in camera converter
* naoqi_driver_node is an executable not a library
* Contributors: Karsten Knese, Pierre Hauweele, Vincent Rabaud

0.5.1 (2015-08-11)
------------------
* rename dump_enabled to log_enabled
* introduce prefix to naoqi driver c'tor
* switch to boost program options
* do not set the log level if it has not changed
* get a more generic way of setting the log level
* publish to diagnostics as it should be
* respect the ROS log level
* cleanup main
* update rviz configuration
* extend teleop for set_angles
* exclude driver helper to cpp for one-call only
* cleanup battery diagnostics
* remove max velocity
* Merge pull request `#30 <https://github.com/ros-naoqi/naoqi_driver/issues/30>`_ from laurent-george/patch-1
  fix git repo url
* fix git repo url
  it's a _ not a -
* change doc for renaming to naoqi driver
* renamed files for naoqi_driver
* update doc to correct renaming
* update doc to correct renaming
* add stiffness and fix battery status
* Contributors: George Laurent, Karsten Knese, Vincent Rabaud

* remove legacy code
* fix typo in package.xml
* rename package to naoqi_driver
* remove alrosbridge prefix and cleanup
* fix typo in cmakelist
* Fixes for c++11
* remove naoqi_msgs includes
* fix for correct header include of msgs
* remove deprecation warning
* Contributors: Guillaume JACOB, Karsten Knese, Vincent Rabaud

0.1.2 (2015-07-15)
------------------
* update start doc for v1.2
* lower default values for camera
* add bottom camera
* create launch file for running rosbridge
* remove ros args from cmdline
* nao basefootprint
* remove ros args
* main:  support 2nd argument as network interface
* ros_env.hpp write error message when network interface is not found
* include install instructions for ROS
* Contributors: Karsten Knese, Kei Okada, Vincent Rabaud

0.1.1 (2015-06-25)
------------------
* update the Pepper URDF
* add optical frame
* Contributors: Karsten Knese, Vincent Rabaud

0.1.0 (2015-06-18)
------------------
* devel-space compatibility
* move application files to app folder
* Add methods to remove bags presents on folder
* Add an helper function to check size taken by bags
* Add an helper function to check presents bags on folder
* rename urdf
* add romeo.urdf
* update and rename files to be consistent with description
* update doc for rosrun
* updated roscore option in doc
* remove test folder
* Contributors: Karsten Knese, Marine CHAMOUX, Vincent Rabaud

0.0.7 (2015-06-02)
------------------
* correct filepath lookup for catkin and qibuild
* hotfix: do not cast 'getenv' return to string when it's null
* hotfix: allow to register correctly a converter on the fly
* Contributors: Karsten Knese, Marine CHAMOUX, zygopter

0.0.6 (2015-05-28)
------------------
* add install rule for the module file
* Contributors: Vincent Rabaud

0.0.5 (2015-05-24)
------------------
* clean seperation between catkin and qibuild
* adjust sdk prefixes with cmake_prefix
* fix devel problems and rename lib
* set sdk prefix to devel space
* add a file to register a NAOqi module
* Contributors: Karsten Knese, Vincent Rabaud

0.0.4 (2015-05-17 22:48)
------------------------
* get code to compile with catkin
* Contributors: Vincent Rabaud

0.0.3 (2015-05-17 21:22)
------------------------
* get code to compile with catkin
* Contributors: Vincent Rabaud

0.0.2 (2015-05-17 14:08)
------------------------
* bump version
* get code to compile with catkin
* bugfix: cyclic buffer for log
* bugfix: apply config file
* replace tf helpers with tf2
* remove legacy code
* introduce a config json format for configuring converters
* Merge pull request `#7 <https://github.com/ros-naoqi/alrosbridge/issues/7>`_ from zygopter/master
  Bufferize & minidump event converters (as audio)
* Hotfix: Put throwing function inside the try/catch
* Hotfix: use set_capacity instead of resize for circular buffer
* Hotfix: allow to record event converters in 'startRecordingConverters'
* Hotfix: put checker condition to true when record is started
* Better synchronazation of data for minidump
* Use a circular buffer instead of a simple list for optimization
* Add beggining time of minidump call for event synchronization
* Hotfix: block buffer writing to have synchronized data in minidump
* Hotfix: resize correctly the buffer when changing the duration
* Remove spamming logs
* Hotfix: set buffer duration for event converters
* Add prefix name for ROSBag in minidump
* Be able to write the event converter's buffer in miniDump
* Be able to bufferize event converters
* remove while loop in startConverter&miniDump
* Merge branch 'devel' (early part)
  Conflicts:
  src/alrosbridge.cpp
* introduce time lapse measure
* configuration booleans for default pub
* try lock for write_mutex
* Merge pull request `#6 <https://github.com/ros-naoqi/alrosbridge/issues/6>`_ from zygopter/master
  Correct Buffersize calculation
* Use a static const variable instead of a #define
* Add a getter method for buffer duration
* Set a global variable for default buffer duration
* Hotfix: set correct size for buffer
* Allow to start the application automatically
* Merge pull request `#5 <https://github.com/ros-naoqi/alrosbridge/issues/5>`_ from zygopter/master
  hotfix: bad path for header in test
* hotfix: bad path for header in test
* Merge pull request `#4 <https://github.com/ros-naoqi/alrosbridge/issues/4>`_ from zygopter/master
  Refactoring of audio converter to manage publishing & recording
* Merge pull request `#3 <https://github.com/ros-naoqi/alrosbridge/issues/3>`_ from GuillaumeJacob/master
  fix cameraInfo for infrared camera
* Refactor audio converter to manage to record it
* Rename event class and move to event folder
  Delete unused files
* Split reset function into publisher & recorder for events
* fix cameraInfo for infrared camera
* Merge pull request `#2 <https://github.com/ros-naoqi/alrosbridge/issues/2>`_ from Karsten1987/master
  no roscore dependency for recording
* Merge pull request `#1 <https://github.com/ros-naoqi/alrosbridge/issues/1>`_ from zygopter/master
  Update documentation for installation
* Change 'git clone' by 'qisrc add' to download & reference projects in qibuild
* hotfix: setting timestamp
* trigger init function also with given roscore ip
* api change: start rosloop without rosmaster initialization
* exclude TransformBroadcaster into a shared_ptr
  this allows to create a joint state publisher without a need to create a
  nodehandle
* Add missing dependency in install.rst
* take rostime.now for camera to sync with other publisher
* Fix wrong project name in rst configuration file
* Add gitignore file
* remove console bridge dependency
* Initial commit
* Add link from rst doc to doxygen doc
* Update Doxyfile
* README points to the doc URL
* Use RST instead of markdown
* Doc test
* add support for Doxygen
* add instructions on how to build the docs
* fix: correct licence agreement
* adjust camera msg timestamp to alimage timestamp
* change colorspace to rgb8 for front camera
* Merge branch 'master' into 'master'
  Master
* add color for better understanding
* bugfix on run script for linux64
* Add dependency for linux64
* Add qicli call function to choose converters for minidump
* Change message output for minidump and stop record
* support for IR camera
* hotfix: stabilize publisher frequence
* give the master ip directly via commandline args
* Fix doc line
* Prepare files for doxygen documentation
* Move test includes into test/ (so they are not considered by doxygen)
* Merge branch 'master' into 'master'
  Master
* Factorize the code to retrieve anyvalues
* Hotfix: register callback to bufferize for memory converters
* Add test for minidump
* Add a setter function to choose the ROSbag duration for minidump
* Merge branch 'doc' into 'master'
  Doc
* Merge branch 'master' into 'master'
  Master
* Hotfix: catch exception when key does not exist in ALMemory && return boolean
* Doc fix
* Add links to go back to main menu
* Final touch
* Add topics page
* Add troubleshooting, next step and other usage pages
* Fix wrong definition of getMasterURI in api.rst
* Small fixes
* Add API page
* Add getting started page
* Create the index, add the howto install page
* Simplify README.rst, and point to the doc/ folder
* Avoid segfault if a value retrieval fails
* Merge branch 'compilation_fix' into 'master'
  Compilation fix
* Fix compilation issue after toolchain update
* Merge branch 'mc/event' into 'master'
  Mc/event
* Move 'getDataType' function to helpers.hpp
* Support no usage of ALValue
* support new recorder API
* remove useless debbug logs
* Switch in respect to data type of event
* Improve life functionement of event registration
* Refactor test due to library changes
* Check if the process is started
* Add a qicli function to register a memory converter
* Add mutexes in EventRegister
* Add a generic virtual class for event converter
* Add privacy to internal functions && delete test function
* Add test for new event ros bridge
* New class to deal with memory events
* Merge branch 'mc/devel' into 'master'
  Mc/devel
* Use optional custom frequency for buffer data
* Fix test
* Add qicli call function to write a ROSbag with the last 10s data buffer
* Register LOG callback to 'bufferize' recorder's function
* recorder: Add function to write buffer in a ROSbag
* recorder: Add bufferize function for camera & new buffer frequency argument in constructor
* recorder: Add a function to bufferize converter's data over the last 10 sec
* recorder: Add frequency argument in recorder reset function
* recorder: Check if vector is empty before writing a TF message on ROSbag
* recorder: Check message timestamp to write it on ROSbag
* Change message type for Info converter
* unixify the README file
* Get rid of the qimessaging warning
* Update alvisiondefinitions.h with latest available doc (this fix `#31 <https://github.com/ros-naoqi/alrosbridge/issues/31>`_)
* Remove useless comment
* Add security when getting image (in case no image is retrieved)
* Merge branch 'sa/no_alvalue' into 'master'
  Sa/no alvalue
* Remove undesirable dependency
* Do not use ALValue when guessing memory key type anymore
* Do not use ALValue when retrieving memory list anymore
* Fix indexing error
* Do not use ALValue when retrieving audio anymore
* Do not use ALValue anymore to retrieve the cameras
* Merge branch 'mc/devel' into 'master'
  Mc/devel
* recorder: be consistent between publisher topic & recorder topic
* Remove useless files (issue `#28 <https://github.com/ros-naoqi/alrosbridge/issues/28>`_)
* remove alvalue includes
* use proper string conversion
* Fix `#29 <https://github.com/ros-naoqi/alrosbridge/issues/29>`_: wrong rviz config for nao
* Merge branch 'sa/devel' into 'master'
  Sa/devel
* Audio converter (never stops)
* Merge branch 'mc/devel' into 'master'
  Mc/devel
* Update README
* Add timestamp in memory list message
* Update README.rst to add explanations on converters/recording
* Merge branch 'sa/info' into 'master'
  Sa/info
* Make the info publisher set the robot_description
* Reset the list of publishers when resetting sonar publisher node
* Useless calls
* Normalize log publisher init
* Merge branch 'sa/recorder_cleanup' into 'master'
  Sa/recorder cleanup
* Recorder clean up
* Merge branch 'mc/devel' into 'master'
  Mc/devel
* Catch error when getting typed data from ALMemory in all converters
* Catch error when getting typed data from ALMemory in Info & MemoryBoolConverter
* Merge branch 'mc/devel' into 'master'
  Mc/devel
* hotfix: delete float publisher from CMakeList
* Fix test compilation
* Merge branch 'sa/pub_cleanup' into 'master'
  Sa/pub cleanup
  I know it is scary, but this actually reduces the code a lot and it still works.
* Remane BasePublisher in BasicPublisher
* Big cleanup of publishers (next)
* Big cleanup of publishers
* Merge branch 'mc/devel' into 'master'
  Mc/devel
* hotfix: use toolchain custom ros msgs include
* Fix CMakeList.txt
* Remove include files (integrated in the toolchain)
* Info conv/pub/rec
* Merge branch 'sa/diagnostics_recorder' into 'master'
  Sa/diagnostics recorder
* Add diagnostics recorder
* Merge branch 'sa/diagnostics_converter_and_fix' into 'master'
  Sa/diagnostics converter and fix
* Remove useless include
* Remove useless call to reset
* Add diagnostics converter/publisher
* Fix naming error
* remove alvalue dependencies
  still exist in camera
* Merge branch 'sa/include' into 'master'
  Sa/include
  Remove useless includes, reorganize them all
* Remove useless include in main src, move the others to minimize their scope
* Remove useless include in converter, move the others to minimize their scope
* Remove useless include in tool, move the others to minimize their scope
* Remove useless include in publisher, move the others to minimize their scope
* Remove useless include in recorder, move the others to minimize their scope
* Cleaning: remove useless include in subscribers
* Prettify #include in subscribers
* Prettify the #include in recorders
* Prettify the #include in publishers
* Prettify the #include in converters
* Merge branch 'sa/setMasterUri' into 'master'
  Sa/set master uri
* Move getRobotDescription into tools/
* Set /robot_description when setting Master URI
* Make JS Converter non-dependent from the Node handle
* Reset tf broadcaster when JS publisher is reset
* Only register new converters if required
* Better mutex and proper stop of the ROS loop when changing master URI
* Merge branch 'mc/devel' into 'master'
  Mc/devel
* Update README.rst for function 'registerMemoryConverter' changes
* Add bool msg for memory converter
* Add namespace for DataType enum
* Update README.rst to add new API function
* Add templated function to register memory converter
* delete naoqi_bridge messages
* Add function to get data type from memory_key && add frequency argument
* Only publish/record msgs when the memory data is valid
* Use specific stamped msg for memory converters
* Return max() when there is no data in ALMemory
* Add new API function 'registerMemoryConverter'
* add test for register memory key converter
* add enum for memory data type
* add converters for int/float/string memory key
* Merge branch 'sa/conv_pub_rec_sub_factorize' into 'master'
  Factorization of conv/pub/rec/sub init
  Put everything that is required to properly initialize sub/pub/rec/conv elements in the corresponding register function
* Remove useless init function
* Move call to sub.reset
* Factorize registration code
* Factorize recorder reset
* Factorize publisher reset
* Remove new memory converters initialization (useless now)
* Factorize conv.reset() in registerConverter()
* Init the converters as soon as they are registered
* Merge branch 'mc/recorder' into 'master'
  Mc/recorder
* hotfix: check first list of topics to open a bag only if at least one topic is available
* Merge branch 'devel' into 'master'
  Devel
* Merge branch 'documentation' into 'devel'
  Documentation
* Update README
* Merge branch 'sa/hotfix' into 'master'
  Sa/hotfix
* Change module name in Documentation
* Rename alros_bin to alrosbridge_bin in run.sh
* Add API description in README
* Merge branch 'devel'
  Conflicts:
  include/alrosbridge/alrosbridge.hpp
  manifest.xml
  src/alrosbridge.cpp
* rename alsrosconverter to alrosbridge
* Merge branch 'sa/mem_list_improvement' into 'devel'
  Sa/mem list improvement
* Accept bool ALValue (convert them in Int)
* Merge branch 'mc/recorder' into 'devel'
  Mc/recorder
* Rename API function to be consistant
* Merge branch 'mc/recorder' into 'devel'
  Mc/recorder
* Rename API function 'startRecordTopics' to 'startRecordConverters'
  Conflicts:
  src/alrosbridge.cpp
* Merge branch 'sa/mem_list_doc' into 'devel'
  Sa/mem list doc
* Add doc in README about mem key list publication
* hotfix :-)
* Merge branch 'sa/list_of_mem_keys' into 'devel'
  Sa/list of mem keys
* Parse the JSON file containing the mem key list and give it to the converter
* Safely return from addMemoryConverters if node handle is not initialized
* Add a recorder for the list of memory keys
* Publish the memory list
* Instanciate a memory list converter (file parsing mocked up)
* Fix reset message at each cycle
* Fix string in message creation in converter
* Add memory list publisher
* Memory list converter
* Add new naoqi messages to manage memory values list
* Add new API method addMemoryConverters (does not do anything for now)
* Re-establish the truth
* Avoid warning message from qimessaging spam
* Merge branch 'mc/devel' into 'devel'
  Mc/devel
* Recorder: rename topics in ROSbag as publishers rostopic
* Merge branch 'mc/devel' into 'devel'
  Mc/devel
* Recorder: add sonar and laser
* Update package version
* Merge branch 'sa/new_concept' into 'devel'
  Sa/new concept
* Merge branch 'sa/concept_test' into 'devel'
  Testing the change of concept
* Change concept to store shared_ptr instead of objects themselves
* Change converters constructors to allow construction through make_shared
* Test new concept style
* merge commit
* rviz config with laser and sonar
* hotfix: no callall for empty action vector
* sonar support
* Merge branch 'mc/devel' into 'devel'
  Mc/devel
* remove unused functions from converter concept
* Recorder: use colors defined in tools
* Recorder: add coloured logs for recording functions
* Recorder: implement startRecordtopics API function
* Merge branch 'mc/devel' into 'devel'
  Mc/devel
* Return a string in stopRecord function
* hotfix: hidden improvement
* Change converter's name
* Add 2 getters for converter's name and subscribed publisher's name
* Merge branch 'sa/devel' into 'devel'
  Sa/devel
  Small fixes
* No laser for Nao
* Fix spelling mistake
* Remove old calls to publishers replaced by converters
* Merge branch 'sa/devel' into 'devel'
  IMU recorder
* Merge branch 'hotfix' into 'devel'
  Hotfix
* hotfix: check current path to add it to the bag name
* Remove useless inclusion (already included in another header)
* Add Imu recorder to the bridge
* IMU recorder
* Remove useless ";"
* Merge branch 'mc/devel' into 'devel'
  Mc/devel
* Proper way to get relative share folder path && always reload description from file
* bugfix: initialize tf_buffer before converter
* odometry
* Merge branch 'bug26/bagpath' into 'devel'
  Bug26/bagpath
* Fix `#26 <https://github.com/ros-naoqi/alrosbridge/issues/26>`_: Use an absolute path to store the bag
* Merge branch 'sa/devel' into 'devel'
  Fix `#25 <https://github.com/ros-naoqi/alrosbridge/issues/25>`_
* Fix `#25 <https://github.com/ros-naoqi/alrosbridge/issues/25>`_: log spam due to implicit conversion from ALValue to float vector
* Merge branch 'sa/dev' into 'devel'
  Sa/dev
* Add IMU_base for Pepper
* Rename IMU in IMU_torso
* Do not start depth camera if using a Nao
* Converter and publisher for IMU
* Merge branch 'mc/devel' into 'devel'
  Mc/devel
* Delete spamming logs
* Merge branch 'mc/devel' into 'devel'
  Mc/devel
* hotfix: install share folder for runtime loading
* hotfix: Check if sleep time is positive
  initially the pubs are not scheduled in the future
  so the time to sleep can be negative, which resolves in infinity
  Conflicts:
  src/alrosbridge.cpp
* recorder: first check if rosbag is open before writing
* hotfix: install share folder for runtime loading
* hotfix: Check if sleep time is positive
  initially the pubs are not scheduled in the future
  so the time to sleep can be negative, which resolves in infinity
* Update README.rst
* Merge branch 'sa/dev' into 'devel'
  Sa/dev
  Some small fixes
* Merge branch 'master' into 'master'
  Master
* rename 'start/stop' into 'startPublishing/stopPublishing'
* Update README.rst
* Update README.rst to have it without building it
* Factorize isSubscribed function
  Conflicts:
  src/publishers/info.hpp
  src/publishers/laser.hpp
  src/publishers/publisher_base.hpp
* Avoid useless copy
* Remove useless ;
* Package project into an app c++
* correct camera info frames and publisher
* first version of record and publish via callback
* sonar converter
* laser converter
* Merge branch 'mc/devel' into 'devel'
  Mc/devel
* recorder: bugfix `#24 <https://github.com/ros-naoqi/alrosbridge/issues/24>`_ recorder base class does not implement all functions
* Package project into an app c++
* Merge branch 'mc/devel' into 'devel'
  Mc/devel
* recorder: add tests for new recorder's API
* recorder: implement data recording in main class
  Conflicts:
  src/alrosbridge.cpp
* recorder: add methods in 'converter' to know if recording is enabling for a converter instance
* recorder: add concrete recorder instances for each converters
* recorder: add a recorder concept class to instanciate concrete recorders
* Merge branch 'devel' of gitlab.aldebaran.lan:kknese/alrosconverter into mc/devel
  Conflicts:
  CMakeLists.txt
  include/alrosbridge/alrosbridge.hpp
  src/alrosbridge.cpp
  src/publishers/joint_state.cpp
  src/publishers/joint_state.hpp
  test/recorder_test.cpp
* camera and joint states
* camera converter callback
* camera converter callback
* test converters
* refactoring cleanup
* recorder namespace
* recorder: add a new instanciation of 'write' method for vector<geometry_msgs::TransformStamped> messages
* recorder: clean test recording in alrosbridge.cpp
* Revert "Recorder: clean recorder files from master branch"
  This reverts commit 00f2d313b96308f2256dc001af9766d3f417578d.
  Conflicts:
  include/alrosbridge/alrosbridge.hpp
* Revert "Recorder: remove unuseful dependency"
  This reverts commit 4f0e7e677ca241c0d45aa053b4fe3e6cb150c0d2.
* Stop publishing thread before removing the publishers and subscribers
  Conflicts:
  src/alrosbridge.cpp
* Register callback on qi::application::atStop to handle variable's destruction before run() returns
* demo config
* Merge branch 'master' into 'master'
  Master
* Stop publishing thread before removing the publishers and subscribers
* Register callback on qi::application::atStop to handle variable's destruction before run() returns
* camera converter
* initial refactoring, moving files, changing baseclass
* get moveto to be asynchronous
* replace tf listeners by a shared tf buffer
* Merge branch 'master' into 'master'
  Master
* Recorder: remove unuseful dependency
* Recorder: clean recorder files from master branch
* Recorder: Add public method to record by topics
* Recorder: Renaming in recorder & test recording by topics
* get moveto to be asynchronous
* replace tf listeners by a shared tf buffer
* rm consolebridge dependency
* rm consolebridge dependency
* Merge branch 'sambrose/master' into 'master'
  Sambrose/master
  Some small fixes to avoid segfault or nasty stuff when leaving the program.
* Avoid segfault if setting the master URI, but no task is scheduled
* Do not use unlock, scope the mutex
* Avoid segfault when quiting without having set a Master URI
* add refactoring test
* first test for callback refactor
* Merge branch 'master' into 'master'
  Master
* Recorder: Add time to bag name
* Recorder: Add a basic test for recorder class
* Recorder: First draft of a ROSbag recorder API
* use latest urdf file
* cleaner NAO - Pepper separation in Publisher registration
* remove useless checks as we can now support proper latching
* properly schedule publishers in case of ROS_MASTER_URI reset.
* add boost callback test
* basefootprint publisher for nao
  add nao_joint_states.cpp
* fix time stamp
* add pepper rviz config file
* exclude odometry from joint_state_publisher
* Merge branch 'sambrose/master' into 'master'
  Automatically deploy ros from toolchain
  Hey !
  This is a very small MR to:
  1) Test the MR behavior when using branches on the same project
  2) To share my great progress: allow the ros toolchain to be deployed to the robot just by adding a word :D
  Hope you will like it ^^
* Add ros dependency to qiproject
  This will automatically deploy ros package on the robot when using
  qibuild deploy
* Merge branch 'master' into 'master'
  Master
  Fix issue `#11 <https://github.com/ros-naoqi/alrosbridge/issues/11>`_
  - Correct frame transform in moveto
  - Add correct yaw orientation to moveto command
* Correct tf2 time lookup in moveto && Add orientation to moveto command
* add NAO rviz config file
* bugfix: publish correct depth_camera encoding
* reduce default CPU usage by not using a tf2 listener if no subscriber
* Do not advertise compressed depth topics for non depth images
  This fixes `#3 <https://github.com/ros-naoqi/alrosbridge/issues/3>`_
* remove verbosity in laser
* check against AL::kDepthCamera instead of 2
* use camera with correct frequency
  removes hardcoded 20
* bugfix: correct parent path
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
* load urdf from file if no rosparam
* start depth camera only on pepper
* Merge branch 'update_doc' into 'master'
  Update doc
* Moving section compiling into Getting started.
  It is easier to read the documentation this way: In getting start it's
  straightforward no need to go to end of page to understand how to install the
  ros bridge.
* fix correct robot id
* fix runtime problem
* update the todos
* switch to tf2
* first import of the current naoqi msgs
* add a basic way of importing messages and having them be part of our headers
* update README
* clean msg folder
* update doc for Android and misc clean-ups
* add a method to set the netowork interface too
* add proper timestamps for the images / camera info
* Revert "remove a memory copy for images"
  This reverts commit 72b02187b48bafcfdee7eaa889d0b185bec57793.
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
  Conflicts:
  CMakeLists.txt
  src/alrosbridge.cpp
* 2d nav goal (rviz) moveto support
* better handling of potential log explosion
* add a log bridge
* fix abusive rate for info
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
  Conflicts:
  CMakeLists.txt
* support for teleop subscriber
* quickfix: return correct robot string
* add the first draft of an info module
* fix compilation
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
* quickfix: return correct robot name
* quickfix: remove whitespaces in string compare
* fix crashes when resetting the master URI
* properly call the subscribe/unsusbcribe methods for sonar
* increase laser frequency to 10hz
* limit laser range to 1.5 to eliminate noise
* correct odometry frame
* motion twist subscriber
* fix camera frames so that they are the optical frame
* use a proper raw topic
* only publish lasers when on Pepper
* add a sonar publisher
* add a way to know the ID of the robot and unify publisher constructors
* add a bit more specs
* clean reset logging
* remove a memory copy for images
* disabled verbosity in lasers
* unregister properly from VideoDevice when quitting or resetting
* initial support for laser scan
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
  Conflicts:
  src/publishers/camera.cpp
* publish odom frame
* expose name in print statement
* fix overlap of camera_infos
* use proper image_Transport API and show loadable plugins
* fix install of package with latest qibuild
* update docs
* first draft of diagnostics
  A proper solution would publish al ldiagnostics at different
  rates and use an aggregator as usually done.
  We will check with the CPU usage whether this is possible
* Merge branch 'camera_info'
  Conflicts:
  src/publishers/camera.cpp
  src/publishers/camera.hpp
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
* minimize the memory copies for the image
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
  Conflicts:
  include/alrosbridge/alrosbridge.hpp
  src/alrosbridge.cpp
* implement depth image with camera info
* quickfix: resolve segfault in schedule publisher
  hint: prevent a re-alloc of memory in all_publisher variable since this leads to invalid pointer
* const pointer implementation
* fix a crash with undefined pointer
* use the create_module macro as it should be
* update docs
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
  Conflicts:
  include/alrosbridge/alrosbridge.hpp
  include/alrosbridge/publisher/publisher.hpp
* add license and public interface doc
* add license and public interface doc
* rename project name to alrosbridge
* rename external service entry point
* remove legacy code
* quickfix: change CMake for filechange
* enable all default publisher
* renamend autoload entry point
* remove constructor with nodehandle parameter
* expose public interface headers in include folder
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
* small cleanups
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
* Merge branch 'master' into 'master'
  clean base classes
  This will be useful for diagnostics too: I don't want to implement yet another base class there.
* cleanup: remove unused interface
* clean base classes
* quick fix: enable publishing in alrosbridge
* Merge branch 'master' into 'master'
  allow for different publisher frequencies
* allow for different publisher frequencies
* Merge branch 'camera_publisher'
* remove constructor with nodehandle
  no reset by initialization
* bugfix: single reset/init point
* remove verbosity in publishing
* added a bgr8 front camera publisher
* add precisions about topics
* update doc
* update documentation
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
* update README
* trigger ros-init without siginthandler
* add basic doc
* basic naoqi2 module with start/stop publising
  has a minor bug of destroying the module
* main.cpp for external binary execution
* exclude naoqi autoload registration
* implement operator==()
* introduce crtp
* send dynamic float array for benchmark
* updated readme
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
* add robot state publisher in code
* publishing joint states in global namespace
* add test_primitives
* add naoqi agnostic ros code for benchmarking
* add another TODO
* update README
* Merge branch 'master' of gitlab.aldebaran.lan:kknese/alrosconverter
* code cleanup
* exclude static ros function in ros_env.hpp
* added joint_state_publisher
* increase publish rate to 15
* use linux64 toolchain pkg for local compile
* add a README file
* basic bridge example for int and strings
* basic publisher example (string, int)
* support for multiple publishers
* base structure of bridge concept
* adding simple publisher
* initial commit
* Contributors: Guillaume JACOB, Karsten KNESE, Karsten Knese, Laurent GEORGE, Marine CHAMOUX, Surya AMBROSE, Surya Ambrose, Vincent Rabaud, sambrose, zygopter
