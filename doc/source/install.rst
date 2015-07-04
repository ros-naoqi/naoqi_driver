.. _install:

How to install it
=================

.. toggle_table::
  :arg1: ROS
  :arg2: NAOqi

.. toggle:: ROS

  **Binary:**

  If you are in a ROS environment, you followed the official `instructions <http://wiki.ros.org/ROS/Installation>`_.

  The easiest therefore is to install the official binary:

  .. code-block:: console

    sudo apt-get install ros-.*-naoqi-rosbridge

  **From source:**

  If you want to help develop the bridge and are unused to catkin, you will follow the official `catkin tutorials <http://wiki.ros.org/catkin/Tutorials>`_. On Ubuntu, it is:

  .. code-block:: console

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/ros-naoqi/alrosbridge.git
    # make sure you get all the dependencies installed
    rosdep install -i -y --from-paths ./alrosbridge
    # build
    source /opt/ros/indigo/setup.sh
    cd ../ && catkin_make

.. toggle:: NAOqi

  **Requirement:**

  The **ALRosBridge** is made to be compiled with qiBuild so you first need to install it:

  .. code-block:: console

    pip install qibuild

  To set up your environment and start using qiBuild, run the following commands:

  .. code-block:: console

    mkdir ~/qibuild_ws
    cd ~/qibuild_ws
    qibuild init

  If you encounter any issue, please refer to `qiBuild documentation <http://doc.aldebaran.com/qibuild/>`_.

  When qiBuild compiles, it will only look for dependencies within a toolchain, so one needs to be created.

  .. code-block:: console

    # Create the toolchain
    qitoolchain create <tc_name>
    qibuild add-config <tc_name> -t <tc_name>

    # Add a package that will find your local ROS installation
    cd ~/qibuild_ws
    git clone https://github.com/ros-naoqi/ros-toolchain.git
    ./ros-toolchain/update_toolchain.sh local_ros <tc_name>

  Download the dependencies of the alrosbridge:

  .. code-block:: console

    cd ~/qibuild_ws
    qisrc add https://github.com/aldebaran/gtest.git
    qisrc add https://github.com/aldebaran/libqiprobes.git
    qisrc add https://github.com/aldebaran/libqicore.git
    qisrc add https://github.com/aldebaran/libqi.git

  Download this project and compile it:

  .. code-block:: console

    cd ~/qibuild_ws
    qisrc add https://github.com/ros-naoqi/alrosbridge.git
    cd alrosbridge
    qibuild configure -c <tc_name>
    qibuild make -c <tc_name>

  Once compilation is over, the resulting binary will be in *~/qibuild_ws/alrosbridge/build-<tcname>/sdk/bin/alrosbridge_bin*

  If you encounter any compilation issue, (unable to find some dependencies), you might need to install them (through *apt-get install* for instance)

  Once you successfully compiled the module, you can learn how to use it on the :ref:`Getting started page <start>` or you can go back to the :ref:`index <main menu>`.
