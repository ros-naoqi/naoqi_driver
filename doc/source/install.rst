How to install it
=================

**Requirement:**

The **ALRosBridge** is made to be compiled with qiBuild so you first need to install it: ::

  $ pip install qibuild

To set up your environment and start using qiBuild, run the following commands: ::

  $ mkdir ~/qibuild_ws
  $ cd ~/qibuild_ws
  $ qibuild init

If you encounter any issue, please refer to `qiBuild documentation <http://doc.aldebaran.com/qibuild/>`_.

Create a toolchain: ::

  # Create the toolchain
  $ qitoolchain create <tc_name>
  $ qibuild add-config <tc_name> -t <tc_name>

  # Update it with a package that redirects qibuild towards your local ROS installation
  $ cd ~/qibuild_ws
  $ git clone https://github.com/ros-naoqi/ros-toolchain.git
  $ ./ros-toolchain/update_toolchain.sh local_ros <tc_name>

Download the dependencies of the alrosbridge ::

  $ cd ~/qibuild_ws
  $ git clone https://github.com/aldebaran/gtest.git
  $ git clone https://github.com/aldebaran/libqiprobes.git
  $ git clone https://github.com/aldebaran/libqi.git

Download this project and compile it ::

  $ cd ~/qibuild_ws
  $ git clone https://github.com/ros-naoqi/alrosbridge.git
  $ cd alrosbridge
  $ qibuild configure -c <tc_name>
  $ qibuild make -c <tc_name>

If you encounter any compilation issue, (unable to find some dependencies), you might need to install them (through *apt-get install* for instance)