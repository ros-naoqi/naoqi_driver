cmake_minimum_required(VERSION 2.8)
project(myservice)
find_package(qibuild)

add_library(
  alrosbridge
  src/alrosbridge.cpp
  src/publishers/publisher_string.cpp
  src/publishers/publisher_int.cpp
  src/publishers/publisher_joint_state.cpp
)

add_library(
  alrosconverter
  src/converters/alrosconverter.cpp
)

qi_create_bin(
  alros
  src/main.cpp
)

qi_use_lib(alros QI ROS)
qi_use_lib(alrosconverter QI ROS)
qi_use_lib(alrosbridge QI ALCOMMON ROS)

target_link_libraries(
  alros
  alrosbridge
  alrosconverter
)

add_subdirectory( test )
