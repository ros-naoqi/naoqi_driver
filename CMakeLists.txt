cmake_minimum_required(VERSION 2.8)
project(myservice)

find_package(qibuild)
find_package(qimodule)

SET (CMAKE_CXX_FLAGS_INIT "-Wall -std=c++0x -stdlib=libc++")

include_directories( include )

qi_create_lib(
  alros
  SRC
  src/registration.cpp
  SHARED
)
qi_stage_lib( alros )

qi_create_lib(
  alrosbridge
  SHARED
  src/alrosbridge.cpp
  src/publishers/camera.cpp
  src/publishers/int.cpp
  src/publishers/joint_state.cpp
  src/publishers/string.cpp
)
qi_stage_lib( alrosbridge )

qi_create_lib(
  alrosconverter
  src/converters/alrosconverter.cpp
)
qi_stage_lib( alrosconverter )


qi_use_lib(alros QI ROS)
qi_use_lib(alrosbridge QI ALCOMMON ALVISION ROS)
qi_use_lib(alrosconverter QI ROS)

target_link_libraries(
  alros
  alrosbridge
  alrosconverter
)

qi_create_bin(alros_bin src/main.cpp)
target_link_libraries(
  alros_bin
  alros
)


add_subdirectory( benchmark )
add_subdirectory( test )
