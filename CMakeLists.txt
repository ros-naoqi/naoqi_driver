cmake_minimum_required(VERSION 2.8)
project(alrosbridge)

find_package(qibuild)
find_package(qimodule)

SET (CMAKE_CXX_FLAGS_INIT "-Wall -std=c++0x -stdlib=libc++")

include_directories( include )

qi_create_lib(
  alros
  src/autoload_registration.cpp
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


qi_use_lib(alros QI ROS)
qi_use_lib(alrosbridge QI ALCOMMON ALVISION ROS)

target_link_libraries(
  alros
  alrosbridge
)

qi_create_bin(alros_bin src/external_registration.cpp)
target_link_libraries(
  alros_bin
  alros
)


add_subdirectory( benchmark )
add_subdirectory( test )
