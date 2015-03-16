cmake_minimum_required(VERSION 2.8)
project(alrosbridge)

find_package(qibuild)
find_package(qimodule)

#SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -std=c++0x")

include_directories( include )

qi_create_module(
  alros
  src/autoload_registration.cpp
  SHARED
)
qi_stage_lib( alros )

set(
  CONVERTERS
  src/converters/int.cpp
  src/converters/string.cpp
  )
qi_create_lib(
  alrosconverters
  SHARED
  ${CONVERTERS}
  )
qi_use_lib( alrosconverters QI QICORE ALCOMMON ALVISION ROS )
qi_stage_lib( alrosconverters )

set(
  PUBLISHER
  src/publishers/camera.cpp
  src/publishers/diagnostics.cpp
  src/publishers/int.cpp
  src/publishers/log.cpp
  src/publishers/string.cpp
  src/publishers/joint_state.cpp
  #  src/publishers/nao_joint_state.cpp
  #  src/publishers/odometry.cpp
  src/publishers/laser.cpp
  src/publishers/sonar.cpp
  src/publishers/info.cpp
  )

set(
  SUBSCRIBER
  src/subscribers/teleop.cpp
  src/subscribers/moveto.cpp
  )

qi_create_lib(
  alrosbridge
  SHARED
  src/alrosbridge.cpp
  src/recorder/recorder.cpp
  ${PUBLISHER}
  ${SUBSCRIBER}
)

qi_stage_lib( alrosbridge )
qi_use_lib(alros QI ROS)
qi_use_lib(alrosbridge QICORE QI ALCOMMON ALVISION ROS)
target_link_libraries(
  alrosbridge
  alrosconverters
  )

target_link_libraries(
  alros
  alrosbridge
)

qi_create_bin(alros_bin src/external_registration.cpp)
target_link_libraries(
  alros_bin
  alros
)

# install the urdf for runtime loading
qi_install_data( share/)

add_subdirectory( benchmark )
add_subdirectory( test )
