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
  src/converters/camera.cpp
  src/converters/imu.cpp
  src/converters/int.cpp
  src/converters/joint_state.cpp
  src/converters/laser.cpp
  src/converters/memory_list.cpp
  src/converters/memory/int.cpp
  src/converters/memory/float.cpp
  src/converters/memory/string.cpp
  src/converters/sonar.cpp
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
  src/publishers/float.cpp
  src/publishers/imu.cpp
  src/publishers/info.cpp
  src/publishers/int.cpp
  src/publishers/joint_state.cpp
  src/publishers/laser.cpp
  src/publishers/log.cpp
  src/publishers/memory_list.cpp
  src/publishers/memory/int.cpp
  src/publishers/memory/float.cpp
  src/publishers/memory/string.cpp
  src/publishers/sonar.cpp
  src/publishers/string.cpp
  #  src/publishers/nao_joint_state.cpp
  #  src/publishers/odometry.cpp
  )

set(
  SUBSCRIBER
  src/subscribers/teleop.cpp
  src/subscribers/moveto.cpp
  )

set(
  RECORDER
  src/recorder/camera.cpp
  src/recorder/diagnostics.cpp
  src/recorder/float.cpp
  src/recorder/imu.cpp
  src/recorder/info.cpp
  src/recorder/int.cpp
  src/recorder/joint_state.cpp
  src/recorder/laser.cpp
  src/recorder/log.cpp
  src/recorder/memory_list.cpp
  src/recorder/memory/int.cpp
  src/recorder/memory/float.cpp
  src/recorder/memory/string.cpp
  src/recorder/sonar.cpp
  src/recorder/string.cpp
  )

qi_create_lib(
  alrosbridge
  SHARED
  src/alrosbridge.cpp
  src/recorder/globalrecorder.cpp
  ${PUBLISHER}
  ${SUBSCRIBER}
  ${RECORDER}
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

qi_create_bin(alrosbridge_bin src/external_registration.cpp)
target_link_libraries(
  alrosbridge_bin
  alros
)

# install the urdf for runtime loading
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/share/" DESTINATION "${QI_SDK_DIR}/${QI_SDK_SHARE}/")
qi_install_data( share/)

add_subdirectory( benchmark )
add_subdirectory( test )
