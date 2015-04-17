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
  src/converters/diagnostics.cpp
  src/converters/imu.cpp
  src/converters/info.cpp
  src/converters/int.cpp
  src/converters/joint_state.cpp
  src/converters/laser.cpp
  src/converters/memory_list.cpp
  src/converters/memory/bool.cpp
  src/converters/memory/int.cpp
  src/converters/memory/float.cpp
  src/converters/memory/string.cpp
  src/converters/sonar.cpp
  src/converters/string.cpp
  src/converters/log.cpp
  )
set(
  TOOLS
  src/tools/robot_description.cpp
  )
qi_create_lib(
  alrosconverters
  SHARED
  ${CONVERTERS}
  ${TOOLS}
  )
qi_use_lib( alrosconverters QI QICORE ALCOMMON ALVISION ROS )
qi_stage_lib( alrosconverters )

set(
  PUBLISHER
  src/publishers/camera.cpp
  src/publishers/info.cpp
  src/publishers/joint_state.cpp
  src/publishers/log.cpp
  src/publishers/sonar.cpp
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
  src/recorder/joint_state.cpp
  src/recorder/log.cpp
  src/recorder/sonar.cpp
  )

qi_create_lib(
  alrosbridge
  SHARED
  src/alrosbridge.cpp
  src/recorder/globalrecorder.cpp
  ${PUBLISHER}
  ${SUBSCRIBER}
  ${RECORDER}
  ${TOOLS}
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
