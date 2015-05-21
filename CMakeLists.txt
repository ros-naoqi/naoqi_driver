cmake_minimum_required(VERSION 2.8.3)
project(naoqi_rosbridge)

# define source files that can be used by qibuild or catkin
set(
  CONVERTERS_SRC
  src/converters/audio.cpp
  src/converters/camera.cpp
  src/converters/diagnostics.cpp
  src/converters/imu.cpp
  src/converters/info.cpp
  src/converters/joint_state.cpp
  src/converters/laser.cpp
  src/converters/memory_list.cpp
  src/converters/memory/bool.cpp
  src/converters/memory/int.cpp
  src/converters/memory/float.cpp
  src/converters/memory/string.cpp
  src/converters/sonar.cpp
  src/converters/log.cpp
  )
set(
  TOOLS_SRC
  src/tools/robot_description.cpp
  src/tools/from_any_value.cpp
  )

set(
  PUBLISHER_SRC
  src/publishers/camera.cpp
  src/publishers/info.cpp
  src/publishers/joint_state.cpp
  src/publishers/log.cpp
  src/publishers/sonar.cpp
  )

set(
  SUBSCRIBER_SRC
  src/subscribers/teleop.cpp
  src/subscribers/moveto.cpp
  )

set(
  RECORDER_SRC
  src/recorder/camera.cpp
  src/recorder/diagnostics.cpp
  src/recorder/joint_state.cpp
  src/recorder/log.cpp
  src/recorder/sonar.cpp
  )

set(
  BRIDGE_SRC
  src/alrosbridge.cpp
  src/recorder/globalrecorder.cpp
  src/event/basic.hxx
  src/event/basic.hpp
  src/event/audio.cpp
  )

# use catkin if qibuild is not found
if(DEFINED qibuild_DIR)
  find_package(qibuild QUIET)
else()
  find_package(catkin QUIET)
  include(CMakeLists_catkin.txt)
  return()
endif()

find_package(qimodule)

#SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -std=c++0x")

include_directories( include )

qi_create_module(
  alros
  src/autoload_registration.cpp
  SHARED
)
qi_stage_lib( alros )

# create the different libraries
qi_create_lib(
  alrosconverters
  SHARED
  ${CONVERTERS_SRC}
  ${TOOLS_SRC}
  )
qi_use_lib( alrosconverters QI QICORE ROS )
qi_stage_lib( alrosconverters )

qi_create_lib(
  alrosbridge
  SHARED
  ${BRIDGE_SRC}
  ${PUBLISHER_SRC}
  ${SUBSCRIBER_SRC}
  ${RECORDER_SRC}
  ${TOOLS_SRC}
)
qi_use_lib(alrosbridge QICORE QI ROS)
target_link_libraries(
  alrosbridge
  alrosconverters
  )
qi_stage_lib( alrosbridge )

qi_use_lib(alros QI ROS)
target_link_libraries(
  alros
  alrosbridge
)

# create the binary of the bridge
qi_create_bin(alrosbridge_bin src/external_registration.cpp)
target_link_libraries(
  alrosbridge_bin
  alros
)

# install the urdf for runtime loading
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/share/" DESTINATION "${QI_SDK_DIR}/${QI_SDK_SHARE}/")
qi_install_data( share/)

add_subdirectory( benchmark )
#add_subdirectory( test )
