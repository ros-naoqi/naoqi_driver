cmake_minimum_required(VERSION 2.8.3)
project(naoqi_driver)

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
  SERVICES_SRC
  src/services/robot_config.cpp
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
  DRIVER_SRC
  src/naoqi_driver.cpp
  src/helpers/driver_helpers.cpp
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

include_directories( include )

qi_create_module(
  naoqi_driver_module
  src/autoload_registration.cpp
  SHARED
)
qi_stage_lib( naoqi_driver_module )

qi_create_lib(
  naoqi_driver
  SHARED
  ${DRIVER_SRC}
  ${CONVERTERS_SRC}
  ${PUBLISHER_SRC}
  ${SUBSCRIBER_SRC}
  ${SERVICES_SRC}
  ${RECORDER_SRC}
  ${TOOLS_SRC}
)
qi_use_lib( naoqi_driver QICORE QI ROS )
qi_stage_lib( naoqi_driver )

target_link_libraries(
  naoqi_driver_module
  naoqi_driver
)

qi_create_bin( naoqi_driver_node src/external_registration.cpp)
target_link_libraries(
  naoqi_driver_node
  naoqi_driver
)

# install the urdf for runtime loading
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/share/" DESTINATION "${QI_SDK_DIR}/${QI_SDK_SHARE}/")
qi_install_data( share/)
