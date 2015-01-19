cmake_minimum_required(VERSION 2.8)
project(myservice)
find_package(qibuild)


add_library(
  alrosbridge
  src/alrosbridge.cpp
  src/publishers/publisher_string.cpp
  src/publishers/publisher_int.cpp
)

add_library(
  alrosconverter
  src/converters/alrosconverter.cpp
)


#if (I_AM_A_ROBOT)
qi_create_bin(
  alros
  src/main.cpp
)
qi_use_lib(alros QI ROS)
qi_use_lib(alrosconverter QI ROS)
qi_use_lib(alrosbridge QI ROS)

add_subdirectory( test )

#else()
#qi_create_bin(
#  alros
#  src/main_local.cpp
#)

#find_package( roscpp )
#target_link_libraries(alrosconverter ${roscpp_LIBRARIES} )
#target_link_libraries(alrosbridge ${roscpp_LIBRARIES} )
#qi_use_lib( alrosconverter QI )
#qi_use_lib( alrosbridge QI )
#endif()

target_link_libraries(
  alros
  alrosbridge
  alrosconverter
)
