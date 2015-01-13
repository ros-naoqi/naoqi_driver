cmake_minimum_required(VERSION 2.8)
project(myservice)
find_package(qibuild)

if (I_AM_A_ROBOT)
#include_directories("include")
qi_create_bin(alrosconverter
              src/main.cpp
              src/alrosconverter.cpp
            )
qi_use_lib(alrosconverter QI ROS)
endif()
