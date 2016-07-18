#!/bin/bash

touch .catkin
source ./setup.bash
exec bin/alrosbridge_bin &>> flightrecorder.log
