#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>

#include <qi/os.hpp>

#include <alrosbridge/recorder/recorder.hpp>

int main( int argc, char** argv )
{
  ros::Time::init();

  alros::recorder::Recorder recorder;
  recorder.startRecord();

  int count = 10;
  while (recorder.isRecording()) {

    // Int32
    std_msgs::Int32 i;
    i.data = 42;
    std::string name = "int32";
    recorder.write(name,i);

    // String
    std_msgs::String s;
    s.data = "Marine est la plus belle";
    recorder.write("string",s);

    // PoseStamped
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = 0;
    ps.pose.position.y = 1;
    ps.pose.position.z = 2;
    ps.pose.orientation.w = 5;
    ps.pose.orientation.x = 5;
    ps.pose.orientation.y = 5;
    ps.pose.orientation.z = 5;
    recorder.write("pose stamped",ps);

    qi::os::sleep(0.5);

    count--;
    if (count<0) {
      recorder.stopRecord();
    }
  }

  return 0;

}
