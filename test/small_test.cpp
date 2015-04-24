/*
 * ROS
 */
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/image_encodings.h>
#include <naoqi_bridge_msgs/BoolStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <qi/os.hpp>
#include <qi/applicationsession.hpp>

#include <alrosbridge/tools.hpp>
#include <alrosbridge/recorder/recorder.hpp>
#include <alrosbridge/recorder/globalrecorder.hpp>

#include "../src/publishers/basic.hpp"
#include "../src/recorder/basic.hpp"
#include "../src/converters/memory/float.hpp"
#include "../src/converters/memory/bool.hpp"

#include "../src/event.hpp"

int main( int argc, char** argv )
{
  qi::ApplicationSession app(argc, argv);
  app.start();

  qi::SessionPtr sessionPtr = app.session();

  /*qi::AnyObject p_memory = sessionPtr->service("ALMemory");
  qi::AnyValue value = p_memory.call<qi::AnyValue>("getData", "ALMemory/KeyRemoved");

  std::cout << "Value : " << value.asString() << std::endl;*/

  float buffer_duration = 10.f;
  size_t buffer_size = static_cast<size_t>(buffer_duration * 2.f);

  std::cout << "DATA:\t - duration = " << buffer_duration << std::endl
            << "\t - size = " << buffer_size << std::endl;

  buffer_size = ( buffer_size / buffer_duration ) * 20.f;
  buffer_duration = 20.f;

  std::cout << "DATA:\t - duration = " << buffer_duration << std::endl
            << "\t - size = " << buffer_size << std::endl;

  app.session()->close();
  return 0;

}
