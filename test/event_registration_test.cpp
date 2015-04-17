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

#include "../src/converters/memory/float.hpp"
#include "../src/recorder/memory/float.hpp"

#include "../src/converters/memory/bool.hpp"
#include "../src/recorder/memory/bool.hpp"

#include "../src/event.hpp"

int main( int argc, char** argv )
{
  qi::ApplicationSession app(argc, argv);
  app.start();

  setenv( "ROS_MASTER_URI", "http://10.0.128.9:11311", 1 );
  ros::init( argc, argv, "converter_memory_test" );
  ros::NodeHandle n;

  if(!ros::master::check())
  {
      std::cerr<<"Could not contact master!\nQuitting... "<< std::endl;
      return -1;
  }

  boost::shared_ptr<alros::recorder::GlobalRecorder> gr = boost::make_shared<alros::recorder::GlobalRecorder>("");

  qi::SessionPtr sessionPtr = app.session();
  alros::EventRegister<alros::converter::MemoryBoolConverter,alros::publisher::BasicPublisher<naoqi_bridge_msgs::BoolStamped>,alros::recorder::MemoryBoolRecorder> event_register("MiddleTactilTouched", sessionPtr);
  event_register.reset(n, gr);

  gr->startRecord();

  event_register.startProcess();
  event_register.isRecording(true);

  qi::os::sleep(30);

  event_register.isRecording(false);
  event_register.stopProcess();
  gr->stopRecord("");

  app.run();
  app.session()->close();
  return 0;

}
