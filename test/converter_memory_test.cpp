/*
 * ROS
 */
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/image_encodings.h>

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
//#define for_each BOOST_FOREACH

#include <qi/os.hpp>
#include <qi/applicationsession.hpp>

#include <alrosbridge/tools.hpp>
#include <alrosbridge/recorder/recorder.hpp>
#include <alrosbridge/recorder/globalrecorder.hpp>

#include "../src/converters/memory/float.hpp"
#include "../src/recorder/float.hpp"
#include "../src/publishers/basic.hpp"

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

  alros::converter::MemoryFloatConverter conv("name", 5, app.session(), "DarknessDetection/DarknessValue");
  alros::publisher::BasicPublisher<std_msgs::Float32> pub = alros::publisher::BasicPublisher<std_msgs::Float32>( "DarknessDetection/DarknessValue" );
  pub.reset( n );

  //std::cout << conv.convert() << std::endl;
  while (true) {
    //pub.publish(conv.convert());
    //ros::Duration(1).sleep();
  }

  app.run();
  app.session()->close();
  return 0;

}
