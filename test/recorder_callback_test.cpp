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

#include <alrosbridge/tools.hpp>
#include <alrosbridge/recorder/recorder.hpp>
#include <alrosbridge/recorder/globalrecorder.hpp>

#include "../src/recorder/int.hpp"
#include "../src/recorder/string.hpp"
#include "../src/recorder/camera.hpp"

int main( int argc, char** argv )
{
  ros::Time::init();

  boost::shared_ptr<alros::recorder::GlobalRecorder> gr = boost::make_shared<alros::recorder::GlobalRecorder>("");

  boost::shared_ptr<alros::recorder::IntRecorder> ir = boost::make_shared<alros::recorder::IntRecorder>( "int" );
  ir->reset( gr );
  ir->subscribe(true);
  boost::shared_ptr<alros::recorder::StringRecorder> sr = boost::make_shared<alros::recorder::StringRecorder>( "string" );
  sr->reset( gr );
  sr->subscribe(true);
  boost::shared_ptr<alros::recorder::CameraRecorder> cr = boost::make_shared<alros::recorder::CameraRecorder>( "camera" );
  cr->reset( gr );
  cr->subscribe(true);

  gr->startRecord();

  int count = 10;
  while (gr->isStarted()) {
    // Int
    std_msgs::Int32 i_msg;
    i_msg.data = 123;
    ir->write(i_msg);

    // String
    std_msgs::String s_msg;
    s_msg.data = "Hello World";
    sr->write(s_msg);

    // Camera
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.stamp = ros::Time::now();
    cam_info_msg.header.frame_id = "CameraDepth_frame";
    cam_info_msg.width = 320;
    cam_info_msg.height = 240;
    cam_info_msg.K = boost::array<double, 9>{{ 525/2.0f, 0, 319.5000000/2.0f, 0, 525/2.0f, 239.5000000000000/2.0f, 0, 0, 1  }};
    cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
    cam_info_msg.P = boost::array<double, 12>{{ 525/2.0f, 0, 319.500000/2.0f, 0, 0, 525/2.0f, 239.5000000000/2.0f, 0, 0, 0, 1, 0 }};
    sensor_msgs::ImagePtr img;
    cv::Mat cv_img = cv::Mat::eye(3,3,CV_64FC1);
    img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();
    img->header.stamp = ros::Time::now();
    img->header.frame_id = "CameraTop_optical_frame";
    cr->write(img, cam_info_msg);

    qi::os::msleep(100);

    count--;
    if (count<0) {
      gr->stopRecord("");
    }
  }

  return 0;

}
