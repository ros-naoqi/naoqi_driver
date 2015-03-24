/*
 * ROS
 */
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
//#define for_each BOOST_FOREACH

#include <qi/os.hpp>

#include <alrosbridge/tools.hpp>
#include <alrosbridge/recorder/recorder.hpp>

/**
 * Emulate different converters here
 */
struct CameraConverter
{
  bool recordEnabled;

  CameraConverter(){
    recordEnabled = false;
  }

  sensor_msgs::CameraInfo convert() {
    std::cout << "Converting camera into ROS msg" << std::endl;
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = "CameraDepth_frame";
    cam_info_msg.width = 320;
    cam_info_msg.height = 240;
    cam_info_msg.K = boost::array<double, 9>{{ 525/2.0f, 0, 319.5000000/2.0f, 0, 525/2.0f, 239.5000000000000/2.0f, 0, 0, 1  }};
    cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
    cam_info_msg.P = boost::array<double, 12>{{ 525/2.0f, 0, 319.500000/2.0f, 0, 0, 525/2.0f, 239.5000000000/2.0f, 0, 0, 0, 1, 0 }};
    return cam_info_msg;
  }

  void setRecordEnabled(bool enabled) {
    recordEnabled = enabled;
  }

  bool isRecordEnabled() {
    return recordEnabled;
  }
};
struct LaserConverter
{
  bool recordEnabled;

  LaserConverter(){
    recordEnabled = false;
  }

  sensor_msgs::LaserScan convert() {
    std::cout << "Converting laser into ROS msg" << std::endl;
    sensor_msgs::LaserScan laser;
    laser.angle_min = 0;
    laser.angle_max = 180;
    laser.ranges = boost::assign::list_of(0)(20)(40)(60)(80)(100);
    return laser;
  }

  void setRecordEnabled(bool enabled) {
    recordEnabled = enabled;
  }

  bool isRecordEnabled() {
    return recordEnabled;
  }
};
struct SonarConverter
{
  bool recordEnabled;

  SonarConverter(){
    recordEnabled = false;
  }

  sensor_msgs::Range convert() {
    std::cout << "Converting sonar into ROS msg" << std::endl;
    sensor_msgs::Range sonar;
    sonar.min_range = 0.25;
    sonar.max_range = 2.55;
    sonar.field_of_view = 0.523598776;
    sonar.radiation_type = sensor_msgs::Range::ULTRASOUND;
    return sonar;
  }

  void setRecordEnabled(bool enabled) {
    recordEnabled = enabled;
  }

  bool isRecordEnabled() {
    return recordEnabled;
  }
};

/*
 * Emulate the startRecord call
 */
void recordAll() {
  alros::Recorder recorder;
  recorder.startRecord();

  int count = 10;
  while (recorder.isStarted()) {

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

    // CameraInfo
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = "CameraDepth_frame";
    cam_info_msg.width = 320;
    cam_info_msg.height = 240;
    cam_info_msg.K = boost::array<double, 9>{{ 525/2.0f, 0, 319.5000000/2.0f, 0, 525/2.0f, 239.5000000000000/2.0f, 0, 0, 1  }};
    cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
    cam_info_msg.P = boost::array<double, 12>{{ 525/2.0f, 0, 319.500000/2.0f, 0, 0, 525/2.0f, 239.5000000000/2.0f, 0, 0, 0, 1, 0 }};
    recorder.write("camera depth", cam_info_msg);

    // JointState
    sensor_msgs::JointState joint;
    joint.position = boost::assign::list_of(0)(1)(2);
    recorder.write("joint state", joint);

    // Laser
    sensor_msgs::LaserScan laser;
    laser.angle_min = 0;
    laser.angle_max = 180;
    laser.ranges = boost::assign::list_of(0)(20)(40)(60)(80)(100);
    recorder.write("laser", laser);

    // TransformStamped
    std::vector<geometry_msgs::TransformStamped> msg;
    geometry_msgs::TransformStamped msg_1;
    msg_1.header.frame_id = "1";
    geometry_msgs::TransformStamped msg_2;
    msg_2.header.frame_id = "1";
    msg.push_back(msg_1);
    msg.push_back(msg_2);
    recorder.write("transform stamped", msg);

    qi::os::msleep(100);

    count--;
    if (count<0) {
      recorder.stopRecord();
    }
  }
}

/*
 * Emulate the startRecordTopics call
 */
void recordByTopics() {
  alros::Recorder recorder;
  CameraConverter cam;
  LaserConverter laser;
  SonarConverter sonar;

  std::cout << "Is camera recording enabled? " << cam.isRecordEnabled() << std::endl
            << "Is laser recording enabled? " << laser.isRecordEnabled() << std::endl
            <<"Is sonar recording enabled? " << sonar.isRecordEnabled() << std::endl;

  recorder.startRecord();

  std::vector<alros::Topics> topics = boost::assign::list_of(alros::Laser)
                                      (alros::Camera)
                                      (alros::Sonar);

  BOOST_FOREACH(const alros::Topics& topic, topics) {
    if (topic==alros::Camera) {
      cam.setRecordEnabled(true);
    }
    else if (topic==alros::Laser) {
      laser.setRecordEnabled(true);
    }
    else if (topic==alros::Sonar) {
      sonar.setRecordEnabled(true);
    }
    else {
      std::cout << "Error in topic naming." << std::endl;
    }
  }

  int count = 10;
  while (recorder.isStarted()) {

    // CameraInfo
    if (cam.isRecordEnabled()) {
      recorder.write("camera", cam.convert());
    }

    // Laser
    if (laser.isRecordEnabled()) {
      recorder.write("laser", laser.convert());
    }

    // Sonar
    if (sonar.isRecordEnabled()) {
      recorder.write("sonar", sonar.convert());
    }

    qi::os::msleep(100);

    count--;
    if (count<0) {
      recorder.stopRecord();
    }
  }
}

int main( int argc, char** argv )
{
  ros::Time::init();
  recordAll();
  recordByTopics();

  return 0;

}
