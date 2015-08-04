/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GLOBALRECORDER_HPP
#define GLOBALRECORDER_HPP

/*
* LOCAL includes
*/
#include <naoqi_driver/tools.hpp>

/*
* STANDARD includes
*/
#include <string>

/*
* BOOST includes
*/
# include <boost/thread/mutex.hpp>

/*
* ROS includes
*/
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/TransformStamped.h>



namespace naoqi
{
namespace recorder
{

/**
* @brief GlobalRecorder concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/
class GlobalRecorder
{

public:

  /**
  * @brief Constructor for recorder interface
  */
  GlobalRecorder(const std::string& prefix_topic);

  /**
  * @brief Initialize the recording of the ROSbag
  */
  void startRecord(const std::string& prefix_bag = "");

  /**
  * @brief Terminate the recording of the ROSbag
  */
  std::string stopRecord(const std::string& robot_ip = "<ROBOT_IP>");

  /**
  * @brief Insert data into the ROSbag
  */
  template <class T>
  void write(const std::string& topic, const T& msg, const ros::Time& time = ros::Time::now() ) {
    std::string ros_topic;
    if (topic[0]!='/')
    {
      ros_topic = _prefix_topic+topic;
    }
    else
    {
      ros_topic = topic;
    }
    ros::Time time_msg = time;
    boost::mutex::scoped_lock writeLock( _processMutex );
    if (_isStarted) {
      _bag.write(ros_topic, time_msg, msg);
    }
  }

  void write(const std::string& topic, const std::vector<geometry_msgs::TransformStamped>& msgtf);

  /**
  * @brief Check if the ROSbag is opened
  */
  bool isStarted();

private:
  std::string _prefix_topic;
  boost::mutex _processMutex;
  rosbag::Bag _bag;
  std::string _nameBag;
  bool _isStarted;

  // TOPICS
  std::vector<Topics> _topics;

}; // class globalrecorder
} // recorder
} //naoqi

#endif
