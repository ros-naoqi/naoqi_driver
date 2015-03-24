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

#ifndef RECORDER_HPP
#define RECORDER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
# include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/TransformStamped.h>

#include <alrosbridge/tools.hpp>

namespace alros
{
namespace recorder
{

/**
* @brief Recorder concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/
class Recorder
{

public:

  /**
  * @brief Constructor for recorder interface
  */
  Recorder();

  /**
  * @brief Initialize the recording of the ROSbag
  */
  void startRecord();

  /**
  * @brief Terminate the recording of the ROSbag
  */
  void stopRecord();

  /**
  * @brief Insert data into the ROSbag
  */
  template <class T>
  void write(const std::string& topic, const T& msg) {
    boost::mutex::scoped_lock writeLock( _processMutex );
    _bag.write(topic, ros::Time::now(), msg);
  }

  void write(const std::string& topic, const std::vector<geometry_msgs::TransformStamped>& msgtf);

  /**
  * @brief Check if the ROSbag is opened
  */
  bool isStarted();

private:
  boost::mutex _processMutex;
  rosbag::Bag _bag;
  std::string _nameBag;
  bool _isStarted;

  // TOPICS
  std::vector<Topics> _topics;

}; // class recorder
} // recorder
} //alros

#endif
