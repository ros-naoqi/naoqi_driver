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


#ifndef MEMORY_SUBSCRIBER_HPP
#define MEMORY_SUBSCRIBER_HPP

/*
 * LOCAL includes
 */
#include "subscriber_base.hpp"
#include <naoqi_bridge_msgs/MemoryList.h>

/*
 * ROS includes
 */
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace naoqi
{
namespace subscriber
{

class MemorySubscriber: public BaseSubscriber<MemorySubscriber>
{
public:
  MemorySubscriber(const std::string& name, const std::string& memory_topic, const qi::SessionPtr& session );
  ~MemorySubscriber(){}

  void reset( ros::NodeHandle& nh );
  void memory_callback( const naoqi_bridge_msgs::MemoryListConstPtr &msg );

private:

  std::string memory_topic_;

  qi::AnyObject p_memory_;
  ros::Subscriber sub_memory_;



}; // class Memory

} // subscriber
}// naoqi
#endif
