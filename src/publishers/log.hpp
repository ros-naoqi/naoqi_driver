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

#ifndef PUBLISHER_LOG_HPP
#define PUBLISHER_LOG_HPP

/*
* LOCAL includes
*/
#include "basic.hpp"

/*
* ROS includes
*/
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <ros/serialization.h>
#include <std_msgs/String.h>

/*
* BOOST includes
*/
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>

/*
* ALDEBARAN includes
*/
#include <qi/anyobject.hpp>
#include <qicore/logmessage.hpp>
#include <qicore/logmanager.hpp>
#include <qicore/loglistener.hpp>

namespace naoqi
{
namespace publisher
{

class LogPublisher : public BasicPublisher<rosgraph_msgs::Log>
{
public:
  LogPublisher(const std::string& topic);

  // check whether a real copy of the log message should be more safe
  // remove const ref here

  inline bool isSubscribed() const
  {
    // We assume it is essential
    return true;
  }

private:
  ros::Publisher pub_;

};

} //publisher
} //naoqi

#endif
