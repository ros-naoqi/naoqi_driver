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

#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

#include "publisher_base.hpp"

#include <boost/thread/mutex.hpp>

#include <qi/anyobject.hpp>
#include <qicore/logmanager.hpp>
#include <qicore/loglistener.hpp>

namespace alros
{
namespace publisher
{

class LogPublisher : public BasePublisher<LogPublisher>
{
public:
  LogPublisher( const std::string& name, const std::string& topic, float frequency, const qi::SessionPtr& sessions );

  void publish();

  void reset( ros::NodeHandle& nh );

  inline bool isSubscribed() const
  {
    // We assume it is essential
    return true;
  }

private:
  ros::Publisher pub_;

  qi::LogManagerPtr logger_;
  qi::LogListenerPtr listener_;
};

} //publisher
} //alros

#endif
