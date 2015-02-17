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

#ifndef PUBLISHER_SONAR_HPP
#define PUBLISHER_SONAR_HPP

#include <ros/ros.h>
#include <qi/anyobject.hpp>

#include <sensor_msgs/Range.h>

#include "publisher_base.hpp"

namespace alros
{
namespace publisher
{

class SonarPublisher : public BasePublisher<SonarPublisher>
{
public:
  SonarPublisher( const std::string& name, const std::string& topic, float frequency, const qi::AnyObject& p_memory, const qi::AnyObject& p_video );

  ~SonarPublisher();

  void publish();

  void reset( ros::NodeHandle& nh );

  inline bool isSubscribed() const
  {
    if (is_initialized_ == false) return false;
    for(std::vector<ros::Publisher>::const_iterator it = pubs_.begin(); it != pubs_.end(); ++it)
      if (it->getNumSubscribers())
        return true;
    return false;
  }

private:
  std::vector<ros::Publisher> pubs_;

  /** Sonar (Proxy) configurations */
  qi::AnyObject p_sonar_;
  /** Memory (Proxy) configurations */
  qi::AnyObject p_memory_;
  /** Key describeing whether we are subscribed to the ALSonar module */
  bool is_subscribed_;

  /** The memory keys of the sonars */
  AL::ALValue keys_;
  /** The frames of the sonars */
  std::vector<std::string> frames_;
  /** The topics that will be published */
  std::vector<std::string> topics_;
  /** Pre-filled messges that are sent */
  std::vector<sensor_msgs::Range> msgs_;
};

} //publisher
} //alros

#endif
