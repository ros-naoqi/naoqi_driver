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

/*
* ROS includes
*/
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

namespace naoqi
{
namespace publisher
{

class SonarPublisher
{
public:
  SonarPublisher( const std::vector<std::string>& topics );

  inline std::string topic() const
  {
    return "sonar";
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  void publish( const std::vector<sensor_msgs::Range>& sonar_msgs );

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
  std::vector<std::string> topics_;
  std::vector<ros::Publisher> pubs_;
  bool is_initialized_;

};

} //publisher
} //naoqi

#endif
