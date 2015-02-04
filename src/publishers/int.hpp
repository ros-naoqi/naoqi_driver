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

#ifndef INT_PUBLISHER_HPP
#define INT_PUBLISHER_HPP

#include <ros/ros.h>

#include "publisher_base.hpp"

namespace alros
{
namespace publisher
{

class IntPublisher : public BasePublisher<IntPublisher>
{

public:
  IntPublisher( const std::string& name, const std::string& topic, float frequency );

  void publish();

  void reset( ros::NodeHandle& nh );

  inline bool isSubscribed() const
  {
    if (is_initialized_ == false) return false;
    return pub_.getNumSubscribers() > 0;
  }

private:
  ros::Publisher pub_;
}; // class

} //publisher
} // alros

#endif
