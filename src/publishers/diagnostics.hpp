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

#ifndef DIAGNOSTICS_PUBLISHER_HPP
#define DIAGNOSTICS_PUBLISHER_HPP

#include "publisher_base.hpp"

/**
* ROS includes
*/
#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>

namespace alros
{
namespace publisher
{

class DiagnosticsPublisher : public BasePublisher<DiagnosticsPublisher>
{

public:
  DiagnosticsPublisher( );

  void publish( diagnostic_msgs::DiagnosticArray& msg );

  void reset( ros::NodeHandle& nh );

  inline bool isSubscribed() const
  {
    // Should always be publishe: it is the convention
    // and it a low frame rate
    return true;
  }

private:
  ros::Publisher pub_;
}; // class

} //publisher
} // alros

#endif
