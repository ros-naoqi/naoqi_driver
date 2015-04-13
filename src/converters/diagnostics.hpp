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

#include <diagnostic_updater/diagnostic_updater.h>

/**
* ALDEBARAN includes
*/
#include <alvalue/alvalue.h>
#include <qi/anyobject.hpp>

namespace alros
{
namespace publisher
{

class DiagnosticsPublisher : public BasePublisher<DiagnosticsPublisher>
{

public:
  DiagnosticsPublisher( const std::string& name, float frequency, qi::SessionPtr& session );

  void publish();

  void reset( ros::NodeHandle& nh );

  inline bool isSubscribed() const
  {
    // Should always be publishe: it is the convention
    // and it a low frame rate
    return true;
  }

private:
  ros::Publisher pub_;
  ros::Publisher pub2_;

  /** The names of the joints in the order given by the motion proxy */
  std::vector<std::string> joint_names_;
  /** The list of the ALMemory keys for joint temperatures */
  std::vector<std::string> joint_temperatures_keys_;
  /** The list of the ALMemory keys for the battery */
  std::vector<std::string> battery_keys_;
  /** all the keys to check. It is a concatenation of joint_temperatures_keys_, battery_keys_ */
  AL::ALValue all_keys_;
  /** Keys for the battery status */
  std::vector<std::string> battery_status_keys_;

  /** Proxy to ALMemory */
  qi::AnyObject p_memory_;

  float temperature_warn_level_;
  float temperature_error_level_;
}; // class

} //publisher
} // alros

#endif
