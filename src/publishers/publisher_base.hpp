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

#ifndef BASE_PUBLISHER_HPP
#define BASE_PUBLISHER_HPP

#include <algorithm>
#include <iostream>
#include <string>

#include <alvalue/alvalue.h>
#include <qi/session.hpp>

#include <alrosbridge/tools.hpp>

namespace alros
{
namespace publisher
{

// CRTP
template<class T>
class BasePublisher
{

public:
  BasePublisher( const std::string& name, const std::string& topic, float frequency, qi::SessionPtr session ):
    name_( name ),
    topic_( topic ),
    frequency_( frequency ),
    is_initialized_( false ),
    robot_( UNIDENTIFIED ),
    session_(session)
  {}

  virtual ~BasePublisher() {};

  inline std::string name() const
  {
    return name_;
  }

  inline std::string topic() const
  {
    return topic_;
  }

  inline float frequency() const
  {
    return frequency_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  /** Function that returns the type of a robot
   */
  inline Robot robot() const
  {
    if (robot_ != UNIDENTIFIED)
      return robot_;

    qi::AnyObject p_memory = session_->service("ALMemory");
    std::string robot = p_memory.call<AL::ALValue>("getData", "RobotConfig/Body/Type" );
    std::transform(robot.begin(), robot.end(), robot.begin(), ::tolower);

    if (std::string(robot) == "nao")
      robot_ = NAO;
    else if (std::string(robot) == "pepper")
      robot_ = PEPPER;
    else
      robot_ = UNIDENTIFIED;
    return robot_;
  }

  virtual bool isSubscribed() const = 0;

protected:
  std::string name_, topic_;

  bool is_initialized_;

  /** Frequency at which the publisher should publish. This is informative */
  float frequency_;
  /** The type of the robot */
  mutable Robot robot_;

  /** Pointer to a session from which we can create proxies */
  qi::SessionPtr session_;
}; // class

} //publisher
} // alros

#endif
