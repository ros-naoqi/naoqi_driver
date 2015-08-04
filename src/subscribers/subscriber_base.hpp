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

#ifndef BASE_SUBSCRIBER_HPP
#define BASE_SUBSCRIBER_HPP

/*
 * ALDEBARAN includes
 */
#include <qi/session.hpp>

/*
 * LOCAL includes
 */
#include <naoqi_driver/tools.hpp>
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
namespace subscriber
{

// CRTP
template<class T>
class BaseSubscriber
{

public:
  BaseSubscriber( const std::string& name, const std::string& topic, qi::SessionPtr session ):
    name_( name ),
    topic_( topic ),
    is_initialized_( false ),
    robot_( helpers::driver::getRobot(session) ),
    session_(session)
  {}

  virtual ~BaseSubscriber() {}

  inline std::string name() const
  {
    return name_;
  }

  inline std::string topic() const
  {
    return topic_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

protected:
  std::string name_, topic_;

  bool is_initialized_;

  /** The type of the robot */
  const robot::Robot& robot_;

  /** Pointer to a session from which we can create proxies */
  qi::SessionPtr session_;
}; // class

} // subscriber
} // naoqi

#endif
