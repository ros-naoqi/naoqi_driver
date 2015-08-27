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

#ifndef BASE_CONVERTER_HPP
#define BASE_CONVERTER_HPP

/*
* LOCAL includes
*/
#include <naoqi_driver/tools.hpp>
#include "../helpers/driver_helpers.hpp"

/*
* ALDEBARAN includes
*/
#include <qi/session.hpp>
#include <qi/anyobject.hpp>

namespace naoqi
{
namespace converter
{

// CRTP
template<class T>
class BaseConverter
{

public:
  BaseConverter( const std::string& name, float frequency, qi::SessionPtr session ):
    name_( name ),
    frequency_( frequency ),
    robot_( helpers::driver::getRobot(session) ),
    session_(session),
    record_enabled_(false)
  {}

  virtual ~BaseConverter() {}

  inline std::string name() const
  {
    return name_;
  }

  inline float frequency() const
  {
    return frequency_;
  }

protected:
  std::string name_;

  /** Frequency at which the converter should turn. This is informative */
  float frequency_;
  /** The type of the robot */
  const robot::Robot& robot_;

  /** Pointer to a session from which we can create proxies */
  qi::SessionPtr session_;

  /** Enable recording */
  bool record_enabled_;
}; // class

} // converter
} // naoqi

#endif
