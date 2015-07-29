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


#ifndef BRIDGE_HELPERS_HPP
#define BRIDGE_HELPERS_HPP

#include <alrosbridge/tools.hpp>

#include <qi/applicationsession.hpp>

namespace alros
{
namespace helpers
{
namespace bridge
{

/** Function that returns the type of a robot
 */
inline robot::Robot robot( const qi::SessionPtr& session)
{
  robot::Robot bot;

  qi::AnyObject p_memory = session->service("ALMemory");
  std::string robot = p_memory.call<std::string>("getData", "RobotConfig/Body/Type" );
  std::transform(robot.begin(), robot.end(), robot.begin(), ::tolower);

  if (std::string(robot) == "nao")
  {
    bot = robot::NAO;
    return bot;
  }
  if (std::string(robot) == "pepper" || std::string(robot) == "juliette" )
  {
    bot = robot::PEPPER;
    return bot;
  }
  else
  {
    return robot::UNIDENTIFIED;
  }
}

static const robot::Robot& getRobot( const qi::SessionPtr& session )
{
  static const robot::Robot r = robot( session );
  return r;
}

} // naoqi
} // helpers
} // alros

#endif
