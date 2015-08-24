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

#include "driver_helpers.hpp"

namespace naoqi
{
namespace helpers
{
namespace driver
{

/** Function that returns the type of a robot
 */
static naoqi_bridge_msgs::RobotInfo& getRobotInfoLocal( const qi::SessionPtr& session)
{
  static naoqi_bridge_msgs::RobotInfo info;
  static qi::SessionPtr session_old;

  if (session_old == session)
    return info;

  session_old = session;

  // Get the robot type
  std::cout << "I am going to call RobotModel proxy" << std::endl;
  qi::AnyObject p_memory = session->service("ALMemory");
  std::string robot = p_memory.call<std::string>("getData", "RobotConfig/Body/Type" );
  std::transform(robot.begin(), robot.end(), robot.begin(), ::tolower);

  if (std::string(robot) == "nao")
  {
    info.robot_model = naoqi_bridge_msgs::RobotInfo::NAO;
  }
  if (std::string(robot) == "pepper" || std::string(robot) == "juliette" )
  {
    info.robot_model = naoqi_bridge_msgs::RobotInfo::PEPPER;
  }
  if (std::string(robot) == "romeo" )
  {
    info.robot_model = naoqi_bridge_msgs::RobotInfo::ROMEO;
  }

  // Get the data from RobotConfig
  qi::AnyObject p_motion = session->service("ALMotion");
  std::vector<std::vector<qi::AnyValue> > config = p_motion.call<std::vector<std::vector<qi::AnyValue> > >("getRobotConfig");

  for (size_t i=0; i<config[0].size(); ++i)
  {
    // TODO, fill with the proper string matches from http://doc.aldebaran.com/2-1/naoqi/motion/tools-general-api.html#ALMotionProxy::getRobotConfig

//    if (config[1][i].kind() == qi::TypeKind_Int)
//      info.config_int_[config[0][i].asString()] = config[1][i].toInt();
//    if (config[1][i].kind() == qi::TypeKind_String)
//      info.config_string_[config[0][i].asString()] = config[1][i].asString();
  }

  return info;
}

robot::Robot getRobot( const qi::SessionPtr& session )
{
  // TODO: return the type from naoqi_bridge_msgs::RobotInfo
  if ( getRobotInfo(session).robot_model == naoqi_bridge_msgs::RobotInfo::NAO )
    return robot::NAO;
  if ( getRobotInfo(session).robot_model == naoqi_bridge_msgs::RobotInfo::PEPPER )
    return robot::PEPPER;
}

const naoqi_bridge_msgs::RobotInfo& getRobotInfo( const qi::SessionPtr& session )
{
  return getRobotInfoLocal(session);
}

} // driver
} // helpers
} // naoqi
