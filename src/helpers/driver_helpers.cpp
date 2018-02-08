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
  static qi::Url robot_url;

  if (robot_url == session->url())
  {
    return info;
  }

  robot_url = session->url();

  // Get the robot type
  std::cout << "Receiving information about robot model" << std::endl;
  qi::AnyObject p_memory = session->service("ALMemory");
  std::string robot = p_memory.call<std::string>("getData", "RobotConfig/Body/Type" );
  std::transform(robot.begin(), robot.end(), robot.begin(), ::tolower);

  if (std::string(robot) == "nao")
  {
    info.type = naoqi_bridge_msgs::RobotInfo::NAO;
  }
  if (std::string(robot) == "pepper" || std::string(robot) == "juliette" )
  {
    info.type = naoqi_bridge_msgs::RobotInfo::PEPPER;
  }
  if (std::string(robot) == "romeo" )
  {
    info.type = naoqi_bridge_msgs::RobotInfo::ROMEO;
  }

  // Get the data from RobotConfig
  qi::AnyObject p_motion = session->service("ALMotion");
  std::vector<std::vector<qi::AnyValue> > config = p_motion.call<std::vector<std::vector<qi::AnyValue> > >("getRobotConfig");

  // TODO, fill with the proper string matches from http://doc.aldebaran.com/2-1/naoqi/motion/tools-general-api.html#ALMotionProxy::getRobotConfig

  for (size_t i=0; i<config[0].size();++i)
  {
    if (config[0][i].as<std::string>() == "Model Type")
    {
      try{
        info.model = config[1][i].as<std::string>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Head Version")
    {
      try{
        info.head_version = config[1][i].as<std::string>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Body Version")
    {
      try{
        info.body_version = config[1][i].as<std::string>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Arm Version")
    {
      try{
        info.arm_version = config[1][i].as<std::string>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Laser")
    {
      try{
        info.has_laser = config[1][i].as<bool>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Extended Arms")
    {
      try{
        info.has_extended_arms = config[1][i].as<bool>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Number of Legs")
    {
      try{
        info.number_of_legs = config[1][i].as<int>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Number of Arms")
    {
      try{
        info.number_of_arms = config[1][i].as<int>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

    if (config[0][i].as<std::string>() == "Number of Hands")
    {
      try{
        info.number_of_hands = config[1][i].as<int>();
      }
      catch(const std::exception& e)
      {
        std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
      }
    }

  }
  return info;
}

const robot::Robot& getRobot( const qi::SessionPtr& session )
{
  static robot::Robot robot = robot::UNIDENTIFIED;

  if ( getRobotInfo(session).type == naoqi_bridge_msgs::RobotInfo::NAO )
  {
    robot = robot::NAO;
  }
  if ( getRobotInfo(session).type == naoqi_bridge_msgs::RobotInfo::PEPPER )
  {
    robot = robot::PEPPER;
  }
  if ( getRobotInfo(session).type == naoqi_bridge_msgs::RobotInfo::ROMEO )
  {
    robot = robot::ROMEO;
  }

  return robot;
}

const naoqi_bridge_msgs::RobotInfo& getRobotInfo( const qi::SessionPtr& session )
{
  static naoqi_bridge_msgs::RobotInfo robot_info =  getRobotInfoLocal(session);
  return robot_info;
}

/** Function that sets language for a robot
 */
bool& setLanguage( const qi::SessionPtr& session, naoqi_bridge_msgs::SetStringRequest req)
{
  static bool success;
  std::cout << "Receiving service call of setting speech language" << std::endl;
  try{
    qi::AnyObject p_text_to_speech = session->service("ALTextToSpeech");
    p_text_to_speech.call<void>("setLanguage", req.data);
    success = true;
    return success;
  }
  catch(const std::exception& e){
    success = false;
    return success;
  }
}

/** Function that gets language set to a robot
 */
std::string& getLanguage( const qi::SessionPtr& session )
{
  static std::string language;
  std::cout << "Receiving service call of getting speech language" << std::endl;
  qi::AnyObject p_text_to_speech = session->service("ALTextToSpeech");
  language = p_text_to_speech.call<std::string>("getLanguage");
  return language;
}

/** Function that fades eye leds
 */
std_msgs::Empty& fadeLeds( const qi::SessionPtr& session, naoqi_bridge_msgs::FadeLedsRequest req )
{
  std::cout << "Receiving service call of fading leds" << std::endl;
  qi::AnyObject p_leds = session->service("ALLeds");
  p_leds.call<void>("fadeRGB", req.fade_rgb.led_name, req.fade_rgb.color.r, req.fade_rgb.color.g, req.fade_rgb.color.b, req.fade_rgb.fade_duration.toSec());
}

/** Function that reset leds color
 */
std_msgs::Empty& resetLeds( const qi::SessionPtr& session, naoqi_bridge_msgs::SetStringRequest req )
{
  std::cout << "Receiving service call of resetting leds" << std::endl;
  qi::AnyObject p_leds = session->service("ALLeds");
  p_leds.call<void>("reset", req.data);
}

} // driver
} // helpers
} // naoqi
