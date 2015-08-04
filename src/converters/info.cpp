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

/*
* LOCAL includes
*/
#include "info.hpp"
#include "../tools/from_any_value.hpp"

/*
* BOOST includes
*/
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace naoqi
{
namespace converter
{

InfoConverter::InfoConverter( const std::string& name, float frequency, const qi::SessionPtr& session )
  : BaseConverter( name, frequency, session ),
    p_memory_( session->service("ALMemory") )
{
  keys_.push_back("RobotConfig/Head/FullHeadId");
  keys_.push_back("Device/DeviceList/ChestBoard/BodyId");
  keys_.push_back("RobotConfig/Body/Type");
  keys_.push_back("RobotConfig/Body/BaseVersion");
  keys_.push_back("RobotConfig/Body/Device/LeftArm/Version");
  keys_.push_back("RobotConfig/Body/Device/RightArm/Version");
  keys_.push_back("RobotConfig/Body/Device/Hand/Left/Version");
  keys_.push_back("RobotConfig/Body/Version");
  keys_.push_back("RobotConfig/Body/SoftwareRequirement");
  keys_.push_back("RobotConfig/Body/Device/Legs/Version");
  if(robot_ == robot::PEPPER)
  {
    keys_.push_back("Device/DeviceList/BatteryFuelGauge/SerialNumber");
    keys_.push_back("Device/DeviceList/BatteryFuelGauge/FirmwareVersion");
    keys_.push_back("RobotConfig/Body/Device/Platform/Version");
    keys_.push_back("RobotConfig/Body/Device/Brakes/Version");
    keys_.push_back("RobotConfig/Body/Device/Wheel/Version");
  }
}

void InfoConverter::reset()
{
}

void InfoConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void InfoConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  std::vector<std::string> values;
  try {
      qi::AnyValue anyvalues = p_memory_.call<qi::AnyValue>("getListData", keys_);
      tools::fromAnyValueToStringVector(anyvalues, values);
  } catch (const std::exception& e) {
    std::cerr << "Exception caught in InfoConverter: " << e.what() << std::endl;
    return;
  }

  naoqi_bridge_msgs::StringStamped msg;

  msg.header.stamp = ros::Time::now();
  for(size_t i = 0; i < keys_.size(); ++i)
  {
    msg.data += keys_[i] + ": " + values[i];
    if (i != keys_.size()-1)
    msg.data += " ; ";
  }
  for_each( const message_actions::MessageAction& action, actions )
  {
    callbacks_[action](msg);
  }
}

} // converter
} //naoqi
