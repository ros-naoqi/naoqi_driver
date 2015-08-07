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
#include "diagnostics.hpp"
#include "../tools/from_any_value.hpp"

/*
* ROS includes
*/
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

/*
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace
{
void setMessageFromStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
{
  if (status.level == diagnostic_msgs::DiagnosticStatus::OK) {
    status.message = "OK";
  } else if (status.level == diagnostic_msgs::DiagnosticStatus::WARN) {
    status.message = "WARN";
  } else {
    status.message = "ERROR";
  }
}
}

namespace naoqi
{
namespace converter
{

DiagnosticsConverter::DiagnosticsConverter( const std::string& name, float frequency, const qi::SessionPtr& session ):
    BaseConverter( name, frequency, session ),
    p_memory_(session->service("ALMemory")),
    p_body_temperature_(session->service("ALBodyTemperature")),
    temperature_warn_level_(68),
    temperature_error_level_(74)
{
  // Allow for temperature reporting (for CPU)
  p_body_temperature_.call<void>("setEnableNotifications", true);

  // Get all the joint names
  qi::AnyObject p_motion = session->service("ALMotion");
  joint_names_ = p_motion.call<std::vector<std::string> >("getBodyNames", "JointActuators" );

  for(std::vector<std::string>::const_iterator it = joint_names_.begin(); it != joint_names_.end(); ++it) {
    all_keys_.push_back(std::string("Device/SubDeviceList/") + (*it) + std::string("/Temperature/Sensor/Value"));
    all_keys_.push_back(std::string("Device/SubDeviceList/") + (*it) + std::string("/Hardness/Actuator/Value"));
  }

  // Get all the battery keys
  all_keys_.push_back(std::string("BatteryChargeChanged"));
  all_keys_.push_back(std::string("BatteryPowerPluggedChanged"));
  all_keys_.push_back(std::string("BatteryFullChargedFlagChanged"));
  all_keys_.push_back(std::string("Device/SubDeviceList/Battery/Current/Sensor/Value"));

  std::string battery_status_keys[] = {"Charging", "Fully Charged"};
  battery_status_keys_ = std::vector<std::string>(battery_status_keys, battery_status_keys+2);

  // Get the CPU keys
  // TODO check that: it is apparently always -1 ...
  //all_keys_.push_back(std::string("HeadProcessorIsHot"));

  // TODO get ID from Device/DeviceList/ChestBoard/BodyId
}

void DiagnosticsConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  diagnostic_msgs::DiagnosticArray msg;
  msg.header.stamp = ros::Time::now();

  // Get all the keys
  //qi::details::printMetaObject(std::cout, p_memory_.metaObject());
  std::vector<float> values;
  try {
      qi::AnyValue anyvalues = p_memory_.call<qi::AnyValue>("getListData", all_keys_);
      tools::fromAnyValueToFloatVector(anyvalues, values);
  } catch (const std::exception& e) {
    std::cerr << "Exception caught in DiagnosticsConverter: " << e.what() << std::endl;
    return;
  }

  // Fill the temperature / stiffness message for the joints
  double maxTemperature = 0.0;
  double maxStiffness = 0.0;
  double minStiffness = 1.0;
  double minStiffnessWoHands = 1.0;
  std::stringstream hotJointsSS;

  size_t val = 0;
  diagnostic_msgs::DiagnosticStatus::_level_type max_level = diagnostic_msgs::DiagnosticStatus::OK;
  for(size_t i = 0; i < joint_names_.size(); ++i)
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = std::string("naoqi_driver_joints:") + joint_names_[i];

    double temperature = static_cast<double>(values[val++]);
    double stiffness = static_cast<double>(values[val++]);

    // Fill the status data
    status.hardware_id = joint_names_[i];
    status.add("Temperature", temperature);
    status.add("Stiffness", stiffness);

    // Define the level
    if (temperature < temperature_warn_level_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "OK";
    }
    else if (temperature < temperature_error_level_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Hot";
    }
    else
    {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "Too hot";
    }

    msg.status.push_back(status);

    // Fill the joint data for later processing
    max_level = std::max(max_level, status.level);
    maxTemperature = std::max(maxTemperature, temperature);
    maxStiffness = std::max(maxStiffness, stiffness);
    minStiffness = std::min(minStiffness, stiffness);
    if(joint_names_[i].find("Hand") == std::string::npos)
      minStiffnessWoHands = std::min(minStiffnessWoHands, stiffness);
    if(status.level >= (int) diagnostic_msgs::DiagnosticStatus::WARN) {
      hotJointsSS << std::endl << joint_names_[i] << ": " << temperature << "°C";
    }
  }

  // Get the aggregated joints status
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = std::string("naoqi_driver_joints:Status");
    status.hardware_id = "joints";
    status.level = max_level;
    setMessageFromStatus(status);

    status.add("Highest Temperature", maxTemperature);
    status.add("Highest Stiffness", maxStiffness);
    status.add("Lowest Stiffness", minStiffness);
    status.add("Lowest Stiffness without Hands", minStiffnessWoHands);
    status.add("Hot Joints", hotJointsSS.str());

    msg.status.push_back(status);
  }

  // Fill the message for the battery
  {
    int battery_percentage = static_cast<int>(values[val++]);

    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = std::string("naoqi_driver_battery:Status");
    status.hardware_id = "battery";
    status.add("Percentage", battery_percentage);
    // Add the semantic info
    std::stringstream ss;
    for( size_t i = 0; i < battery_status_keys_.size(); ++i) {
      bool value = bool(values[val++]);
      status.add(battery_status_keys_[i], value);

      if (i == 0)
      {
        if (value)
        {
          status.level = diagnostic_msgs::DiagnosticStatus::OK;
          ss << "Charging (" << std::setw(4) << battery_percentage << "%)";
        }
        else
        {
          if (battery_percentage > 60)
          {
              status.level = diagnostic_msgs::DiagnosticStatus::OK;
              ss << "Battery OK (" << std::setw(4) << battery_percentage << "% left)";
          }
          else if (battery_percentage > 30)
          {
              status.level = diagnostic_msgs::DiagnosticStatus::WARN;
              ss << "Battery discharging (" << std::setw(4) << battery_percentage << "% left)";
          }
          else
          {
              status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
              ss << "Battery almost empty (" << std::setw(4) << battery_percentage << "% left)";
          }
        }
      }
      else if ((i == 1) && value)
      {
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
        status.message = "Battery fully charged";
      }
    }
    if (!ss.str().empty())
      status.message = ss.str();

    max_level = status.level;
    msg.status.push_back(status);
  }

  // Process the current battery information
  {
    float current = float(values[val++]);
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = std::string("naoqi_driver_battery:Current");
    status.hardware_id = "battery";
    status.add("Current", current);
    status.level = max_level;
    std::ostringstream ss;
    if (current > 0)
      ss << "Total Current: " << std::setw(5) << current << " Ampere (charging)";
    else
      ss << "Total Current: " << std::setw(5) << current << " Ampere (discharging)";
    status.message = ss.str();

    msg.status.push_back(status);
  }

  // TODO: CPU information should be obtained from system files like done in Python
  // We can still get the temperature
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = std::string("naoqi_driver_computer:CPU");
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    //status.add("Temperature", static_cast<float>(values[val++]));
    // setting to -1 until we find the right key
    status.add("Temperature", static_cast<float>(-1));

    msg.status.push_back(status);
  }

  // TODO: wifi and ethernet statuses should be obtained from DBUS

  for_each( message_actions::MessageAction action, actions )
  {
    callbacks_[action]( msg);
  }

}

void DiagnosticsConverter::reset()
{
}

void DiagnosticsConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

} //converter
} // naoqi
