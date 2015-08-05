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

#ifndef DIAGNOSTICS_CONVERTER_HPP
#define DIAGNOSTICS_CONVERTER_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"
#include <naoqi_driver/message_actions.h>

/*
* ROS includes
*/
#include <diagnostic_msgs/DiagnosticArray.h>

namespace naoqi
{
namespace converter
{

/**
 * @brief This class defines a Diagnostic converter
 * It does not use the DiagnostricsUpdater for optimization.
 * A full diagnostic_msgs/DiagnosticArray is built and sent to requesting nodes
 */
class DiagnosticsConverter : public BaseConverter<DiagnosticsConverter>
{

  typedef boost::function<void(diagnostic_msgs::DiagnosticArray&) > Callback_t;

public:
  DiagnosticsConverter(const std::string& name, float frequency, const qi::SessionPtr &session );

  void reset();

  void callAll( const std::vector<message_actions::MessageAction>& actions );

  void registerCallback( const message_actions::MessageAction action, Callback_t cb );

private:
  /** The names of the joints in the order given by the motion proxy */
  std::vector<std::string> joint_names_;
  /** all the keys to check. It is a concatenation of joint_temperatures_keys_, battery_keys_ */
  std::vector<std::string> all_keys_;
  /** Keys for the battery status */
  std::vector<std::string> battery_status_keys_;

  /** Proxy to ALMemory */
  qi::AnyObject p_memory_;
  /** Proxy to ALBodyTemperature */
  qi::AnyObject p_body_temperature_;

  float temperature_warn_level_;
  float temperature_error_level_;

  /** Registered Callbacks **/
  std::map<message_actions::MessageAction, Callback_t> callbacks_;
};

} //converter
} // naoqi

#endif
