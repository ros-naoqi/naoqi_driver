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

#ifndef CONVERTERS_LOG_HPP
#define CONVERTERS_LOG_HPP

#include <rosgraph_msgs/Log.h>

#include <naoqi_driver/message_actions.h>
#include "converter_base.hpp"

#include <qicore/logmanager.hpp>
#include <qicore/loglistener.hpp>

namespace naoqi
{
namespace converter
{

class LogConverter : public BaseConverter<LogConverter>
{

  typedef boost::function<void(rosgraph_msgs::Log&) > Callback_t;

public:
  LogConverter( const std::string& name, float frequency, const qi::SessionPtr& sessions );

  void reset( );

  void registerCallback( const message_actions::MessageAction action, Callback_t cb );

  void callAll( const std::vector<message_actions::MessageAction>& actions );

private:
  /** Function that sets the NAOqi log level to the ROS one */
  void set_qi_logger_level();

  qi::LogManagerPtr logger_;
  /** Log level that is currently translated to ROS */
  qi::LogLevel log_level_;
  qi::LogListenerPtr listener_;

  std::map<message_actions::MessageAction, Callback_t> callbacks_;
};

} //publisher
} //naoqi

#endif
