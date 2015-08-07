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

#include "log.hpp"

#include <qicore/logmessage.hpp>
#include <queue>

#include <std_msgs/String.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/foreach.hpp>

#include <ros/console.h>

#define for_each BOOST_FOREACH

namespace naoqi
{
namespace converter
{

/** mutex to change the logs list */
boost::mutex MUTEX_LOGS;
/** list of ogs in which the NAOqi callback will write its logs */
std::queue<rosgraph_msgs::Log> LOGS;
/** Generic Log Level used to store all the correspondences between the libqi and ROS log levels */
class LogLevel
{
public:
  LogLevel(qi::LogLevel qi, rosgraph_msgs::Log::_level_type ros_msg, ros::console::levels::Level ros_console) :
      qi_(qi), ros_msg_(ros_msg), ros_console_(ros_console)
  {
    all_.push_back(*this);
  }

  static const LogLevel& get_from_qi(qi::LogLevel qi)
  {
    for_each(const LogLevel& log_level, all_)
      if (log_level.qi_ == qi)
        return log_level;
  }

  static const LogLevel& get_from_ros_msg(rosgraph_msgs::Log::_level_type ros_msg)
  {
    for_each(const LogLevel& log_level, all_)
      if (log_level.ros_msg_ == ros_msg)
        return log_level;
  }

  static const LogLevel& get_from_ros_console(ros::console::levels::Level ros_console)
  {
    for_each(const LogLevel& log_level, all_)
      if (log_level.ros_console_ == ros_console)
        return log_level;
  }

  qi::LogLevel qi_;
  rosgraph_msgs::Log::_level_type ros_msg_;
  ros::console::levels::Level ros_console_;

private:
  static std::vector<LogLevel> all_;
};

std::vector<LogLevel> LogLevel::all_ = std::vector<LogLevel>();

/** Callback called for each libqi log message
 */
void logCallback(const qi::LogMessage& msg)
{
  // Convert the NAOqi log to a ROS log
  rosgraph_msgs::Log log;

  std::vector<std::string> results;
  boost::split(results, msg.source, boost::is_any_of(":"));
  log.file = results[0];
  log.function = results[1];
  log.line = atoi(results[2].c_str());
  log.level = LogLevel::get_from_qi(msg.level).ros_msg_;
  log.name = msg.category;
  log.msg = msg.message;
  log.header.stamp = ros::Time(msg.timestamp.tv_sec, msg.timestamp.tv_usec);

  // If we are not publishing, the queue will increase, so we have to prevent an explosion
  // We only keep a log if it's within 5 second of the last publish (totally arbitrary)
  boost::mutex::scoped_lock lock( MUTEX_LOGS );
  while (LOGS.size() > 1000)
  {
    LOGS.pop();
  }
  LOGS.push(log);
}

LogConverter::LogConverter( const std::string& name, float frequency, const qi::SessionPtr& session )
  : BaseConverter( name, frequency, session ),
    logger_( session->service("LogManager") ),
    // Default log level is info
    log_level_(qi::LogLevel_Info)
{
  // Define the log equivalents
  LogLevel(qi::LogLevel_Silent, rosgraph_msgs::Log::DEBUG, ros::console::levels::Debug);
  LogLevel(qi::LogLevel_Fatal, rosgraph_msgs::Log::FATAL, ros::console::levels::Fatal);
  LogLevel(qi::LogLevel_Error, rosgraph_msgs::Log::ERROR, ros::console::levels::Error);
  LogLevel(qi::LogLevel_Warning, rosgraph_msgs::Log::WARN, ros::console::levels::Warn);
  LogLevel(qi::LogLevel_Info, rosgraph_msgs::Log::INFO, ros::console::levels::Info);
  LogLevel(qi::LogLevel_Verbose, rosgraph_msgs::Log::DEBUG, ros::console::levels::Debug);
  LogLevel(qi::LogLevel_Debug, rosgraph_msgs::Log::DEBUG, ros::console::levels::Debug);

  listener_ = logger_->getListener();
  set_qi_logger_level();
  listener_->onLogMessage.connect(logCallback);
}

void LogConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void LogConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  while ( !LOGS.empty() )
  {
    rosgraph_msgs::Log& log_msg = LOGS.front();
    for_each( const message_actions::MessageAction& action, actions)
    {
      callbacks_[action](log_msg);
    }
    {
      boost::mutex::scoped_lock lock( MUTEX_LOGS );
      LOGS.pop();
    }
  }
  set_qi_logger_level();
}

void LogConverter::reset( )
{
}

void LogConverter::set_qi_logger_level( )
{
  // Check that the log level is above or equal to the current one
  std::map<std::string, ros::console::levels::Level> loggers;
  ros::console::get_loggers(loggers);

  std::map<std::string, ros::console::levels::Level>::iterator iter = loggers.find("ros.naoqi_driver");

  if (iter == loggers.end())
    return;

  qi::LogLevel new_level = LogLevel::get_from_ros_console(iter->second).qi_;
  // Only change the log level if it has changed (otherwise, there is a flood of warnings)
  if (new_level == log_level_)
      return;

  log_level_ = new_level;
  qi::log::setLogLevel(log_level_);
}

} // publisher
} //naoqi
