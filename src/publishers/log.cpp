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

#include <ros/serialization.h>
#include <std_msgs/String.h>

#include <boost/algorithm/string.hpp>

namespace alros
{
namespace publisher
{

/** mutex to change the logs list */
boost::mutex MUTEX_LOGS;
/** list of ogs in which the NAOqi callback will write its logs */
std::list<rosgraph_msgs::Log> LOGS;
/** Vector where at index NAOQI_LOG_LEVEL, there is the matching ROS level */
std::vector<rosgraph_msgs::Log::_level_type> LOG_LEVELS;
/** Latest time at which the updates happened */
ros::Time last_log_update_ = ros::TIME_MIN;

void logCallback(const qi::LogMessage& msg)
{
  // Convert the NAOqi log to a ROS log
  rosgraph_msgs::Log log;

  std::vector<std::string> results;
  boost::split(results, msg.source, boost::is_any_of(":"));
  log.file = results[0];
  log.function = results[1];
  log.line = atoi(results[2].c_str());
  log.level = LOG_LEVELS[msg.level];
  log.name = msg.category;
  log.msg = msg.message;
  log.header.stamp = ros::Time(msg.timestamp.tv_sec, msg.timestamp.tv_usec);

  // If we are not publishing, the queue will increase, so we have to prevent an explosion
  // We only keep a log if it's within 5 second of the last publish (totally arbitrary)
  if ((log.header.stamp - last_log_update_) < ros::Duration(5))
  {
    boost::mutex::scoped_lock lock( MUTEX_LOGS );
    LOGS.push_back(log);
  }
}

LogPublisher::LogPublisher( const std::string& name, const std::string& topic, float frequency, const qi::SessionPtr& session )
  : BasePublisher( name, topic, frequency, session ),
    logger_( session->service("LogManager") )
{
  float levels[] = {rosgraph_msgs::Log::DEBUG, rosgraph_msgs::Log::FATAL, rosgraph_msgs::Log::ERROR,
    rosgraph_msgs::Log::WARN, rosgraph_msgs::Log::INFO, rosgraph_msgs::Log::DEBUG, rosgraph_msgs::Log::DEBUG};
  LOG_LEVELS = std::vector<rosgraph_msgs::Log::_level_type>(levels, levels + 7);

  listener_ = logger_->getListener();
  listener_->onLogMessage.connect(logCallback);
}

void LogPublisher::publish()
{
  while ( !LOGS.empty() )
  {
    pub_.publish(LOGS.front());
    {
      boost::mutex::scoped_lock lock( MUTEX_LOGS );
      LOGS.pop_front();
    }
  }
  last_log_update_ = ros::Time::now();
}

void LogPublisher::reset( ros::NodeHandle& nh )
{
  // We latch as we only publish once
  pub_ = nh.advertise<rosgraph_msgs::Log>( "/rosout", 1 );

  is_initialized_ = true;
}

} // publisher
} //alros
