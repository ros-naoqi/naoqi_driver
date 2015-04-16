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

/**
* LOCAL includes
*/
#include "log.hpp"

namespace alros
{
namespace publisher
{

LogPublisher::LogPublisher( )
  : BasePublisher( "/rosout" )
{
}

void LogPublisher::publish( const rosgraph_msgs::Log& log_msg )
{
  pub_.publish( log_msg );
}

void LogPublisher::reset( ros::NodeHandle& nh )
{
  // We latch as we only publish once
  pub_ = nh.advertise<rosgraph_msgs::Log>( "/rosout", 1 );

  is_initialized_ = true;
}

} // publisher
} //alros
