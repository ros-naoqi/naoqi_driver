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
#include "bool.hpp"

namespace naoqi
{
namespace publisher
{

MemoryBoolPublisher::MemoryBoolPublisher( const std::string& topic ):
  BasePublisher( topic )
{}

void MemoryBoolPublisher::publish(const naoqi_bridge_msgs::BoolStamped& msg )
{
  pub_.publish( msg );
}

void MemoryBoolPublisher::reset( ros::NodeHandle& nh )
{
  pub_ = nh.advertise< naoqi_bridge_msgs::BoolStamped >( topic_, 10 );
  is_initialized_ = true;
}

} //publisher
} // naoqi
