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
#include "teleop.hpp"


namespace naoqi
{
namespace subscriber
{

TeleopSubscriber::TeleopSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session ):
  BaseSubscriber( name, topic, session ),
  p_motion_( session->service("ALMotion") )
{}

void TeleopSubscriber::reset( ros::NodeHandle& nh )
{
  sub_teleop_ = nh.subscribe( topic_, 10, &TeleopSubscriber::callback, this );

  is_initialized_ = true;
}

void TeleopSubscriber::callback( const geometry_msgs::TwistConstPtr& twist_msg )
{
  // no need to check for max velocity since motion clamps the velocities internally
  const float vel_x = twist_msg->linear.x;
  const float vel_y = twist_msg->linear.y;
  const float vel_th = twist_msg->angular.z;

  std::cout << "going to move x: " << vel_x << " y: " << vel_y << " th: " << vel_th << std::endl;
  p_motion_.async<void>("move", vel_x, vel_y, vel_th );
}

} //publisher
} // naoqi
