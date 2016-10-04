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
#include "odom.hpp"
#include "../tools/from_any_value.hpp"

/*
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace naoqi
{
namespace converter
{
 
OdomConverter::OdomConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session ):
  BaseConverter( name, frequency, session ),
  p_motion_( session->service("ALMotion") )
  
{
}

void OdomConverter::registerCallback( message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void OdomConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  
  int FRAME_WORLD = 1;
  bool use_sensor = true;
  // documentation of getPosition available here: http://doc.aldebaran.com/2-1/naoqi/motion/control-cartesian.html
  std::vector<float> al_odometry_data = p_motion_.call<std::vector<float> >( "getPosition", "Torso", FRAME_WORLD, use_sensor );
  
  const ros::Time& odom_stamp = ros::Time::now();
  std::vector<float> al_speed_data = p_motion_.call<std::vector<float> >( "getRobotVelocity" );
  
  const float& odomX  =  al_odometry_data[0];
  const float& odomY  =  al_odometry_data[1];
  const float& odomZ  =  al_odometry_data[2];
  const float& odomWX =  al_odometry_data[3];
  const float& odomWY =  al_odometry_data[4];
  const float& odomWZ =  al_odometry_data[5];
  
  const float& dX = al_speed_data[0];
  const float& dY = al_speed_data[1];
  const float& dWZ = al_speed_data[2];

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  tf2::Quaternion tf_quat;
  tf_quat.setRPY( odomWX, odomWY, odomWZ );
  geometry_msgs::Quaternion odom_quat = tf2::toMsg( tf_quat );

  static nav_msgs::Odometry msg_odom;
  msg_odom.header.frame_id = "odom";
  msg_odom.child_frame_id = "base_link";
  msg_odom.header.stamp = odom_stamp;

  msg_odom.pose.pose.orientation = odom_quat;
  msg_odom.pose.pose.position.x = odomX;
  msg_odom.pose.pose.position.y = odomY;
  msg_odom.pose.pose.position.z = odomZ;
  
  msg_odom.twist.twist.linear.x = dX;
  msg_odom.twist.twist.linear.y = dY;
  msg_odom.twist.twist.linear.z = 0;
  
  msg_odom.twist.twist.angular.x = 0;
  msg_odom.twist.twist.angular.y = 0;
  msg_odom.twist.twist.angular.z = dWZ;

  for_each( message_actions::MessageAction action, actions )
  {
    callbacks_[action](msg_odom);
    
  }
}

void OdomConverter::reset( )
{
}

} //converter
} // naoqi
