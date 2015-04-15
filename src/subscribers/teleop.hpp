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


#ifndef TELEOP_SUBSCRIBER_HPP
#define TELEOP_SUBSCRIBER_HPP

/**
 * STANDARD includes
 */
#include <math.h>

/**
 * ROS includes
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

/**
 * ALDEBARAN includes
 */
#include <qi/anyobject.hpp>

/**
 * LOCAL includes
 */
#include "subscriber_base.hpp"
namespace alros
{
namespace subscriber
{

class TeleopSubscriber: public BaseSubscriber<TeleopSubscriber>
{
public:
  TeleopSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session );
  ~TeleopSubscriber(){}

  void reset( ros::NodeHandle& nh );
  void callback( const geometry_msgs::TwistConstPtr& twist_msg );

private:
  qi::AnyObject p_motion_;
  ros::Subscriber sub_teleop_;

}; // class Teleop

} // subscriber
}// alros
#endif
