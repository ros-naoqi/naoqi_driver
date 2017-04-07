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


#ifndef NAVIGATETO_SUBSCRIBER_HPP
#define NAVIGATETO_SUBSCRIBER_HPP

/*
 * LOCAL includes
 */
#include "subscriber_base.hpp"

/*
 * ROS includes
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

namespace naoqi
{
namespace subscriber
{

class NavigatetoSubscriber: public BaseSubscriber<NavigatetoSubscriber>
{
public:
  NavigatetoSubscriber( const std::string& name,
                        const std::string& topic,
                        const qi::SessionPtr& session,
                        const boost::shared_ptr<tf2_ros::Buffer>& tf2_buffer );
  ~NavigatetoSubscriber(){}

  void reset( ros::NodeHandle& nh );
  void callback( const geometry_msgs::PoseStampedConstPtr& pose_msg );

private:
  qi::AnyObject p_navigation_;
  ros::Subscriber sub_navigateto_;
  boost::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
};

} // subscriber
}// naoqi
#endif
