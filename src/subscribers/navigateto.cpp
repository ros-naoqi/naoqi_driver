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
#include "navigateto.hpp"

/*
 * ROS includes
 */
//#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "../helpers/transform_helpers.hpp"

namespace naoqi
{
namespace subscriber
{

NavigatetoSubscriber::NavigatetoSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session,
                                    const boost::shared_ptr<tf2_ros::Buffer>& tf2_buffer):
  BaseSubscriber( name, topic, session ),
  p_navigation_( session->service("ALNavigation") ),
  tf2_buffer_( tf2_buffer )
{}

void NavigatetoSubscriber::reset( ros::NodeHandle& nh )
{
  sub_navigateto_ = nh.subscribe( topic_, 10, &NavigatetoSubscriber::callback, this );
  is_initialized_ = true;
}

void NavigatetoSubscriber::callback( const geometry_msgs::PoseStampedConstPtr& pose_msg )
{
  if ( pose_msg->header.frame_id == "base_footprint" )
  {
    double yaw = helpers::transform::getYaw(pose_msg->pose);

    std::cout << "going to navigate x: " << pose_msg->pose.position.x
              << " y: " << pose_msg->pose.position.y
              << " z: " << pose_msg->pose.position.z
              << " yaw: " << yaw << std::endl;
    p_navigation_.async<void>("navigateTo",
                              pose_msg->pose.position.x,
                              pose_msg->pose.position.y,
                              yaw);
  }
  else if (pose_msg->header.frame_id == "map")
  {
    double yaw = helpers::transform::getYaw(pose_msg->pose);
    std::cout << "map to navigate x: " << pose_msg->pose.position.x
              << " y: " << pose_msg->pose.position.y
              << " z: " << pose_msg->pose.position.z
              << " yaw: " << yaw << std::endl;
    std::vector<float> pose(3);
    pose[0] = pose_msg->pose.position.x;
    pose[1] = pose_msg->pose.position.y;
    pose[2] = yaw;
    p_navigation_.call<void>("navigateToInMap", pose);
  }
  else
  {
    geometry_msgs::PoseStamped pose_msg_bf;
    bool canTransform = tf2_buffer_->canTransform("base_footprint",
                                                  pose_msg->header.frame_id,
                                                  ros::Time(0),
                                                  ros::Duration(2) );
    if (!canTransform)
    {
      std::cout << "Cannot transform from " << pose_msg->header.frame_id
                << " to base_footprint" << std::endl;
      return;
    }
    try
    {
      tf2_buffer_->transform( *pose_msg, pose_msg_bf, "base_footprint", ros::Time(0), pose_msg->header.frame_id );
      double yaw = helpers::transform::getYaw(pose_msg_bf.pose);
      std::cout << "odom to navigate x: " << pose_msg_bf.pose.position.x
                << " y: " << pose_msg_bf.pose.position.y
                << " z: " << pose_msg_bf.pose.position.z
                << " yaw: " << yaw << std::endl;
      p_navigation_.async<void>("navigateTo",
                                pose_msg_bf.pose.position.x,
                                pose_msg_bf.pose.position.y,
                                yaw);
    }
    catch( const tf2::LookupException& e)
    {
      std::cout << e.what() << std::endl;
      std::cout << "navigateto position in frame_id " << pose_msg->header.frame_id
                << "is not supported in other base frame than basefootprint" << std::endl;
    }
    catch( const tf2::ExtrapolationException& e)
    {
      std::cout << "received an error on the time lookup" << std::endl;
    }
  }
}

} //publisher
} // naoqi
