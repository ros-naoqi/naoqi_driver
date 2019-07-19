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
#include "moveto.hpp"

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

MovetoSubscriber::MovetoSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session,
                                    const boost::shared_ptr<tf2_ros::Buffer>& tf2_buffer):
  BaseSubscriber( name, topic, session ),
  p_motion_( session->service("ALMotion") ),
  tf2_buffer_( tf2_buffer )
{}

void MovetoSubscriber::reset( ros::NodeHandle& nh )
{
  sub_moveto_ = nh.subscribe( topic_, 10, &MovetoSubscriber::callback, this );
  is_initialized_ = true;
}

void MovetoSubscriber::callback( const naoqi_bridge_msgs::PoseStampedWithSpeedConstPtr &msg )
{

  std::vector<std::pair<std::string, float> > moveConfig;
  float speed;

  // On NAO, the moveTo config will not be applied
  speed = msg->speed_percentage * 0.45 + 0.1;
  moveConfig.push_back(std::make_pair("MaxVelXY", speed));

  if (msg->pose_stamped.header.frame_id == "odom") {
    geometry_msgs::PoseStamped pose_msg_bf;

    bool canTransform = tf2_buffer_->canTransform(
      "base_footprint",
      "odom",
      ros::Time(0),
      ros::Duration(2));

    if (!canTransform) {
      std::cout << "Cannot transform from "
                << "odom"
                << " to base_footprint"
                << std::endl;
      return;
    }

    try {
      geometry_msgs::PoseStamped pose_stamped = msg->pose_stamped;
      pose_stamped.header.frame_id = "odom";

      tf2_buffer_->transform(
        pose_stamped,
        pose_msg_bf,
        "base_footprint",
        ros::Time(0),
        "odom");

      double yaw = helpers::transform::getYaw(pose_msg_bf.pose);

      std::cout << "odom to move x: "
                <<  pose_msg_bf.pose.position.x
                << " y: "
                << pose_msg_bf.pose.position.y
                << " z: "
                << pose_msg_bf.pose.position.z
                << " yaw: "
                << yaw
                << std::endl;

      if (std::isnan(yaw)) {
        yaw = 0.0;
        std::cout << "Yaw is nan, changed to 0.0" << std::endl;
      }

      if (robot_ == robot::PEPPER)
        p_motion_.async<void>(
          "moveTo",
          pose_msg_bf.pose.position.x,
          pose_msg_bf.pose.position.y,
          yaw,
          qi::AnyValue::from(moveConfig));
      else
        p_motion_.async<void>(
          "moveTo",
          pose_msg_bf.pose.position.x,
          pose_msg_bf.pose.position.y,
          yaw);

    } catch( const tf2::LookupException& e) {
      std::cout << e.what() << std::endl;
      std::cout << "moveto position in frame_id "
                << msg->pose_stamped.header.frame_id
                << "is not supported in any other base frame than "
                   "basefootprint"
                << std::endl;
    } catch( const tf2::ExtrapolationException& e) {
      std::cout << "received an error on the time lookup" << std::endl;
    }
  }

  else if (msg->pose_stamped.header.frame_id == "base_footprint"){
    double yaw = helpers::transform::getYaw(msg->pose_stamped.pose);
    std::cout << "going to move x: "
              <<  msg->pose_stamped.pose.position.x
              << " y: " << msg->pose_stamped.pose.position.y
              << " z: " << msg->pose_stamped.pose.position.z
              << " yaw: " << yaw << std::endl;

    if (std::isnan(yaw)) {
      yaw = 0.0;
      std::cout << "Yaw is nan, changed to 0.0" << std::endl;
    }

    if (robot_ == robot::PEPPER)
      p_motion_.async<void>(
        "moveTo",
        msg->pose_stamped.pose.position.x,
        msg->pose_stamped.pose.position.y,
        yaw,
        qi::AnyValue::from(moveConfig));
    else
      p_motion_.async<void>(
        "moveTo",
        msg->pose_stamped.pose.position.x,
        msg->pose_stamped.pose.position.y,
        yaw);
  }

  else
    std::cout << "Cannot reach position expressed in the "
              << msg->pose_stamped.header.frame_id
              << " frame, enter a valid frame id in the pose's header"
                 " (base_footprint or odom)"
              << std::endl;

}

} //publisher
} // naoqi
