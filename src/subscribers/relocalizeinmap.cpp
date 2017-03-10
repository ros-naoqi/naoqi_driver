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
#include "relocalizeinmap.hpp"

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

RelocalizeSubscriber::RelocalizeSubscriber( const std::string& name,
                                            const std::string& topic,
                                            const qi::SessionPtr& session,
                                            const boost::shared_ptr<tf2_ros::Buffer>& tf2_buffer):
  BaseSubscriber( name, topic, session ),
  p_navigation_( session->service("ALNavigation") ),
  tf2_buffer_( tf2_buffer )
{
  pose.reserve(3);
  pose.resize(3);
}

void RelocalizeSubscriber::reset( ros::NodeHandle& nh )
{
  sub_relocalize_ = nh.subscribe( topic_, 10, &RelocalizeSubscriber::callback, this );
  is_initialized_ = true;
}

void RelocalizeSubscriber::callback( const geometry_msgs::PoseWithCovarianceStamped& pose_msg )
{
  if ( pose_msg.header.frame_id == "map" )
  {
    double yaw = helpers::transform::getYaw(pose_msg.pose.pose);

    std::cout << "going to " << name_
              << " x: " <<  pose_msg.pose.pose.position.x
              << " y: " << pose_msg.pose.pose.position.y
              << " z: " << pose_msg.pose.pose.position.z
              << " yaw: " << yaw << std::endl;

    pose[0] = pose_msg.pose.pose.position.x;
    pose[1] = pose_msg.pose.pose.position.y;
    pose[2] = yaw;
    p_navigation_.async<void>(name_, pose);
  }
}

} //publisher
} // naoqi
