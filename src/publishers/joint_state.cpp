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

#include <iostream>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <tf/transform_datatypes.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include "joint_state.hpp"

namespace alros
{
namespace publisher
{

JointStatePublisher::JointStatePublisher( const std::string& name, const std::string& topic, float frequency, qi::SessionPtr& session ):
  BasePublisher( name, topic, frequency, session ),
  p_motion_( session->service("ALMotion") )
{}

void JointStatePublisher::publish()
{
  // get joint state values
  std::vector<float> al_joint_angles = p_motion_.call<std::vector<float> >("getAngles", "JointActuators", true );
  msg_joint_states_.header.stamp = ros::Time::now();
  // STUPID CONVERTION FROM FLOAT TO DOUBLE ARRAY --> OPTIMIZE THAT!
  msg_joint_states_.position = std::vector<double>( al_joint_angles.begin(), al_joint_angles.end() );
  pub_joint_states_.publish( msg_joint_states_ );

  // Leg might be an artificial joint in between all wheels of pepper
  std::vector<float> al_odometry_data = p_motion_.call<std::vector<float> >( "getRobotPosition", true );

  msg_nav_odom_.header.stamp = ros::Time::now();
  msg_tf_odom_.header.stamp = ros::Time::now();


  const float& odomX =  al_odometry_data[0];
  const float& odomY =  al_odometry_data[1];
  const float& odomTheta =  al_odometry_data[2];

  // we can improve this with a velocity computation as well
  // with this we simply set the velocity as the diff between (odomX - msg_odom_.pose.pose.position.x)/dt
  msg_nav_odom_.pose.pose.position.x = odomX;
  msg_nav_odom_.pose.pose.position.y = odomY;
  msg_nav_odom_.pose.pose.position.z = 0;
  msg_nav_odom_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( 0, 0, odomTheta );
  // fill in velocity components here
  pub_odom_.publish(msg_nav_odom_);



  // put joint states in tf broadcaster
  std::map< std::string, double > joint_state_map;
  // stupid version --> change this with std::transform c++11 ??!
  std::transform( msg_joint_states_.name.begin(), msg_joint_states_.name.end(), msg_joint_states_.position.begin(), std::inserter( joint_state_map, joint_state_map.end() ), std::make_pair<std::string, double>);

  static const std::string& jt_tf_prefix = "";
  rspPtr_->publishTransforms( joint_state_map, msg_joint_states_.header.stamp, jt_tf_prefix );
  rspPtr_->publishFixedTransforms( jt_tf_prefix );

  //const geometry_msgs::Point& p = msg_nav_odom_.pose.pose.position;
  //const geometry_msgs::Quaternion& q = msg_nav_odom_.pose.pose.orientation;
  //msg_tf_odom_.transform.translation.x = odomX;
  //msg_tf_odom_.transform.translation.y = odomY;
  //msg_tf_odom_.transform.translation.z = 0;
  //msg_tf_odom_.transform.rotation = msg_nav_odom_.pose.pose.orientation;

  // DEBUG
  static tf::Transform transform;
  transform.setOrigin( tf::Vector3(odomX, odomY, 0) );
  tf::Quaternion q;
  q.setRPY(0, 0, odomTheta);
  transform.setRotation(q);

  tf_br_.sendTransform( tf::StampedTransform(transform, msg_tf_odom_.header.stamp, "base_footprint", "odom" ) );
  //tf_br_.sendTransform( msg_tf_odom_ );
}

void JointStatePublisher::reset( ros::NodeHandle& nh )
{
  pub_joint_states_ = nh.advertise<sensor_msgs::JointState>( topic_, 10 );
  pub_odom_ = nh.advertise<nav_msgs::Odometry>( "/odom", 10 );

  // load urdf from param server (alternatively from file)
  if ( nh.hasParam("/robot_description") )
  {
    KDL::Tree tree;
    std::string robot_desc;
    nh.getParam("/robot_description", robot_desc);
    kdl_parser::treeFromString( robot_desc, tree );
    rspPtr_.reset( new robot_state_publisher::RobotStatePublisher(tree) );
  }
  else{
    std::cerr << "failed to load robot description in joint_state_publisher" << std::endl;
    is_initialized_ = false;
    return;
  }

  // pre-fill joint states message
  msg_joint_states_.name = p_motion_.call<std::vector<std::string> >("getBodyNames", "JointActuators" );

  // pre-fill odometry message
  msg_nav_odom_.header.frame_id = "odom";
  msg_nav_odom_.child_frame_id = "base_footprint";

  msg_tf_odom_.header.frame_id = "odom";
  msg_tf_odom_.child_frame_id = "base_footprint";

  is_initialized_ = true;
}

bool JointStatePublisher::isSubscribed() const
{
  // assume JS and TF as essential, so publish always
  return true;
}


} //publisher
} // alros
