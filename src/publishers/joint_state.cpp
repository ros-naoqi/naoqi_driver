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
#include <fstream>
#include <stdio.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <robot_state_publisher/robot_state_publisher.h>

#include "boost/filesystem.hpp"
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
  std::vector<float> al_joint_angles = p_motion_.call<std::vector<float> >("getAngles", "Body", true );
  // Leg might be an artificial joint in between all wheels of pepper
  std::vector<float> al_odometry_data = p_motion_.call<std::vector<float> >( "getPosition", "Torso", 1, true );
  const ros::Time& stamp = ros::Time::now();
  const float& odomX =  al_odometry_data[0];
  const float& odomY =  al_odometry_data[1];
  const float& odomZ =  al_odometry_data[2];
  const float& odomWX =  al_odometry_data[3];
  const float& odomWY =  al_odometry_data[4];
  const float& odomWZ =  al_odometry_data[5];
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw( odomWX, odomWY, odomWZ );

  /**
   * JOINT STATE PUBLISHER
   */
  msg_joint_states_.header.stamp = stamp;

  // STUPID CONVERTION FROM FLOAT TO DOUBLE ARRAY --> OPTIMIZE THAT!
  msg_joint_states_.position = std::vector<double>( al_joint_angles.begin(), al_joint_angles.end() );
  pub_joint_states_.publish( msg_joint_states_ );

  /**
   * ROBOT STATE PUBLISHER
   */
  // put joint states in tf broadcaster
  std::map< std::string, double > joint_state_map;
  // stupid version --> change this with std::transform c++11 ??!
  std::transform( msg_joint_states_.name.begin(), msg_joint_states_.name.end(), msg_joint_states_.position.begin(), std::inserter( joint_state_map, joint_state_map.end() ), std::make_pair<std::string, double>);

  static const std::string& jt_tf_prefix = "";
  rspPtr_->publishTransforms( joint_state_map, stamp, jt_tf_prefix );
  rspPtr_->publishFixedTransforms( jt_tf_prefix );


  /**
   * ODOMETRY FRAME
   */
  msg_tf_odom_.header.stamp = stamp+ros::Duration(0.5);

  msg_tf_odom_.transform.translation.x = odomX;
  msg_tf_odom_.transform.translation.y = odomY;
  msg_tf_odom_.transform.translation.z = odomZ;
  msg_tf_odom_.transform.rotation = odom_quat;

  tf_br_.sendTransform( msg_tf_odom_ );


  /**
   * ODOMETRY MESSAGE
   */
  // we can improve this with a velocity computation as well
  // with this we simply set the velocity as the diff between (odomX - msg_odom_.pose.pose.position.x)/dt
//  msg_nav_odom_.header.stamp = msg_tf_odom_.header.stamp;
//
//  msg_nav_odom_.pose.pose.position.x = odomX;
//  msg_nav_odom_.pose.pose.position.y = odomY;
//  msg_nav_odom_.pose.pose.position.z = 0;
//  msg_nav_odom_.pose.pose.orientation = odom_quat;
//  // fill in velocity components here
//  pub_odom_.publish(msg_nav_odom_);
}

void JointStatePublisher::reset( ros::NodeHandle& nh )
{
  pub_joint_states_ = nh.advertise<sensor_msgs::JointState>( topic_, 10 );
  pub_odom_ = nh.advertise<nav_msgs::Odometry>( "/odom", 10 );

  std::string robot_desc;
  // load urdf from param server (alternatively from file)
  if ( nh.hasParam("/robot_description") )
  {
    nh.getParam("/robot_description", robot_desc);
    std::cout << "load robot description from param server" << std::endl;
  }
  // load urdf from file
  else{

    // FIX THAT LATER: didn't find a better way to get a relative share folder
    char current_path[FILENAME_MAX];
    readlink("/proc/self/exe", current_path, sizeof(current_path));
    boost::filesystem::path root_path = boost::filesystem::complete( current_path ).parent_path().parent_path();
    const std::string& share_folder = root_path.string()+"/share/";
    std::cout << "share folder found in " << share_folder << std::endl;

    std::string path;
    if ( robot() == PEPPER)
    {
      path = "urdf/pepper_robot.urdf";
    }
    else if ( robot() == NAO )
    {
      path = "urdf/nao_robot.urdf";
    }
    else
    {
      is_initialized_ = false;
      return;
    }

    std::ifstream stream( (share_folder+path).c_str() );
    if (!stream)
    {
      std::cerr << "failed to load robot description in joint_state_publisher: " << path << std::endl;
      is_initialized_ = false;
      return;
    }
    robot_desc = std::string( (std::istreambuf_iterator<char>(stream)),
        std::istreambuf_iterator<char>());
    // upload to param server
    nh.setParam("/robot_description", robot_desc);
    std::cout << "load robot description from file" << std::endl;
  }

  if ( robot_desc.empty() )
  {
    is_initialized_ = false;
    std::cout << "error in loading robot description" << std::endl;
    return;
  }
  KDL::Tree tree;
  kdl_parser::treeFromString( robot_desc, tree );
  rspPtr_.reset( new robot_state_publisher::RobotStatePublisher(tree) );

  // pre-fill joint states message
  msg_joint_states_.name = p_motion_.call<std::vector<std::string> >("getBodyNames", "Body" );

  msg_tf_odom_.header.frame_id = "odom";
  msg_tf_odom_.child_frame_id = "base_link";

  // pre-fill odometry message
  msg_nav_odom_.header.frame_id = "odom";
  msg_nav_odom_.child_frame_id = "base_footprint";

  is_initialized_ = true;
}

bool JointStatePublisher::isSubscribed() const
{
  // assume JS and TF as essential, so publish always
  return true;
}


} //publisher
} // alros
