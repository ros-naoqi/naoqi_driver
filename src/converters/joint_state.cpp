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

#include "boost/filesystem.hpp"
#include "joint_state.hpp"

#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace alros
{
namespace converter
{

JointStateConverter::JointStateConverter( const std::string& name, float frequency, BufferPtr tf2_buffer, qi::SessionPtr& session, ros::NodeHandle& nh ):
  BaseConverter( name, frequency, session ),
  p_motion_( session->service("ALMotion") ),
  tf2_buffer_(tf2_buffer)
{
  // load urdf from param server (alternatively from file)
  if ( nh.hasParam("/robot_description") )
  {
    nh.getParam("/robot_description", robot_desc_);
    std::cout << "load robot description from param server" << std::endl;
  }
  // load urdf from file
  else{

    // FIX THAT LATER: didn't find a better way to get a relative share folder
    char current_path[FILENAME_MAX];
    readlink("/proc/self/exe", current_path, sizeof(current_path));
    boost::filesystem::path root_path = boost::filesystem::complete( current_path );
    std::cout << "executable path found in " << root_path.string() << std::endl;
    root_path = root_path.parent_path().parent_path();
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
      std::cerr << " could not load urdf file from disk " << std::endl;
      return;
    }

    std::ifstream stream( (share_folder+path).c_str() );
    if (!stream)
    {
      std::cerr << "failed to load robot description in joint_state_publisher: " << path << std::endl;
      return;
    }
    robot_desc_ = std::string( (std::istreambuf_iterator<char>(stream)),
        std::istreambuf_iterator<char>());
    // upload to param server
    nh.setParam("/robot_description", robot_desc_);
    std::cout << "load robot description from file" << std::endl;
  }
}

JointStateConverter::~JointStateConverter()
{
  // clear all sessions
}

void JointStateConverter::reset()
{
  if ( robot_desc_.empty() )
  {
    std::cout << "error in loading robot description" << std::endl;
    return;
  }
  KDL::Tree tree;
  kdl_parser::treeFromString( robot_desc_, tree );

  addChildren( tree.getRootSegment() );

  // pre-fill joint states message
  msg_joint_states_.name = p_motion_.call<std::vector<std::string> >("getBodyNames", "Body" );
}

void JointStateConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void JointStateConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  // get joint state values
  std::vector<float> al_joint_angles = p_motion_.call<std::vector<float> >("getAngles", "Body", true );
  const ros::Time& stamp = ros::Time::now();

  /**
   * JOINT STATE PUBLISHER
   */
  msg_joint_states_.header.stamp = stamp;

  // STUPID CONVERTION FROM FLOAT TO DOUBLE ARRAY --> OPTIMIZE THAT!
  msg_joint_states_.position = std::vector<double>( al_joint_angles.begin(), al_joint_angles.end() );

  /**
   * ROBOT STATE PUBLISHER
   */
  // put joint states in tf broadcaster
  std::map< std::string, double > joint_state_map;
  // stupid version --> change this with std::transform c++11 ??!
  std::transform( msg_joint_states_.name.begin(), msg_joint_states_.name.end(), msg_joint_states_.position.begin(), std::inserter( joint_state_map, joint_state_map.end() ), std::make_pair<std::string, double>);

  // reset the transforms we want to use at this time
  tf_transforms_.clear();
  static const std::string& jt_tf_prefix = "";
  setTransforms(joint_state_map, stamp, jt_tf_prefix);
  setFixedTransforms(jt_tf_prefix, stamp);

  // If nobody uses that buffer, do not fill it next time
  if (( tf2_buffer_ ) && ( tf2_buffer_.use_count() == 1 ))
  {
    tf2_buffer_.reset();
  }

  for_each( message_actions::MessageAction action, actions )
  {
    callbacks_[action]( msg_joint_states_, tf_transforms_ );
  }
}


// Copied from robot state publisher
void JointStateConverter::setTransforms(const std::map<std::string, double>& joint_positions, const ros::Time& time, const std::string& tf_prefix)
{
  geometry_msgs::TransformStamped tf_transform;
  tf_transform.header.stamp = time;

  // loop over all joints
  for (std::map<std::string, double>::const_iterator jnt=joint_positions.begin(); jnt != joint_positions.end(); jnt++){
    std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator seg = segments_.find(jnt->first);
    if (seg != segments_.end()){
      seg->second.segment.pose(jnt->second).M.GetQuaternion(tf_transform.transform.rotation.x,
                                                            tf_transform.transform.rotation.y,
                                                            tf_transform.transform.rotation.z,
                                                            tf_transform.transform.rotation.w);
      tf_transform.transform.translation.x = seg->second.segment.pose(jnt->second).p.x();
      tf_transform.transform.translation.y = seg->second.segment.pose(jnt->second).p.y();
      tf_transform.transform.translation.z = seg->second.segment.pose(jnt->second).p.z();

      tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
      tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);

      tf_transforms_.push_back(tf_transform);

      if (tf2_buffer_)
        tf2_buffer_->setTransform(tf_transform, "alrosconverter", false);
    }
  }
  //tf_broadcaster_.sendTransform(tf_transforms);
}

// Copied from robot state publisher
void JointStateConverter::setFixedTransforms(const std::string& tf_prefix, const ros::Time& time)
{
  geometry_msgs::TransformStamped tf_transform;
  tf_transform.header.stamp = time+ros::Duration(0.5);  // future publish by 0.5 seconds

  // loop over all fixed segments
  for (std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator seg=segments_fixed_.begin(); seg != segments_fixed_.end(); seg++){
    seg->second.segment.pose(0).M.GetQuaternion(tf_transform.transform.rotation.x,
                                                tf_transform.transform.rotation.y,
                                                tf_transform.transform.rotation.z,
                                                tf_transform.transform.rotation.w);
    tf_transform.transform.translation.x = seg->second.segment.pose(0).p.x();
    tf_transform.transform.translation.y = seg->second.segment.pose(0).p.y();
    tf_transform.transform.translation.z = seg->second.segment.pose(0).p.z();

    tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
    tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);

    tf_transforms_.push_back(tf_transform);

    if (tf2_buffer_)
      tf2_buffer_->setTransform(tf_transform, "alrosconverter", true);
  }
  //tf_broadcaster_.sendTransform(tf_transforms);
}

void JointStateConverter::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
  for (unsigned int i=0; i<children.size(); i++){
    const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
    robot_state_publisher::SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None){
      segments_fixed_.insert(std::make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    else{
      segments_.insert(std::make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    addChildren(children[i]);
  }
}

} //publisher
} // alros
