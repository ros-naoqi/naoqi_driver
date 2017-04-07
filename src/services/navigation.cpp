/*
 * Copyright 2017 SoftBank Robotics Europe
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

#include "navigation.hpp"
#include "../helpers/driver_helpers.hpp"
#include "../helpers/transform_helpers.hpp"

namespace naoqi
{
namespace service
{

void NavigationEmptyService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &NavigationEmptyService::callback, this);
}

bool NavigationEmptyService::callback(std_srvs::EmptyRequest& req,
                                      std_srvs::EmptyResponse& resp)
{
  p_navigation_.call<void>(func_);
  return true;
}

void NavigateToService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &NavigateToService::callback, this);
}

bool NavigateToService::callback(nao_interaction_msgs::GoToPoseRequest& req,
                                 nao_interaction_msgs::GoToPoseResponse& resp)
{
    if ( req.pose.header.frame_id == "base_footprint" )
    {
      double yaw = helpers::transform::getYaw(req.pose.pose);

      std::cout << "going to navigate x: " << req.pose.pose.position.x
                << " y: " << req.pose.pose.position.y
                << " z: " << req.pose.pose.position.z
                << " yaw: " << yaw << std::endl;

      p_navigation_.async<void>(func_,
                               req.pose.pose.position.x,
                               req.pose.pose.position.y,
                               yaw);
    }
    else if (req.pose.header.frame_id == "map")
    {
      double yaw = helpers::transform::getYaw(req.pose.pose);
      std::cout << "map to navigate x: " << req.pose.pose.position.x
                << " y: " << req.pose.pose.position.y
                << " z: " << req.pose.pose.position.z
                << " yaw: " << yaw << std::endl;

      std::vector<float> pose(3);
      pose[0] = req.pose.pose.position.x;
      pose[1] = req.pose.pose.position.y;
      pose[2] = yaw;
      p_navigation_.async<void>("navigateToInMap", pose);
    }
    else
    {
      std::string frame = "base_footprint";
      geometry_msgs::PoseStamped pose_msg_bf;
      bool canTransform = tf2_buffer_->canTransform(frame,
                                                    req.pose.header.frame_id,
                                                    ros::Time(0),
                                                    ros::Duration(2) );
      if (!canTransform)
      {
        std::cout << "Cannot transform from " << req.pose.header.frame_id
                  << " to " << frame << std::endl;
        return false;
      }
      try
      {
        tf2_buffer_->transform( req.pose,
                                pose_msg_bf,
                                frame,
                                ros::Time(0),
                                req.pose.header.frame_id );

        double yaw = helpers::transform::getYaw(pose_msg_bf.pose);
        std::cout << "odom to navigate x: "
                  << pose_msg_bf.pose.position.x
                  << " y: " << pose_msg_bf.pose.position.y
                  << " z: " << pose_msg_bf.pose.position.z
                  << " yaw: " << yaw << std::endl;

        p_navigation_.async<void>(func_,
                                 pose_msg_bf.pose.position.x,
                                 pose_msg_bf.pose.position.y,
                                 yaw);
      }
      catch( const tf2::LookupException& e)
      {
        std::cout << e.what() << std::endl;
        std::cout << "navigateto position in frame_id " << req.pose.header.frame_id
                  << " is not supported; use the " << frame << " frame" << std::endl;
      }
      catch( const tf2::ExtrapolationException& e)
      {
        std::cout << "received an error on the time lookup" << std::endl;
      }
    }
    return true;
}

void NavigateToInMapService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &NavigateToInMapService::callback, this);
}

bool NavigateToInMapService::callback(nao_interaction_msgs::GoToPoseRequest& req,
                                      nao_interaction_msgs::GoToPoseResponse& resp)
{
  if ( req.pose.header.frame_id == "map" )
  {
    double yaw = helpers::transform::getYaw(req.pose.pose);
    std::cout << "map to navigate x: " << req.pose.pose.position.x
              << " y: " << req.pose.pose.position.y
              << " z: " << req.pose.pose.position.z
              << " yaw: " << yaw << std::endl;

    pose[0] = req.pose.pose.position.x;
    pose[1] = req.pose.pose.position.y;
    pose[2] = yaw;
    p_navigation_.async<void>(func_, pose);
  }
  else
  {
    std::string frame("map");
    geometry_msgs::PoseStamped pose_msg_bf;
    bool canTransform = tf2_buffer_->canTransform(frame,
                                                  req.pose.header.frame_id,
                                                  ros::Time(0),
                                                  ros::Duration(2));

    if (!canTransform) {
      std::cout << "Cannot transform from " << req.pose.header.frame_id
                << " to " << frame << " frame" << std::endl;
      return false;
    }

    try
    {
      tf2_buffer_->transform( req.pose, pose_msg_bf,
                              frame,
                              ros::Time(0),
                              req.pose.header.frame_id );

      double yaw = helpers::transform::getYaw(pose_msg_bf.pose);
      std::cout << "going to navigate x: " << pose_msg_bf.pose.position.x
                << " y: " << pose_msg_bf.pose.position.y
                << " z: " << pose_msg_bf.pose.position.z
                << " yaw: " << yaw << std::endl;

      pose[0] = pose_msg_bf.pose.position.x;
      pose[1] = pose_msg_bf.pose.position.y;
      pose[2] = yaw;
      p_navigation_.async<void>(func_, pose);
    }
    catch( const tf2::LookupException& e)
    {
      std::cout << e.what() << std::endl;
      std::cout << "navigateto position in frame_id " << req.pose.header.frame_id
                << "is not supported; use the " << frame << " frame" << std::endl;
    }
    catch( const tf2::ExtrapolationException& e)
    {
      std::cout << "received an error on the time lookup" << std::endl;
    }
  }
  return true;
}

void ExploreService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &ExploreService::callback, this);
}

bool ExploreService::callback(nao_interaction_msgs::ExploreRequest& req,
                              nao_interaction_msgs::ExploreResponse& resp)
{
  //explore
  bool res(true);
  resp.path_to_map = "";

  ROS_INFO_STREAM("Starting exploration in " << req.radius << " meters");
  int error_code = p_navigation_.call<int>(func_, req.radius);
  if (error_code != 0)
  {
    ROS_ERROR_STREAM(func_ << " failed.");
    return false;
  }
  ROS_INFO("Finished exploration");

  //stop exploration if it did not stop yet
  p_navigation_.call<void>("stopExploration");

  //save exploration
  resp.path_to_map = p_navigation_.call<std::string>("saveExploration");

  //stop localization
  p_navigation_.call<void>("stopLocalization");

  //load exploration
  res = p_navigation_.call<bool>("loadExploration", resp.path_to_map);

  //relocalize
  if (!res)
  {
    ROS_ERROR("The explored map cannot be loaded.");
    return res;
  }

  ROS_INFO("Now, you need to move your robot to zero in the map and \
    relocalize with RelocalizeInMapService with (0,0,0)");

  return res;
}

void LoadExplorationService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &LoadExplorationService::callback, this);
}

bool LoadExplorationService::callback(nao_interaction_msgs::LoadExplorationRequest& req,
                                      nao_interaction_msgs::LoadExplorationResponse& resp)
{
  //stop localization
  p_navigation_.call<void>("stopLocalization");

  //load exploration
  resp.result = p_navigation_.call<bool>(func_, req.path_to_map);

  //relocalize
  if (!resp.result)
    ROS_ERROR("The explored map cannot be loaded.");

  return resp.result;
}

void RelocalizeInMapService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &RelocalizeInMapService::callback, this);
}

bool RelocalizeInMapService::callback(nao_interaction_msgs::RelocalizeInMapRequest& req,
                                      nao_interaction_msgs::RelocalizeInMapResponse& resp)
{
  resp.result = true;

  //stop localization
  p_navigation_.call<void>("stopLocalization");

  if ( req.pose.header.frame_id == "map" )
  {
    //relocalize
    pose[0] = req.pose.pose.position.x;
    pose[1] = req.pose.pose.position.y;
    pose[2] = helpers::transform::getYaw(req.pose.pose);
    p_navigation_.call<void>(func_, pose);
  }
  else
  {
    std::string frame("map");
    geometry_msgs::PoseStamped pose_msg_bf;
    bool canTransform = tf2_buffer_->canTransform(frame,
                                                  req.pose.header.frame_id,
                                                  ros::Time(0),
                                                  ros::Duration(2));

    if (!canTransform) {
      std::cout << "Cannot transform from " << req.pose.header.frame_id
                << " to " << frame << std::endl;
      resp.result = false;
      return false;
    }
    try
    {
      tf2_buffer_->transform( req.pose, pose_msg_bf,
                              frame,
                              ros::Time(0),
                              req.pose.header.frame_id );

      double yaw = helpers::transform::getYaw(pose_msg_bf.pose);
      std::cout << "going to navigate x: " << pose_msg_bf.pose.position.x
                << " y: " << pose_msg_bf.pose.position.y
                << " z: " << pose_msg_bf.pose.position.z
                << " yaw: " << yaw << std::endl;

      pose[0] = pose_msg_bf.pose.position.x;
      pose[1] = pose_msg_bf.pose.position.y;
      pose[2] = yaw;
      p_navigation_.async<void>(func_, pose);
    }
    catch( const tf2::LookupException& e)
    {
      std::cout << e.what() << std::endl;
      std::cout << "navigateto position in frame_id " << req.pose.header.frame_id
                << "is not supported; use the " << frame << " frame" << std::endl;
      resp.result = false;
    }
    catch( const tf2::ExtrapolationException& e)
    {
      std::cout << "received an error on the time lookup" << std::endl;
      resp.result = false;
    }
  }

  //start localization
  p_navigation_.call<void>("startLocalization");

  return resp.result;
}

}
}
