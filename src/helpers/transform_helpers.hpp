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


#ifndef TRANSFORM_HELPERS_HPP
#define TRANSFORM_HELPERS_HPP

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace naoqi
{
namespace helpers
{
namespace transform
{

inline double getYaw(const geometry_msgs::Pose& pose)
{
  double yaw, _pitch, _roll;
  tf2::Matrix3x3(tf2::Quaternion(pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w)).getEulerYPR(yaw, _pitch, _roll);
  return yaw;
}

inline double getYaw( const geometry_msgs::Transform& pose)
{
  double yaw, _pitch, _roll;
  tf2::Matrix3x3(tf2::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)).getEulerYPR(yaw, _pitch, _roll);
  return yaw;
}

} //transform
} //helpers
} // naoqi

#endif
