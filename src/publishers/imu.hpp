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

#ifndef IMU_PUBLISHER_HPP
#define IMU_PUBLISHER_HPP

/**
* ROS includes
*/
#include <sensor_msgs/Imu.h>

/**
* LOCAL includes
*/
#include "publisher_base.hpp"

namespace alros {

namespace publisher {

class ImuPublisher : public BasePublisher<ImuPublisher>
{
public:
  ImuPublisher(const std::string& name);

  virtual void publish( const sensor_msgs::Imu& imu_msg);

  virtual void reset( ros::NodeHandle& nh );

};

}

}

#endif // IMU_PUBLISHER_HPP
