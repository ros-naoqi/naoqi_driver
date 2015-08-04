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

#ifndef IMU_CONVERTER_HPP
#define IMU_CONVERTER_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"
#include <naoqi_driver/message_actions.h>

/*
* ROS includes
*/
#include <sensor_msgs/Imu.h>

namespace naoqi
{
namespace converter {

namespace IMU{

enum Location{
    TORSO,
    BASE
};

}

class ImuConverter : public BaseConverter<ImuConverter>
{

  typedef boost::function<void(sensor_msgs::Imu&) > Callback_t;

public:
  ImuConverter(const std::string& name, const IMU::Location& location, const float& frequency, const qi::SessionPtr& session);

  ~ImuConverter();

  virtual void reset();

  void registerCallback( const message_actions::MessageAction action, Callback_t cb );

  virtual void callAll(const std::vector<message_actions::MessageAction>& actions);

private:
  sensor_msgs::Imu msg_imu_;
  qi::AnyObject p_memory_;
  std::vector<std::string> data_names_list_;

  /** Registered Callbacks **/
  std::map<message_actions::MessageAction, Callback_t> callbacks_;
};

}

}

#endif // IMU_CONVERTER_HPP
