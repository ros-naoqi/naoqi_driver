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
#include "imu.hpp"
#include "../tools/from_any_value.hpp"

/*
* ROS includes
*/
//#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace naoqi
{
namespace converter {


  ImuConverter::ImuConverter(const std::string& name, const IMU::Location& location,  const float& frequency, const qi::SessionPtr& session):
    BaseConverter(name, frequency, session),
    p_memory_(session->service("ALMemory"))
  {
    if(location == IMU::TORSO){
      msg_imu_.header.frame_id = "base_link";
      data_names_list_.push_back("DCM/Time");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");
    }
    else if(location == IMU::BASE){
      msg_imu_.header.frame_id = "base_footprint";
      data_names_list_.push_back("DCM/Time");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AngleX/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AngleY/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AngleZ/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/GyroscopeX/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/GyroscopeY/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/GyroscopeZ/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AccelerometerX/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AccelerometerY/Sensor/Value");
      data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AccelerometerZ/Sensor/Value");
    }
  }

  ImuConverter::~ImuConverter()
  {

  }

  void ImuConverter::reset()
  {

  }

  void ImuConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
  {
    callbacks_[action] = cb;
  }

  void ImuConverter::callAll(const std::vector<message_actions::MessageAction>& actions)
  {
    // Get inertial data
    std::vector<float> memData;
    try {
        qi::AnyValue anyvalues = p_memory_.call<qi::AnyValue>("getListData", data_names_list_);
        tools::fromAnyValueToFloatVector(anyvalues, memData);
    } catch (const std::exception& e) {
      std::cerr << "Exception caught in ImuConverter: " << e.what() << std::endl;
      return;
    }
    // angle (X,Y,Z) = memData(1,2,3);
    // gyro  (X,Y,Z) = memData(4,5,6);
    // acc   (X,Y,Z) = memData(7,8,9);

    const ros::Time& stamp = ros::Time::now();
    msg_imu_.header.stamp = stamp;

    tf2::Quaternion tf_quat;
    tf_quat.setRPY( memData[1], memData[2], memData[3] );
    msg_imu_.orientation = tf2::toMsg( tf_quat );

    //msg_imu_.orientation = tf::createQuaternionMsgFromRollPitchYaw(
    //                            memData[1],
    //                            memData[2],
    //                            memData[3]);

    msg_imu_.angular_velocity.x = memData[4];
    msg_imu_.angular_velocity.y = memData[5];
    msg_imu_.angular_velocity.z = memData[6];

    msg_imu_.linear_acceleration.x = memData[7];
    msg_imu_.linear_acceleration.y = memData[8];
    msg_imu_.linear_acceleration.z = memData[9];

    // Covariances unknown
    msg_imu_.orientation_covariance[0] = -1;
    msg_imu_.angular_velocity_covariance[0] = -1;
    msg_imu_.linear_acceleration_covariance[0] = -1;


    for_each( message_actions::MessageAction action, actions )
    {
      callbacks_[action]( msg_imu_ );
    }
  }

}

}
