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
#include <vector>

#include <boost/make_shared.hpp>

#include <ros/ros.h>

#include <qi/anyobject.hpp>

#include <naoqi_driver/recorder/globalrecorder.hpp>
#include <naoqi_driver/message_actions.h>
#include "../tools/from_any_value.hpp"
#include <typeinfo>

#include "people.hpp"

namespace naoqi
{

template<class T>
PeopleEventRegister<T>::PeopleEventRegister()
{
}

template<class T>
PeopleEventRegister<T>::PeopleEventRegister( const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session )
  : serviceId(0),
    p_memory_( session->service("ALMemory")),
    p_people_( session->service("ALPeoplePerception") ),
    p_gaze_( session->service("ALGazeAnalysis") ),
    p_face_( session->service("ALFaceCharacteristics") ),
    p_waving_( session->service("ALWavingDetection") ),
    session_(session),
    isStarted_(false),
    isPublishing_(false),
    isRecording_(false),
    isDumping_(false),
    prefix("PeoplePerception/Person/")
{
  memory_keys.push_back("/IsFaceDetected");
  memory_keys.push_back("/IsWaving");
  memory_keys.push_back("/GazeDirection");
  memory_keys.push_back("/HeadAngles");
  memory_keys.push_back("/IsLookingAtRobot");
  memory_keys.push_back("/LookingAtRobotScore");
  memory_keys.push_back("/AgeProperties");
  memory_keys.push_back("/GenderProperties");
  memory_keys.push_back("/SmileProperties");
  memory_keys.push_back("/ExpressionProperties");
  
  publisher_ = boost::make_shared<publisher::BasicPublisher<T> >( name );
  //recorder_ = boost::make_shared<recorder::BasicEventRecorder<T> >( name );
  converter_ = boost::make_shared<converter::PeopleEventConverter<T> >( name, frequency, session );

  converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<T>::publish, publisher_, _1) );
  //converter_->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicEventRecorder<T>::write, recorder_, _1) );
  //converter_->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicEventRecorder<T>::bufferize, recorder_, _1) );

  keys_.resize(keys.size());
  size_t i = 0;
  for(std::vector<std::string>::const_iterator it = keys.begin(); it != keys.end(); ++it, ++i)
    keys_[i] = *it;

  name_ = name;
}

template<class T>
PeopleEventRegister<T>::~PeopleEventRegister()
{
  stopProcess();
}

template<class T>
void PeopleEventRegister<T>::resetPublisher(ros::NodeHandle& nh)
{
  publisher_->reset(nh);
}

template<class T>
void PeopleEventRegister<T>::resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
{
  //recorder_->reset(gr, converter_->frequency());
}

template<class T>
void PeopleEventRegister<T>::startProcess()
{
  boost::mutex::scoped_lock start_lock(mutex_);
  if (!isStarted_)
  {
    if(!serviceId)
    {
      //std::string serviceName = std::string("ROS-Driver-") + typeid(T).name();
      std::string serviceName = std::string("ROS-Driver-") + keys_[0];
      serviceId = session_->registerService(serviceName, this->shared_from_this());
      for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it) {
        std::cerr << *it << std::endl;
        p_memory_.call<void>("subscribeToEvent",it->c_str(), serviceName, "peopleCallback");
      }
      if(keys_[0].compare("PeoplePerception/PeopleDetected")==0) {
          std::cout<<serviceName<<" -> People : Start"<<std::endl;
          p_people_.call<void>("subscribe", "ROS");
          std::cout<<serviceName<<" -> Gaze : Start"<<std::endl;
          p_gaze_.call<void>("subscribe", "ROS");
          std::cout<<serviceName<<" -> Face : Start"<<std::endl;
          p_face_.call<void>("subscribe", "ROS");
          std::cout<<serviceName<<" -> Waving : Start"<<std::endl;
          p_waving_.call<void>("subscribe", "ROS");
      }
      std::cout << serviceName << " : Start" << std::endl;
    }
    isStarted_ = true;
  }
}

template<class T>
void PeopleEventRegister<T>::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(mutex_);
  if (isStarted_)
  {
    //std::string serviceName = std::string("ROS-Driver-") + typeid(T).name();
    std::string serviceName = std::string("ROS-Driver-") + keys_[0];
    if(serviceId){
      if(keys_[0].compare("PeoplePerception/PeopleDetected")==0) {
          std::cout<<serviceName<<" -> People : Stop"<<std::endl;
          p_people_.call<void>("unsubscribe", "ROS");
          std::cout<<serviceName<<" -> Gaze : Stop"<<std::endl;
          p_gaze_.call<void>("unsubscribe", "ROS");
          std::cout<<serviceName<<" -> Face : Stop"<<std::endl;
          p_face_.call<void>("unsubscribe", "ROS");
          std::cout<<serviceName<<" -> Waving : Stop"<<std::endl;
          p_waving_.call<void>("unsubscribe", "ROS");
      }
      for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it) {
        p_memory_.call<void>("unsubscribeToEvent",it->c_str(), serviceName);
      }
      session_->unregisterService(serviceId);
      serviceId = 0;
    }
    std::cout << serviceName << " : Stop" << std::endl;
    isStarted_ = false;
  }
}

template<class T>
void PeopleEventRegister<T>::writeDump(const ros::Time& time)
{
  if (isStarted_)
  {
    //recorder_->writeDump(time);
  }
}

template<class T>
void PeopleEventRegister<T>::setBufferDuration(float duration)
{
  //recorder_->setBufferDuration(duration);
}

template<class T>
void PeopleEventRegister<T>::isRecording(bool state)
{
  boost::mutex::scoped_lock rec_lock(mutex_);
  isRecording_ = state;
}

template<class T>
void PeopleEventRegister<T>::isPublishing(bool state)
{
  boost::mutex::scoped_lock pub_lock(mutex_);
  isPublishing_ = state;
}

template<class T>
void PeopleEventRegister<T>::isDumping(bool state)
{
  boost::mutex::scoped_lock dump_lock(mutex_);
  isDumping_ = state;
}

template<class T>
void PeopleEventRegister<T>::registerCallback()
{
}

template<class T>
void PeopleEventRegister<T>::unregisterCallback()
{
}

template<class T>
void PeopleEventRegister<T>::peopleCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message)
{
  T msg = T();

  peopleCallbackMessage(key, value, msg);

  std::vector<message_actions::MessageAction> actions;
  boost::mutex::scoped_lock callback_lock(mutex_);

  if (isStarted_) {
    // CHECK FOR PUBLISH
    if ( isPublishing_ && publisher_->isSubscribed() )
    {
      actions.push_back(message_actions::PUBLISH);
    }
    // CHECK FOR RECORD
    if ( isRecording_ )
    {
      //actions.push_back(message_actions::RECORD);
    }
    if ( !isDumping_ )
    {
      //actions.push_back(message_actions::LOG);
    }
    if (actions.size() >0)
    {
      converter_->callAll( actions, msg );
    }
  }
}

template<class T>
void PeopleEventRegister<T>::peopleCallbackMessage(std::string &key, qi::AnyValue &value, nao_interaction_msgs::FaceDetectedArray &msg)
{
  tools::NaoqiFaceDetected faces;
  try {
    faces = tools::fromAnyValueToNaoqiFaceDetected(value);
  }
  catch(std::runtime_error& e)
  {
    ROS_DEBUG_STREAM("Cannot retrieve facedetect: " << e.what());
    return;
  }
  if ( faces.face_info.size() == 0 ) return;
  
  msg.header.frame_id = "CameraTop_optical_frame";
  msg.header.stamp = ros::Time::now();

  for(int i = 0; i < faces.face_info.size(); i++) {
    nao_interaction_msgs::FaceDetected face;
    face.header = msg.header;
    face.face_id.data = faces.face_info[i].extra_info[0].face_id;
    face.score_reco.data = faces.face_info[i].extra_info[0].score_reco;
    face.face_label.data = faces.face_info[i].extra_info[0].face_label;

    face.shape_alpha.data = faces.face_info[i].shape_info.alpha;
    face.shape_beta.data  = faces.face_info[i].shape_info.beta;
    face.shape_sizeX.data = faces.face_info[i].shape_info.sizeX;
    face.shape_sizeY.data = faces.face_info[i].shape_info.sizeY;

    face.right_eye_eyeCenter_x.data = faces.face_info[i].extra_info[0].right_eye_points.eye_center_x;
    face.right_eye_eyeCenter_y.data = faces.face_info[i].extra_info[0].right_eye_points.eye_center_y;
    face.right_eye_noseSideLimit_x.data = faces.face_info[i].extra_info[0].right_eye_points.nose_side_limit_x;
    face.right_eye_noseSideLimit_y.data = faces.face_info[i].extra_info[0].right_eye_points.nose_side_limit_y;
    face.right_eye_earSideLimit_x.data = faces.face_info[i].extra_info[0].right_eye_points.ear_side_limit_x;
    face.right_eye_earSideLimit_y.data = faces.face_info[i].extra_info[0].right_eye_points.ear_side_limit_y;

    face.left_eye_eyeCenter_x.data = faces.face_info[i].extra_info[0].left_eye_points.eye_center_x;
    face.left_eye_eyeCenter_y.data = faces.face_info[i].extra_info[0].left_eye_points.eye_center_y;
    face.left_eye_noseSideLimit_x.data = faces.face_info[i].extra_info[0].left_eye_points.nose_side_limit_x;
    face.left_eye_noseSideLimit_y.data = faces.face_info[i].extra_info[0].left_eye_points.nose_side_limit_y;
    face.left_eye_earSideLimit_x.data = faces.face_info[i].extra_info[0].left_eye_points.ear_side_limit_x;
    face.left_eye_earSideLimit_y.data = faces.face_info[i].extra_info[0].left_eye_points.ear_side_limit_y;

    face.nose_bottomCenterLimit_x.data = faces.face_info[i].extra_info[0].nose_points.bottom_center_limit_x;
    face.nose_bottomCenterLimit_y.data = faces.face_info[i].extra_info[0].nose_points.bottom_center_limit_y;
    face.nose_bottomLeftLimit_x.data = faces.face_info[i].extra_info[0].nose_points.bottom_left_limit_x;
    face.nose_bottomLeftLimit_y.data = faces.face_info[i].extra_info[0].nose_points.bottom_left_limit_y;
    face.nose_bottomRightLimit_x.data = faces.face_info[i].extra_info[0].nose_points.bottom_right_limit_x;
    face.nose_bottomRightLimit_y.data = faces.face_info[i].extra_info[0].nose_points.bottom_right_limit_y;

    face.mouth_leftLimit_x.data = faces.face_info[i].extra_info[0].mouth_points.left_limit_x;
    face.mouth_leftLimit_y.data = faces.face_info[i].extra_info[0].mouth_points.left_limit_y;
    face.mouth_rightLimit_x.data = faces.face_info[i].extra_info[0].mouth_points.right_limit_x;
    face.mouth_rightLimit_y.data = faces.face_info[i].extra_info[0].mouth_points.right_limit_y;
    face.mouth_topLimit_x.data = faces.face_info[i].extra_info[0].mouth_points.top_limit_x;
    face.mouth_topLimit_y.data = faces.face_info[i].extra_info[0].mouth_points.top_limit_y;
    
    msg.face_array.push_back(face);
  }
}

template<class T>
geometry_msgs::Point PeopleEventRegister<T>::toCartesian(float dist, float azi, float inc) {
    geometry_msgs::Point p;
    p.x = dist * std::sin(inc) * std::cos(azi) * (-1); // Inverted
    p.y = dist * std::sin(inc) * std::sin(azi);
    p.z = dist * std::cos(inc);
    return p;
}

template<class T>
void PeopleEventRegister<T>::peopleCallbackMessage(std::string &key, qi::AnyValue &value, nao_interaction_msgs::PersonDetectedArray &msg)
{
    tools::NaoqiPersonDetected people;
    try {
        people = tools::fromAnyValueToNaoqiPersonDetected(value);
    }
    catch(std::runtime_error& e)
    {
      ROS_DEBUG_STREAM("Cannot retrieve persondetected: " << e.what());
      return;
    }
    
    msg.header.frame_id = "CameraDepth_optical_frame";
    msg.header.stamp = ros::Time::now();
    
    for(int i = 0; i < people.person_info.size(); i++) {
        std::string sid = num_to_str<int>(people.person_info[i].id);
        std::vector<std::string> keys;
        for(int j = 0; j < memory_keys.size(); j++) {
            keys.push_back(prefix+sid+memory_keys[j]);
        }
        
        qi::AnyValue data = (qi::AnyValue)p_memory_.call<qi::AnyValue>("getListData", keys);
        
        nao_interaction_msgs::PersonDetected pd;
        pd.face.gender = -1; // Since 0 = female, initialising as -1 to disambigute no data from femal.
        
        /* People Perception */
        try {
            pd.id = people.person_info[i].id;
            pd.person.distance = people.person_info[i].distance_to_camera;
            pd.person.yaw = people.person_info[i].yaw_angle_in_image;
            pd.person.pitch = people.person_info[i].pitch_angle_in_image;
            pd.person.position.position = toCartesian(pd.person.distance, pd.person.pitch, pd.person.yaw);
            pd.person.position.orientation.w = 1.0;
            
            if(data.size() != keys.size()) {
                msg.person_array.push_back(pd);
                ROS_DEBUG("Could not retrieve any face information");
                continue;
            }
            
            try {
                pd.person.face_detected = (bool)data[0].content().asInt32(); 
            } catch(...) {
                ROS_DEBUG("Error retreiving face detected");
            }
            
            try {
                pd.person.is_waving = (bool)data[1].content().asInt32();
            } catch(...) {
                ROS_DEBUG("Error retreiving if waving");
            }
            
            if(pd.person.face_detected) {
                /* Gaze Analysis */
                try {
                    qi::AnyReference gaze = data[2].content();
                    if(gaze.kind() == qi::TypeKind_List && gaze.size() == 2)
                    {
                        qi::AnyReference g, yaw, pitch;
                        yaw = gaze[0].content();
                        pitch = gaze[1].content();
                        if(yaw.kind() == qi::TypeKind_Float && pitch.kind() == qi::TypeKind_Float) {
                            tf::Quaternion q;
                            q.setRPY(0.0, pitch.asFloat(), yaw.asFloat());
                            pd.gaze.gaze_angle.x = q.x();
                            pd.gaze.gaze_angle.y = q.y();
                            pd.gaze.gaze_angle.z = q.z();
                            pd.gaze.gaze_angle.w = q.w();
                        } else {
                            ROS_DEBUG("Could not retrieve yaw/pitch");
                        }
                    } else {
                        ROS_DEBUG("Could not retrieve gaze");
                    }
                } catch(std::runtime_error& e) {
                    ROS_DEBUG_STREAM("Error retrieving gaze angle: " << e.what());
                }
                try {
                    qi::AnyReference head_angle = data[3].content();
                    if(head_angle.kind() == qi::TypeKind_List && head_angle.size() == 3)
                    {
                        qi::AnyReference yaw, pitch, roll;
                        yaw = head_angle[0].content();
                        pitch = head_angle[1].content();
                        roll = head_angle[2].content();
                        if(yaw.kind() == qi::TypeKind_Float && pitch.kind() == qi::TypeKind_Float && roll.kind() == qi::TypeKind_Float) {
                            tf::Quaternion q;
                            q.setRPY(roll.asFloat(), pitch.asFloat(), yaw.asFloat());
                            pd.gaze.head_angle.x = q.x();
                            pd.gaze.head_angle.y = q.y();
                            pd.gaze.head_angle.z = q.z();
                            pd.gaze.head_angle.w = q.w();
                        } else {
                            ROS_DEBUG("Could not retrieve yaw/pitch/roll");
                        }
                    } else {
                        ROS_DEBUG("Could not retrieve head angle");
                    }
                } catch(std::runtime_error& e) {
                    ROS_DEBUG_STREAM("Error retrieving head angle: " << e.what());
                }
                try {
                    pd.gaze.looking_at_robot = (bool)data[4].content().asInt32();
                    pd.gaze.looking_at_robot_score = data[5].content().asFloat();
                } catch(std::runtime_error& e) {
                    ROS_DEBUG_STREAM("Error retrieving looking at robot: " << e.what());
                }
                
                /* Face Charcteristics */
                try {
                    qi::AnyReference age_props = data[6].content();
                    if(age_props.kind() == qi::TypeKind_List && age_props.size() == 2)
                    {
                        qi::AnyReference age, conf;
                        age = age_props[0].content();
                        conf = age_props[1].content();
                        if(age.kind() == qi::TypeKind_Float && conf.kind() == qi::TypeKind_Float) {
                            pd.face.age = (int)age.asFloat();
                            pd.face.age_confidence = conf.asFloat();
                        } else {
                            ROS_DEBUG("Could not retrieve age and confidence");
                        }
                    } else {
                        ROS_DEBUG("Could not retrieve age");
                    }
                } catch(std::runtime_error& e) {
                    ROS_DEBUG_STREAM("Error retrieving age: " << e.what());
                }
                try {
                qi::AnyReference gender_props = data[7].content();
                    if(gender_props.kind() == qi::TypeKind_List && gender_props.size() == 2)
                    {
                        qi::AnyReference gender, conf;
                        gender = gender_props[0].content();
                        conf = gender_props[1].content();
                        if(gender.kind() == qi::TypeKind_Float && conf.kind() == qi::TypeKind_Float) {
                            pd.face.gender = (int)gender.asFloat();
                            pd.face.gender_confidence = conf.asFloat();
                        } else {
                            ROS_DEBUG("Could not retrieve gender and confidence");
                        }
                    } else {
                        ROS_DEBUG("Could not retrieve gender");
                    }
                } catch(std::runtime_error& e) {
                    ROS_DEBUG_STREAM("Error retrieving gender: " << e.what());
                }
                try {
                    qi::AnyReference smile_props = data[8].content();
                    if(smile_props.kind() == qi::TypeKind_List && smile_props.size() == 2)
                    {
                        qi::AnyReference smile_degree, conf;
                        smile_degree = smile_props[0].content();
                        conf = smile_props[1].content();
                        if(smile_degree.kind() == qi::TypeKind_Float && conf.kind() == qi::TypeKind_Float) {
                            pd.face.smile_degree = smile_degree.asFloat();
                            pd.face.smile_degree_confidence = conf.asFloat();
                        } else {
                            ROS_DEBUG("Could not retrieve smile degree and confidence");
                        }
                    } else {
                        ROS_DEBUG("Could not retrieve smile");
                    }
                } catch(std::runtime_error& e) {
                    ROS_DEBUG_STREAM("Error retrieving smile: " << e.what());
                }
                try {
                    qi::AnyReference expression_props = data[9].content();
                    if(expression_props.kind() == qi::TypeKind_List && expression_props.size() == 5)
                    {
                        qi::AnyReference neutral, happy, surprised, angry, sad;
                        neutral = expression_props[0].content();
                        happy = expression_props[1].content();
                        surprised = expression_props[2].content();
                        angry = expression_props[3].content();
                        sad = expression_props[4].content();
                        if(neutral.kind() == qi::TypeKind_Float 
                                && happy.kind() == qi::TypeKind_Float
                                && surprised.kind() == qi::TypeKind_Float
                                && angry.kind() == qi::TypeKind_Float
                                && sad.kind() == qi::TypeKind_Float) {
                            pd.face.expression_properties.neutral = neutral.asFloat();
                            pd.face.expression_properties.happy = happy.asFloat();
                            pd.face.expression_properties.surprised = surprised.asFloat();
                            pd.face.expression_properties.angry = angry.asFloat();
                            pd.face.expression_properties.sad = sad.asFloat();
                        } else {
                            ROS_DEBUG("Could not retrieve neutral/happy/surprised/angry/sad");
                        }
                    } else {
                        ROS_DEBUG("Could not retrieve expression");
                    }
                } catch(std::runtime_error& e) {
                    ROS_DEBUG_STREAM("Error retrieving expression: " << e.what());
                }
            }
        } catch(std::runtime_error& e) {
            ROS_DEBUG_STREAM("Error retrieving person information: " << e.what());
        }
        
        msg.person_array.push_back(pd);
    }
}

// http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
template class PeopleEventRegister<nao_interaction_msgs::FaceDetectedArray>;
template class PeopleEventRegister<nao_interaction_msgs::PersonDetectedArray>;

}//namespace
