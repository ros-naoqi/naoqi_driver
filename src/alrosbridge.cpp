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

/*
* NAOQI
*/
#include <qi/anyobject.hpp>
#include <alvision/alvisiondefinitions.h> // for kTop...
/*
* BOOST
*/
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#define foreach BOOST_FOREACH
/*
* ROS
*/
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

/*
* PUBLIC INTERFACE
*/
#include <alrosbridge/alrosbridge.hpp>
#include <alrosbridge/message_actions.h>
/*
 * CONVERTERS
 */
#include "converters/int.hpp"
#include "converters/string.hpp"
#include "converters/camera.hpp"

/*
* publishers
*/
#include "publishers/camera.hpp"
//#include "publishers/diagnostics.hpp"
#include "publishers/int.hpp"
//#include "publishers/info.hpp"
//#include "publishers/joint_state.hpp"
//#include "publishers/nao_joint_state.hpp"
//#include "publishers/odometry.hpp"
//#include "publishers/laser.hpp"
//#include "publishers/log.hpp"
//#include "publishers/sonar.hpp"
#include "publishers/string.hpp"

/*
 * subscribers
 */
#include "subscribers/teleop.hpp"
#include "subscribers/moveto.hpp"

/*
* STATIC FUNCTIONS INCLUDE
*/
#include "ros_env.hpp"
#include "helpers.hpp"

/*
 * ROS
 */
#include <tf2_ros/buffer.h>

namespace alros
{

Bridge::Bridge( qi::SessionPtr& session )
  : sessionPtr_( session ),
  freq_(15),
  publish_enabled_(false),
  publish_cancelled_(false),
  record_enabled_(false),
  record_cancelled_(false),
  _recorder(boost::make_shared<Recorder>())
{
  //_recorder = std::make_shared<Recorder>();
}


Bridge::~Bridge()
{
  std::cout << "ALRosBridge is shutting down.." << std::endl;
  // destroy nodehandle?
  if(nhPtr_)
  {
    nhPtr_->shutdown();
    ros::shutdown();
  }
}

void Bridge::stopService() {
  publish_cancelled_ = true;
  stop();
  if (publisherThread_.get_id() !=  boost::thread::id())
    publisherThread_.join();
  publishers_.clear();
  subscribers_.clear();
}


void Bridge::rosLoop()
{
  while( !publish_cancelled_ )
  {
   {
      boost::mutex::scoped_lock lock( mutex_reinit_ );
      if (publish_enabled_ && !conv_queue_.empty())
      {
        // Wait for the next Publisher to be ready
        size_t conv_index = conv_queue_.top().conv_index_;
        converter::Converter conv = converters_[conv_index];
        ros::Time schedule = conv_queue_.top().schedule_;

        ros::Duration(schedule - ros::Time::now()).sleep();


        std::vector<message_actions::MessageAction> actions;
        actions.push_back( message_actions::PUBLISH );
        conv.callAll( actions );
        //if ( pub.isSubscribed() && pub.isInitialized())
        //{
        //  pub.publish();
        //  if (_recorder->isRecording()) {
        //    std_msgs::Int32 i;
        //    i.data = 32;
        //    geometry_msgs::PointStamped ps;
        //    ps.point.x = 2;
        //    ps.point.y = 4;
        //    ps.point.z = 6;
        //    _recorder->write(pub.name(), ps);
        //  }
        //}

        // Schedule for a future time or not
        conv_queue_.pop();
        if ( conv.frequency() != 0 )
          conv_queue_.push(ScheduledConverter(schedule + ros::Duration(1.0f / conv.frequency()), conv_index));
      } else
        // sleep one second
        ros::Duration(1).sleep();
    }
    ros::spinOnce();
  }
}

void Bridge::registerConverter( converter::Converter conv )
{
}

void Bridge::registerDefaultConverter()
{
}

// public interface here
void Bridge::registerPublisher( publisher::Publisher pub )
{
//  std::vector< boost::shared_ptr<publisher::Publisher> >::iterator it;
//  it = std::find( publishers_.begin(), publishers_.end(), pub );
//
//  // if publisher is not found, register it!
//  if (it == publishers_.end() )
//  {
//    publishers_.push_back( boost::make_shared<publisher::Publisher>(pub) );
//    it = publishers_.end() - 1;
//    std::cout << "registered publisher:\t" << pub.name() << std::endl;
//  }
//  // if found, re-init them
//  else
//  {
//    std::cout << "re-initialized existing publisher:\t" << (*it)->name() << std::endl;
//  }
}

void Bridge::registerDefaultPublisher()
{
//  if (!publishers_.empty())
//    return;
//
//  // Info should be at 0 (latched) but somehow that does not work ...
//  publisher::Publisher info = alros::publisher::InfoPublisher("info", "info", 0.1, sessionPtr_);
//  registerPublisher( info );
//
//  // Define the tf2 buffer
//  if (info.robot() == alros::PEPPER)
//  {
//    alros::publisher::JointStatePublisher publisher("joint_states", "/joint_states", 15, sessionPtr_);
//    tf2_buffer_ = publisher.getTF2Buffer();
//    registerPublisher( publisher );
//  }
//  if (info.robot() == alros::NAO)
//  {
//    alros::publisher::NaoJointStatePublisher publisher( "nao_joint_states", "/joint_states", 15, sessionPtr_);
//    tf2_buffer_ = publisher.getTF2Buffer();
//    registerPublisher( publisher );
//  }
//
//  registerPublisher( alros::publisher::OdometryPublisher( "odometry", "/odom", 15, sessionPtr_, tf2_buffer_) );
//  registerPublisher( alros::publisher::CameraPublisher("front_camera", "camera/front", 10, sessionPtr_, AL::kTopCamera, AL::kQVGA) );
//  registerPublisher( alros::publisher::DiagnosticsPublisher("diagnostics", 1, sessionPtr_) );
//  registerPublisher( alros::publisher::SonarPublisher("sonar", "sonar", 10, sessionPtr_) );
//  registerPublisher( alros::publisher::LogPublisher("logger", "", 5, sessionPtr_) );
//
//  // Pepper specific publishers
//  if (info.robot() == alros::PEPPER)
//  {
//    registerPublisher( alros::publisher::LaserPublisher("laser", "laser", 10, sessionPtr_) );
//    registerPublisher( alros::publisher::CameraPublisher("depth_camera", "camera/depth", 10, sessionPtr_, AL::kDepthCamera, AL::kQVGA) );
//  }



  /** String Publisher */
  boost::shared_ptr<publisher::StringPublisher> sp = boost::make_shared<publisher::StringPublisher>( "string" );
  sp->reset( *nhPtr_ );
  converter::StringConverter sc( "string_converter", 10, sessionPtr_ );
  sc.registerCallback( message_actions::PUBLISH, boost::bind(&publisher::StringPublisher::publish, sp, _1) );
  converters_.push_back( sc );

  /** Int Publisher */
  boost::shared_ptr<publisher::IntPublisher> ip = boost::make_shared<publisher::IntPublisher>( "int" );
  ip->reset( *nhPtr_ );
  converter::IntConverter ic( "int_converter", 15, sessionPtr_);
  ic.registerCallback( message_actions::PUBLISH, boost::bind(&publisher::IntPublisher::publish, ip, _1) );
  //sc.registerCallback( message_actions::RECORD, boost::bind(&Recorder::record<std_msgs::String>, &rec, _1) );
  converters_.push_back( ic );

  /** Front Camera */
  boost::shared_ptr<publisher::CameraPublisher> fcp = boost::make_shared<publisher::CameraPublisher>( "front_camera", AL::kTopCamera );
  fcp->reset( *nhPtr_ );
  converter::CameraConverter fcc( "front_camera_converter", 10, sessionPtr_, AL::kTopCamera, AL::kQVGA );
  fcc.registerCallback( message_actions::PUBLISH, boost::bind(&publisher::CameraPublisher::publish, fcp, _1, _2) );
  converters_.push_back( fcc );

  /** Depth Camera */
  boost::shared_ptr<publisher::CameraPublisher> dcp = boost::make_shared<publisher::CameraPublisher>( "depth_camera", AL::kDepthCamera );
  dcp->reset( *nhPtr_ );
  converter::CameraConverter dcc( "depth_camera_converter", 10, sessionPtr_, AL::kDepthCamera, AL::kQVGA );
  dcc.registerCallback( message_actions::PUBLISH, boost::bind(&publisher::CameraPublisher::publish, dcp, _1, _2) );
  converters_.push_back( dcc );

}

// public interface here
void Bridge::registerSubscriber( subscriber::Subscriber sub )
{
  std::vector<subscriber::Subscriber>::iterator it;
  it = std::find( subscribers_.begin(), subscribers_.end(), sub );
  size_t sub_index = 0;

  // if subscriber is not found, register it!
  if (it == subscribers_.end() )
  {
    sub_index = subscribers_.size();
    subscribers_.push_back( sub );
    std::cout << "registered subscriber:\t" << sub.name() << std::endl;
  }
  // if found, re-init them
  else
  {
    std::cout << "re-initialized existing subscriber:\t" << it->name() << std::endl;
  }
}

void Bridge::registerDefaultSubscriber()
{
  if (!subscribers_.empty())
    return;
  registerSubscriber( alros::subscriber::TeleopSubscriber("teleop", "/cmd_vel", sessionPtr_) );
  registerSubscriber( alros::subscriber::MovetoSubscriber("moveto", "/move_base_simple/goal", sessionPtr_, tf2_buffer_) );
}

void Bridge::init()
{
  // init converters
  conv_queue_ =  std::priority_queue<ScheduledConverter>();
  size_t conv_index = 0;
  foreach( converter::Converter& conv, converters_ )
  {
    // Schedule it for the next publish
    // COULD BE REFACTORED DUE TU POINTER
    conv_queue_.push(ScheduledConverter(ros::Time::now(), conv_index));
    ++conv_index;
  }

  // init publishers
  //stringPublisherPtr_->reset( *nhPtr_ );
  //intPublisherPtr_->reset( *nhPtr_ );
  //foreach( boost::shared_ptr<publisher::Publisher>& pub, publishers_ )
  //{
  //  pub->reset( *nhPtr_ );
  //}

  // init subscribers
  foreach( subscriber::Subscriber& sub, subscribers_ )
  {
    sub.reset( *nhPtr_ );
  }
}


/*
* EXPOSED FUNCTIONS
*/

std::string Bridge::getMasterURI() const
{
  return ros_env::getMasterURI();
}

void Bridge::setMasterURI( const std::string& uri)
{
  setMasterURINet(uri, "eth0");
}
void Bridge::setMasterURINet( const std::string& uri, const std::string& network_interface)
{
  // Stopping publishing
  stopPublishing();

  // Reinitializing ROS
  {
    boost::mutex::scoped_lock lock( mutex_reinit_ );
    nhPtr_.reset();
    std::cout << "nodehandle reset " << std::endl;
    ros_env::setMasterURI( uri, network_interface );
    nhPtr_.reset( new ros::NodeHandle("~") );
  }
  // Create the publishing thread if needed
  if (publisherThread_.get_id() ==  boost::thread::id())
    publisherThread_ = boost::thread( &Bridge::rosLoop, this );

  // register publishers, that will not start them
  registerDefaultPublisher();
  registerDefaultSubscriber();
  // initialize the publishers and subscribers with nodehandle
  init();
  // Start publishing again
  startPublishing();
}

void Bridge::startPublishing()
{
  boost::mutex::scoped_lock lock( mutex_reinit_ );
  publish_enabled_ = true;
}

void Bridge::stopPublishing()
{
  boost::mutex::scoped_lock lock( mutex_reinit_ );
  publish_enabled_ = false;
}

void Bridge::startRecord()
{
  _recorder->startRecord();
}

void Bridge::stopRecord()
{
  _recorder->stopRecord();
}

QI_REGISTER_OBJECT( Bridge, startPublishing, stopPublishing, getMasterURI, setMasterURI, setMasterURINet,
                    startRecord, stopRecord );
} //alros
