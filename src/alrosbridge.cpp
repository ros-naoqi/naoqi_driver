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
#define foreach BOOST_FOREACH

/*
* PUBLIC INTERFACE
*/
#include <alrosbridge/alrosbridge.hpp>

/*
* publishers
*/
#include "publishers/camera.hpp"
#include "publishers/diagnostics.hpp"
#include "publishers/int.hpp"
#include "publishers/info.hpp"
#include "publishers/joint_state.hpp"
#include "publishers/laser.hpp"
#include "publishers/sonar.hpp"
#include "publishers/string.hpp"

/*
 * subscribers
 */
#include "subscribers/teleop.hpp"

/*
* STATIC FUNCTIONS INCLUDE
*/
#include "ros_env.hpp"
#include "helpers.hpp"

namespace alros
{

Bridge::Bridge( qi::SessionPtr& session )
  : sessionPtr_( session ),
  freq_(15),
  publish_enabled_(false),
  publish_cancelled_(false)
{
  std::cout << "application started " << std::endl;
}


Bridge::~Bridge()
{
  std::cout << "ALRosBridge is shutting down.." << std::endl;
  publish_cancelled_ = true;
  stop();
  if (publisherThread_.get_id() !=  boost::thread::id())
    publisherThread_.join();
  // destroy nodehandle?
  nhPtr_->shutdown();
  ros::shutdown();
}


void Bridge::rosLoop()
{
  while( !publish_cancelled_ )
  {
   {
      boost::mutex::scoped_lock lock( mutex_reinit_ );
      if (publish_enabled_)
      {
        // Wait for the next Publisher to be ready
        size_t pub_index = pub_queue_.top().pub_index_;
        publisher::Publisher& pub = publishers_[pub_index];
        ros::Time schedule = pub_queue_.top().schedule_;

        ros::Duration(schedule - ros::Time::now()).sleep();

        if ( pub.isSubscribed() && pub.isInitialized())
        {
          pub.publish();
        }

        // Schedule for a future time
        pub_queue_.pop();
        pub_queue_.push(ScheduledPublish(schedule + ros::Duration(1.0f / pub.frequency()), pub_index));
      } else
        // sleep one second
        ros::Duration(1).sleep();
    }
    ros::spinOnce();
  }
}

// public interface here
void Bridge::registerPublisher( publisher::Publisher pub )
{
  std::vector<publisher::Publisher>::iterator it;
  it = std::find( publishers_.begin(), publishers_.end(), pub );
  size_t pub_index = 0;

  // if publisher is not found, register it!
  if (it == publishers_.end() )
  {
    pub_index = publishers_.size();
    publishers_.push_back( pub );
    it = publishers_.end() - 1;
    std::cout << "registered publisher:\t" << pub.name() << std::endl;
  }
  // if found, re-init them
  else
  {
    pub_index = it - publishers_.begin();
    std::cout << "re-initialized existing publisher:\t" << it->name() << std::endl;
  }

  // Schedule it for the next publish
  pub_queue_.push(ScheduledPublish(ros::Time::now() + ros::Duration(1.0f / pub.frequency()), pub_index));
}

void Bridge::registerDefaultPublisher()
{
  if (!publishers_.empty())
    return;
  //registerPublisher( alros::publisher::StringPublisher( "string_pub", "string_pub", 15) );
  //registerPublisher( alros::publisher::IntPublisher("int_pub", "int_pub", 15) );
  publisher::Publisher joint_states = alros::publisher::JointStatePublisher("joint_states", "/joint_states", 15, sessionPtr_);
  registerPublisher( joint_states );
  registerPublisher( alros::publisher::CameraPublisher("front_camera", "camera/front", 10, sessionPtr_, AL::kTopCamera, AL::kQVGA) );
  registerPublisher( alros::publisher::CameraPublisher("depth_camera", "camera/depth", 10, sessionPtr_, AL::kDepthCamera, AL::kQVGA) );
  registerPublisher( alros::publisher::DiagnosticsPublisher("diagnostics", 1, sessionPtr_) );
  registerPublisher( alros::publisher::SonarPublisher("sonar", "sonar", 10, sessionPtr_) );
  registerPublisher( alros::publisher::InfoPublisher("info", "info", 1000, sessionPtr_) );

  // Pepper specific publishers
  if (joint_states.robot() == alros::PEPPER)
    registerPublisher( alros::publisher::LaserPublisher("laser", "laser", 10, sessionPtr_) );
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
  registerSubscriber( alros::subscriber::TeleopSubscriber("teleop", "cmd_vel", sessionPtr_) );
}

void Bridge::init()
{
  foreach( publisher::Publisher& pub, publishers_ )
  {
    pub.reset( *nhPtr_ );
  }
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

void Bridge::setMasterURI( const std::string& uri )
{
  // Stopping publishing
  stop();

  // Reinitializing ROS
  boost::mutex::scoped_lock lock( mutex_reinit_ );
  nhPtr_.reset();
  std::cout << "nodehandle reset " << std::endl;
  ros_env::setMasterURI( uri );
  nhPtr_.reset( new ros::NodeHandle("~") );
  lock.unlock();

  // Create the publishing thread if needed
  if (publisherThread_.get_id() ==  boost::thread::id())
    publisherThread_ = boost::thread( &Bridge::rosLoop, this );

  // register publishers, that will not start them
  registerDefaultPublisher();
  registerDefaultSubscriber();
  // initialize the publishers and subscribers with nodehandle
  init();
  // Start publishing again
  start();
}

void Bridge::start()
{
  boost::mutex::scoped_lock lock( mutex_reinit_ );
  publish_enabled_ = true;
}

void Bridge::stop()
{
  boost::mutex::scoped_lock lock( mutex_reinit_ );
  publish_enabled_ = false;
}

QI_REGISTER_OBJECT( Bridge, start, stop, getMasterURI, setMasterURI );
} //alros
