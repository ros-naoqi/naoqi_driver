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
#include "publishers/joint_state.hpp"
#include "publishers/string.hpp"

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
  publish_enabled_(false)
{
  std::cout << "application started " << std::endl;
}


Bridge::~Bridge()
{
  std::cout << "ALRosBridge is shutting down.." << std::endl;
  stop();
  // destroy nodehandle?
  nhPtr_->shutdown();
  ros::shutdown();
  publisherThread_.join();
}


void Bridge::rosLoop()
{
  while( ros::ok() )
  {
    if ( isAlive() )
    {
      // Wait for the next Publisher to be ready
      size_t pub_index = pub_queue_.top().pub_index_;
      publisher::Publisher& pub = publishers_[pub_index];
      ros::Time schedule = pub_queue_.top().schedule_;

      ros::Duration(schedule - ros::Time::now()).sleep();

      if ( pub.isSubscribed() && pub.isInitialized() )
      {
        pub.publish();
      }
      // Schedule for a future time
      pub_queue_.pop();
      pub_queue_.push(ScheduledPublish(schedule + ros::Duration(1.0f / pub.frequency()), pub_index));
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
  qi::AnyObject p_motion = sessionPtr_->service("ALMotion");
  qi::AnyObject p_video = sessionPtr_->service("ALVideoDevice");
  qi::AnyObject p_memory = sessionPtr_->service("ALMemory");

  registerPublisher( alros::publisher::StringPublisher( "string_pub", "string_pub", 15) );
  registerPublisher( alros::publisher::IntPublisher("int_pub", "int_pub", 15) );
  registerPublisher( alros::publisher::JointStatePublisher("/joint_states", "/joint_states", 15, p_motion) );
  registerPublisher( alros::publisher::CameraPublisher("front_camera", "camera/front", 10, p_video, AL::kTopCamera, AL::kQVGA) );
  registerPublisher( alros::publisher::CameraPublisher("depth_camera", "camera/depth", 10, p_video, AL::kDepthCamera, AL::kQVGA) );
  registerPublisher( alros::publisher::DiagnosticsPublisher("diagnostics", 1, p_memory, p_motion) );
}

void Bridge::initPublisher()
{
  foreach( publisher::Publisher& pub, publishers_ )
  {
    pub.reset( *nhPtr_ );
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
  boost::mutex::scoped_lock lock( mutex_reinit_ );
  nhPtr_.reset();
  std::cout << "nodehandle reset " << std::endl;
  ros_env::setMasterURI( uri );
  nhPtr_.reset( new ros::NodeHandle("~") );
  lock.unlock();

  publisherThread_ = boost::thread( &Bridge::rosLoop, this );

  // first register publisher
  registerDefaultPublisher();
  // second initialize them with nodehandle
  initPublisher();
  start();
}

void Bridge::start()
{
  boost::mutex::scoped_lock( mutex_reinit_ );
  publish_enabled_ = true;
}

void Bridge::stop()
{
  boost::mutex::scoped_lock( mutex_reinit_ );
  publish_enabled_ = false;
}

bool Bridge::isAlive() const
{
  boost::mutex::scoped_lock( mutex_reinit_ );
  return publish_enabled_;
}

QI_REGISTER_OBJECT( Bridge, start, stop, getMasterURI, setMasterURI );
} //alros
