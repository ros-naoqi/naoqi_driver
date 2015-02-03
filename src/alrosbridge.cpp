#include <iostream>

/*
* NAOQI
*/
#include <qi/anyobject.hpp>

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
#include "publishers/string.hpp"
#include "publishers/int.hpp"
#include "publishers/joint_state.hpp"
#include "publishers/camera.hpp"

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
  std::cout << "BridgeService is shutting down.." << std::endl;
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
      alros::publisher::Publisher* pub = pub_queue_.top().pub_;
      ros::Time schedule = pub_queue_.top().schedule_;

      ros::Duration(schedule - ros::Time::now()).sleep();

      if ( pub->isSubscribed() && pub->isInitialized() )
      {
        // std::cout << "******************************" << std::endl;
        // std::cout << "Publisher name:\t" << pub->name() << std::endl;
        // std::cout << "Publisher subscribed:\t" << pub->isSubscribed() << std::endl;
        // std::cout << "Publisher init:\t" << pub->isInitialized() << std::endl;
        pub->publish();
      }
      // Schedule for a future time
      pub_queue_.pop();
      pub_queue_.push(ScheduledPublish(schedule + ros::Duration(1.0f / pub->frequency()), pub));
    }
    ros::spinOnce();
  }
}

// public interface here
void Bridge::registerPublisher( publisher::Publisher pub )
{

  std::vector<publisher::Publisher>::iterator it;
  it = std::find( all_publisher_.begin(), all_publisher_.end(), pub );
  // if publisher is not found, register it!
  if (it == all_publisher_.end() )
  {
    all_publisher_.push_back( pub );
    it = all_publisher_.end() - 1;
    std::cout << "registered publisher:\t" << pub.name() << std::endl;
  }
  // if found, re-init them
  else
  {
    std::cout << "re-initialized existing publisher:\t" << it->name() << std::endl;
  }

  // Schedule it for the next publish
  pub_queue_.push(ScheduledPublish(ros::Time::now() + ros::Duration(1.0f / pub.frequency()), &(*it)));
}

void Bridge::registerDefaultPublisher()
{
  qi::AnyObject p_motion = sessionPtr_->service("ALMotion");
  qi::AnyObject p_video = sessionPtr_->service("ALVideoDevice");

//  registerPublisher( alros::publisher::StringPublisher( "string_pub", "string_pub", 15) );
//  registerPublisher( alros::publisher::IntPublisher("int_pub", "int_pub", 15) );
//  registerPublisher( alros::publisher::JointStatePublisher("/joint_states", "/joint_states", 15, p_motion) );
  registerPublisher( alros::publisher::CameraPublisher("camera", "camera/front", 15, p_video) );
}

void Bridge::initPublisher()
{

  foreach( publisher::Publisher& pub, all_publisher_ )
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
