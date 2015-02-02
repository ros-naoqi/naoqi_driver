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
* publishers
*/
#include "publishers/publisher_string.hpp"
#include "publishers/publisher_int.hpp"
#include "publishers/publisher_joint_state.hpp"
#include "publishers/publisher_camera.hpp"

#include "alrosbridge.hpp"
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
  // temporary debug master ip
  //const std::string& ip = "http://10.0.132.69:11311";
  //ros_env::setMasterURI( ip );

  //nhPtr_.reset( new ros::NodeHandle("~") );
  //registerDefaultPublisher();

  // publish_enabled_ set to false by default
  //publisherThread_ = boost::thread( &Bridge::rosLoop, this );
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
  static ros::Rate r(freq_);
  while( ros::ok() )
  {
    if ( isAlive() )
    {
      publish();
      ros::spinOnce();
    }
    r.sleep();
  }
}

void Bridge::publish( )
{
  foreach( publisher::Publisher& pub, all_publisher_ )
  {
    std::cout << "******************************" << std::endl;
    std::cout << "Publisher name:\t" << pub.name() << std::endl;
    std::cout << "Publisher subscribed:\t" << pub.isSubscribed() << std::endl;
    std::cout << "Publisher init:\t" << pub.isInitialized() << std::endl;
    if ( pub.isSubscribed() && pub.isInitialized() )
    {
      pub.publish();
    }
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
    if( !pub.isInitialized() )
    {
      pub.reset( *nhPtr_ );
    }
    all_publisher_.push_back( pub );
    std::cout << "registered publisher:\t" << pub.name() << std::endl;
  }
  // if found, re-init them
  else
  {
    if ( !it->isInitialized() )
    {
      it->reset( *nhPtr_ );
    }
    std::cout << "re-initialized existing publisher:\t" << it->name() << std::endl;
  }
}

void Bridge::registerDefaultPublisher()
{
  qi::AnyObject p_motion = sessionPtr_->service("ALMotion");
  qi::AnyObject p_video = sessionPtr_->service("ALVideoDevice");

//  registerPublisher( alros::publisher::StringPublisher( "string_pub", "string_pub") );
//  registerPublisher( alros::publisher::IntPublisher("int_pub", "int_pub") );
//  registerPublisher( alros::publisher::JointStatePublisher("/joint_states", "/joint_states", p_motion) );
  registerPublisher( alros::publisher::CameraPublisher("camera", "camera/front", p_video) );
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
