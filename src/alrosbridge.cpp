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

#include "alrosbridge.hpp"

namespace alros
{

namespace ros_env
{

static void setMasterURI( const std::string& uri )
{
  if (ros::isInitialized() )
  {
    std::cout << "stopping ros init" << std::endl;
    ros::shutdown();
  }

  setenv("ROS_MASTER_URI", uri.c_str(), 1);

  std::string my_master = "__master="+uri;
  std::map< std::string, std::string > remap;
  remap["__master"] = uri;
  ros::init( remap, "alrosconverter" );
  // to prevent shutdown based on no existing nodehandle
  ros::start();

  std::cout << "using master ip: " <<  ros::master::getURI() << std::endl;
}

static std::string getMasterURI( )
{
  return getenv("ROS_MASTER_URI");
}

} // ros_env

Bridge::Bridge( const std::string& ip )
{
  ros_env::setMasterURI( ip );
  nhPtr_.reset( new ros::NodeHandle("~") );
}

Bridge::~Bridge()
{
  // destroy nodehandle?
  nhPtr_->shutdown();
}

std::string Bridge::getMasterURI() const
{
  return ros_env::getMasterURI();
}

void Bridge::setMasterURI( const std::string& uri )
{
  boost::mutex::scoped_lock lock( mutex_reinit_ );
  nhPtr_.reset();
  std::cout << "nodehandle reset " << std::endl;
  //pub_.shutdown();
  //std::cout << "publisher shutdown " << std::endl;
  ros_env::setMasterURI( uri );
  lock.unlock();

  initPublisher();
}


void Bridge::initPublisher()
{
  nhPtr_.reset( new ros::NodeHandle("~") );

  foreach( publisher::Publisher& pub, all_publisher_ )
  {
    pub.reset( *nhPtr_ );
  }
}

void Bridge::registerPublisher( publisher::Publisher pub )
{
  std::cout << "registered publisher:\t" << pub.name() << std::endl;

  if( !pub.isInitialized() )
  {
    pub.reset( *nhPtr_ );
  }

  all_publisher_.push_back( pub );
}

void Bridge::publish( )
{
  foreach( publisher::Publisher& pub, all_publisher_ )
  {
    std::cout << "******************************" << std::endl;
    std::cout << "Publisher name:\t" << pub.name() << std::endl;
    std::cout << "Publisher subscribed:\t" << pub.isSubscribed() << std::endl;
    std::cout << "Publisher init:\t" << pub.isInitialized() << std::endl;
    if ( pub.isSubscribed() )
    {
      pub.publish();
    }
  }
}

bool Bridge::isAlive() const
{
  boost::mutex::scoped_lock( mutex_reinit_ );
  return ros::ok();
}




QI_REGISTER_OBJECT( Bridge, getMasterURI, setMasterURI );

} //alros
