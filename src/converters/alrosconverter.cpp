#include "alrosconverter.hpp"
#include <qi/anyobject.hpp>

// BOOST
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


#include <stdlib.h>

// ROS
#include <std_msgs/Int32.h>


namespace alros
{

namespace ros_env
{
int argc;
char** argv;

static void setMasterURI( const std::string& uri )
{
  std::cout << "setting (new) Master URI" << std::endl;

  if (ros::isInitialized() )
  {
    std::cout << "stopping ros init" << std::endl;
    ros::shutdown();
  }

  setenv("ROS_MASTER_URI", uri.c_str(), 1);

  // to be tested whether we have to store argc/argv somewhere
  //ros::init( argc, argv, "alrosconverter" );
  std::string my_master = "__master="+uri;
  std::map< std::string, std::string > remap;
  remap["__master"] = uri;
  ros::init( remap, "alrosconverter" );
  // to prevent shutdown based on no existing nodehandle
  ros::start();

  std::cout << "using master ip: " <<  ros::master::getURI() << std::endl;
  std::cout << "ros (re-)initialized" << std::endl;
}

static std::string getMasterURI( )
{
  return getenv("ROS_MASTER_URI");
}

} //ros_env


ALRosConverter::ALRosConverter( int argc, char** argv, const std::string& ip)
{
  std::cout << "in constructor " << std::endl;
  // set ros environment
  ros_env::argc = argc;
  ros_env::argv = argv;
  ros_env::setMasterURI( ip );

  initPublisher();
}

void ALRosConverter::initPublisher()
{
  nhPtr_.reset( new ros::NodeHandle("~") );
  pub_ = nhPtr_->advertise< std_msgs::Int32 >( "chatter", 100 );

  std::cout << "ros components initialized" << std::endl;
}

std::string ALRosConverter::getMasterURI() const
{
  return ros_env::getMasterURI();
}

void ALRosConverter::setMasterURI( const std::string& uri )
{
  boost::mutex::scoped_lock lock( mutex_reinit_ );
  nhPtr_.reset();
  std::cout << "nodehandle reset " << std::endl;
  pub_.shutdown();
  std::cout << "publisher shutdown " << std::endl;
  ros_env::setMasterURI( uri );
  lock.unlock();

  initPublisher();
}

bool ALRosConverter::isAlive() const
{
  boost::mutex::scoped_lock( mutex_reinit_ );
  return ros::ok();
}

void ALRosConverter::update()
{
  static std_msgs::Int32 m;
  m.data++;
  std::cout << "publishing in update method" << std::endl;
  pub_.publish( m );
}

QI_REGISTER_OBJECT( ALRosConverter, getMasterURI, setMasterURI );

} // alros
