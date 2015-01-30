#ifndef ROS_ENV_HPP
#define ROS_ENV_HPP

/**
* ROS includes
*/
#include <ros/ros.h>


/**
* ALDEBARAN includes
*/
#include <qi/os.hpp>

#include <stdlib.h>

namespace alros
{
namespace ros_env
{

static std::string getROSIP()
{
  typedef std::map< std::string, std::vector<std::string> > Map_IP;
  static const std::string ip = static_cast<Map_IP>(qi::os::hostIPAddrs())["eth0"][0];
  return ip;
}

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
  remap["__ip"] = ::alros::ros_env::getROSIP();
  // init ros without a sigint-handler in order to shutdown correctly by naoqi
  ros::init( remap, "alrosconverter", ros::init_options::NoSigintHandler );
  // to prevent shutdown based on no existing nodehandle
  ros::start();

  std::cout << "using master ip: " <<  ros::master::getURI() << std::endl;
}

static std::string getMasterURI( )
{
  return getenv("ROS_MASTER_URI");
}

} // ros_env
} // alros
#endif
