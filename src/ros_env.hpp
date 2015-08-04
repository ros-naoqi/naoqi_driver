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


#ifndef ROS_ENV_HPP
#define ROS_ENV_HPP

/*
* ROS includes
*/
#include <ros/ros.h>


/*
* ALDEBARAN includes
*/
#include <qi/os.hpp>

#include <stdlib.h>

#include <boost/algorithm/string.hpp>

namespace naoqi
{
namespace ros_env
{

/** Queries NAOqi to get the IP
 * @param network_interface the name of the network interface to use. "eth0" by default. If you put your NAO in
 * tethering mode, you will need to put "tether"
 */
static std::string getROSIP(std::string network_interface)
{
  if (network_interface.empty())
    network_interface = "eth0";

  typedef std::map< std::string, std::vector<std::string> > Map_IP;
  Map_IP map_ip = static_cast<Map_IP>(qi::os::hostIPAddrs());
  if ( map_ip.find(network_interface) == map_ip.end() ) {
    std::cerr << "Could not find network interface named " << network_interface << ", possible interfaces are ... ";
    for (Map_IP::iterator it=map_ip.begin(); it!=map_ip.end(); ++it) std::cerr << it->first <<  " ";
    std::cerr << std::endl;
    exit(1);
  }

  static const std::string ip = map_ip[network_interface][0];
  return ip;
}

static std::string prefix = "";

static void setPrefix( std::string s )
{
  prefix = s;
  std::cout << "set prefix successfully to " << prefix << std::endl;
}

static std::string getPrefix()
{
  return prefix;
}

static void setMasterURI( const std::string& uri, const std::string& network_interface )
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
  remap["__ip"] = ::naoqi::ros_env::getROSIP(network_interface);
  // init ros without a sigint-handler in order to shutdown correctly by naoqi
  ros::init( remap, ::naoqi::ros_env::getPrefix(), ros::init_options::NoSigintHandler );
  // to prevent shutdown based on no existing nodehandle
  ros::start();

  std::cout << "using master ip: " <<  ros::master::getURI() << std::endl;
}

static std::string getMasterURI( )
{
  return getenv("ROS_MASTER_URI");
}


} // ros_env
} // naoqi
#endif
