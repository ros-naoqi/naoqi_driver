#ifndef ROBOT_CONFIG_SERVICE_HPP
#define ROBOT_CONFIG_SERVICE_HPP

#include <iostream>

#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>

namespace naoqi
{
namespace service
{

class RobotConfigService
{
public:
  RobotConfigService( const std::string& name, const std::string& topic );

  ~RobotConfigService(){};

  std::string name() const
  {
    return name_;
  }

  std::string topic() const
  {
    return topic_;
  }

  void reset( ros::NodeHandle& nh );

  bool callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp );


private:
  const  std::string name_;
  const  std::string topic_;

  ros::ServiceServer service_;
};

} // service
} // naoqi
#endif
