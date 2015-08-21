#include "robot_config.hpp"

namespace naoqi
{
namespace service
{

RobotConfigService::RobotConfigService( const std::string& name, const std::string& topic )
  : name_(name),
  topic_(topic)
{}

void RobotConfigService::reset( ros::NodeHandle& nh )
{
  std::cout << "i will advertise my ros service on topic " << topic_ << std::endl;
  service_ = nh.advertiseService(topic_, &RobotConfigService::callback, this);
}

bool RobotConfigService::callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp )
{
  std::cout << "triggering robot config service" << std::endl;
  return true;
}


}
}
