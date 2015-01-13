#include "alrosconverter.hpp"
#include <qi/anyobject.hpp>

#include <stdlib.h>

namespace ros_env
{

static void setMasterURI( const std::string& uri )
{
  setenv("ROS_MASTER_URI", uri.c_str(), 1);
}

static std::string getMasterURI( )
{
  return getenv("ROS_MASTER_URI");
}

} //ros_env


ALRosConverter::ALRosConverter( const std::string& ip)
{
  // set ros environment
  ros_env::setMasterURI( ip );
}

std::string ALRosConverter::getMasterURI() const
{
  return ros_env::getMasterURI();
}

void ALRosConverter::setMasterURI( const std::string& uri ) const
{
  ros_env::setMasterURI( uri );
}

QI_REGISTER_OBJECT( ALRosConverter, getMasterURI, setMasterURI );
