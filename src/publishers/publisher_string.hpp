#ifndef STRING_PUBLISHER_HPP
#define STRING_PUBLISHER_HPP

#include <ros/ros.h>

#include "publisher_base.hpp"

namespace alros
{
namespace publisher
{

class StringPublisher : public BasePublisher
{

public:
  StringPublisher( const std::string& name, const std::string& topic );
  StringPublisher( const std::string& name, const std::string& topic, ros::NodeHandle& nh );

  void publish();

  void reset( ros::NodeHandle& nh );

//private:
//  ros::Publisher pub_;
}; // class

} //publisher
} // alros

#endif
