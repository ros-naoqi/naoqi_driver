#ifndef INT_PUBLISHER_HPP
#define INT_PUBLISHER_HPP

#include <ros/ros.h>

#include "publisher_base.hpp"

namespace alros
{
namespace publisher
{

class IntPublisher : public BasePublisher<IntPublisher>
{

public:
  IntPublisher( const std::string& name, const std::string& topic );
  IntPublisher( const std::string& name, const std::string& topic, ros::NodeHandle& nh );

  void publish();

  void reset( ros::NodeHandle& nh );

//private:
//  ros::Publisher pub_;
}; // class

} //publisher
} // alros

#endif
