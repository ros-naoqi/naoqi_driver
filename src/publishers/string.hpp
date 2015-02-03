#ifndef STRING_PUBLISHER_HPP
#define STRING_PUBLISHER_HPP

#include <ros/ros.h>

#include "publisher_base.hpp"

namespace alros
{
namespace publisher
{

class StringPublisher : public BasePublisher<StringPublisher>
{

public:
  StringPublisher( const std::string& name, const std::string& topic, float frequency );
  StringPublisher( const std::string& name, const std::string& topic, float frequency, ros::NodeHandle& nh );

  void publish();

  void reset( ros::NodeHandle& nh );

  inline bool isSubscribed() const
  {
    if (is_initialized_ == false) return false;
    return pub_.getNumSubscribers() > 0;
  }

private:
  ros::Publisher pub_;
}; // class

} //publisher
} // alros

#endif
