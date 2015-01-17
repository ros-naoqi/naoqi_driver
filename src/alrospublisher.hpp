#ifndef ALROSPUBLISHER_HPP
#define ALROSPUBLISHER_HPP

#include <string>

/*
* ROS DEPENDENCIES
*/
#include <ros/ros.h>

namespace alros
{

/**
* @brief: Base Class to wrap naoqi functionialities around a ros publisher
*/
class Publisher
{

  /**
  * @brief: Base Constructor
  * @param: ros topic to publish on
  */
  Publisher( const std::string& topic );

  /**
  * @brief: Virtual Desctructor
  */
  ~Publisher();

  /**
  * @brief: abstract base function for publishing
  */
  int publish() = 0;
}



} // alros
#endif
