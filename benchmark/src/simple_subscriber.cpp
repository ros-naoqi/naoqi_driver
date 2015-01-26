#include <ros/ros.h>
#include <std_msgs/String.h>

#include "benchmark_helpers.h"

void stringCallback( const std_msgs::String::ConstPtr& msg )
{
  std::cout << "received " << msg->data << std::endl;
}

int main( int argc, char** argv )
{

  size_t frequence = helpers::get_frequency( argc, argv );
  size_t buffer_size = helpers::get_buffer_size( argc, argv );

  std::cout << "DONT FORGET TO EXPORT ROS_IP AND MASTER_URI BEFOREHAND" << std::endl;

  ros::init( argc, argv, "simple_subscriber" );
  ros::start();
  ros::Rate r( frequence );

  ros::NodeHandle nh("~");
  ros::Subscriber sub = nh.subscribe( "string", buffer_size, stringCallback );
  std::cout << "running a subscriber with framerate " << frequence << " and buffersize " << buffer_size << std::endl;
  std::cout << "starting to listen .." << std::endl;
  while ( ros::ok() )
  {
    ros::spinOnce();
    r.sleep();
  }
  std::cout << "shutting down .. " << std::endl;

  return 0;
}
