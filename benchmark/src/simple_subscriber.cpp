#include <ros/ros.h>
#include <std_msgs/String.h>

#include "float_array.h"
#include "benchmark_helpers.h"

void uint8Callback( const float_array_msg::float_array::ConstPtr& msg )
{
  std::vector<uint8_t> data_vec = msg->data;
  std::cout << "data size: " << data_vec.size() << std::endl;
}

int main( int argc, char** argv )
{

  size_t frequence = helpers::get_frequency( argc, argv );
  size_t buffer_size = helpers::get_buffer_size( argc, argv );

  std::cout << "DONT FORGET TO EXPORT ROS_IP AND MASTER_URI BEFOREHAND" << std::endl;

  ros::init( argc, argv, "simple_subscriber" );
  ros::start();
  ros::Rate r( frequence );

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe( "/benchmark/uint8_array", buffer_size, uint8Callback );
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
