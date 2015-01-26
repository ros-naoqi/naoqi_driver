#include <ros/ros.h>
#include <std_msgs/String.h>

#include "float_array.h"
#include "benchmark_helpers.h"

int main( int argc, char** argv )
{

  std::cout << "number of argc " << argc << std::endl;

  size_t frequence = helpers::get_frequency( argc, argv );
  size_t buffer_size = helpers::get_buffer_size( argc, argv );

  std::cout << "DONT FORGET TO EXPORT ROS_IP AND MASTER_URI BEFOREHAND" << std::endl;

  ros::init( argc, argv, "simple_publisher" );
  ros::start();

  ros::NodeHandle nh;
  //ros::Publisher pub = nh.advertise<std_msgs::String>( "string", buffer_size );
  ros::Publisher pub = nh.advertise< float_array_msg::float_array >( "/benchmark/uint8_array", buffer_size );
  std::cout << "running a publisher with framerate " << frequence << " and buffersize " << buffer_size << std::endl;

  ros::Rate r( frequence );
  static float_array_msg::float_array m;
  m.data.resize( buffer_size );

  std::cout << "starting to publish .." << std::endl;
  while ( ros::ok() )
  {
    pub.publish( m );
    ros::spinOnce();
    r.sleep();
  }
  std::cout << "shutting down .. " << std::endl;

  return 0;
}
