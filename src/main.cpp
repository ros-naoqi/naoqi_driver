// in main.cpp
#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>
#include <qi/session.hpp>

#include <string>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "alrosconverter.hpp"

int main(int argc, char *argv[])
{
  qi::ApplicationSession app(argc, argv);
  app.start();
  qi::SessionPtr session = app.session();

  // temporary debug master ip
  const std::string& ip = "http://10.0.132.69:11311";
  qi::Object<ALRosConverter> converter( new ALRosConverter(ip) );

  session->registerService("ALRosConverter", converter);
  //app.run();


  /*
  * ROS Stuff
  */
  ros::init( argc, argv, "alrosconverter" );
  std::cout << "alrosconverter init" << std::endl;

  /*
  * registers the node at ROS Master's side
  * verify with: rosnode list
  */
  ros::NodeHandle nh("~");
  std::cout << "node handle setup" << std::endl;

  ros::Publisher pub = nh.advertise< std_msgs::Int32 >( "chatter", 100 );
  std_msgs::Int32 m;
  m.data = 0;

  ros::Rate r(1);
  while( nh.ok() )
  {
    std::cout << "publishing " << m.data << std::endl;
    pub.publish( m );
    m.data++;

    r.sleep();
  }
  std::cout << "shutting down .. " << std::endl;
}
