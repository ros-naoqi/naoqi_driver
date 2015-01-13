// in main.cpp
#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>
#include <qi/session.hpp>

#include <string>
#include <ros/ros.h>

#include "alrosconverter.hpp"

int main(int argc, char *argv[])
{
  qi::ApplicationSession app(argc, argv);
  app.start();
  qi::SessionPtr session = app.session();

  // tmp master ip
  const std::string& ip = "http://10.0.132.69:11311";
  qi::Object<ALRosConverter> converter( new ALRosConverter(ip) );

  ros::init( argc, argv, "alrosconverter" );
  std::cout << "alrosconverter init" << std::endl;

  ros::NodeHandle nh;
  std::cout << "node handle setup" << std::endl;

  session->registerService("ALRosConverter", converter);
  app.run();
}
