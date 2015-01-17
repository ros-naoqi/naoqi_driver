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

 std::cout << "application started " << std::endl;

 qi::ApplicationSession app(argc, argv);
 app.start();
 qi::SessionPtr session = app.session();

 // temporary debug master ip
 const std::string& ip = "http://10.0.132.69:11311";
 alros::ALRosConverter* conv =new alros::ALRosConverter(argc, argv, ip);
 qi::Object<alros::ALRosConverter> converter( conv );

 session->registerService("ALRosConverter", converter);
// //app.run();

  std::cout << "entering main loop" << std::endl;

//  // BAD!! FIND A BETTER BREAK CONDITION
  ros::Rate r(1);
  while( converter->isAlive() )
  {
    converter->update();
    std::cout << "publishing alive " << std::endl;
    r.sleep();
  }
  std::cout << "shutting down .. " << std::endl;
  return 0;
}
