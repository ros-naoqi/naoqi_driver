// in main.cpp
#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>
#include <qi/session.hpp>

#include <string>
#include <ros/ros.h>


#include "publishers/publisher_string.hpp"
#include "publishers/publisher_int.hpp"
#include "alrosbridge.hpp"

int main(int argc, char *argv[])
{

 std::cout << "application started " << std::endl;

 qi::ApplicationSession app(argc, argv);
 app.start();
 qi::SessionPtr session = app.session();

 // temporary debug master ip
 const std::string& ip = "http://10.0.132.69:11311";
 qi::Object<alros::Bridge> bridge( new alros::Bridge(ip)  );

 session->registerService("ALRosBridge", bridge);

  alros::publisher::Publisher string_pub = alros::publisher::StringPublisher( "string_pub", "string_pub");
  bridge->registerPublisher( string_pub );
  bridge->registerPublisher( alros::publisher::IntPublisher("int_pub", "int_pub") );

  std::cout << "entering main loop" << std::endl;

//  // BAD!! FIND A BETTER BREAK CONDITION
  ros::Rate r(15);
  while( bridge->isAlive() )
  {
    bridge->publish();
    r.sleep();
  }
  std::cout << "shutting down .. " << std::endl;
  return 0;
}
