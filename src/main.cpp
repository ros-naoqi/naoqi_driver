// in main.cpp
#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>
#include <qi/session.hpp>

#include <string>
#include <ros/ros.h>


#include "publishers/publisher_string.hpp"
#include "publishers/publisher_int.hpp"
#include "publishers/publisher_joint_state.hpp"
#include "alrosbridge.hpp"

int main(int argc, char *argv[])
{

  std::cout << "application started " << std::endl;

  qi::ApplicationSession app(argc, argv);
  app.start();
  qi::SessionPtr session = app.session();

  // get host ip for ros_init
 //  std::map< std::string, std::vector<std::string> > ips = qi::os::hostIPAddrs();
 // const std::string& host_ip = ips["eth0"][0];


  // temporary debug master ip
  const std::string& ip = "http://10.0.132.69:11311";
  qi::Object<alros::Bridge> bridge( new alros::Bridge(ip)  );
  size_t service_id = session->registerService("ALRosBridge", bridge);

  qi::AnyObject p_memory = session->service("ALMemory");
  qi::AnyObject p_motion = session->service("ALMotion");



  alros::publisher::Publisher string_pub = alros::publisher::StringPublisher( "string_pub", "string_pub");
  bridge->registerPublisher( string_pub );
  bridge->registerPublisher( alros::publisher::IntPublisher("int_pub", "int_pub") );
  bridge->registerPublisher( alros::publisher::JointStatePublisher("joint_states", "joint_states", p_motion) );

  std::cout << "entering main loop" << std::endl;

//  // BAD!! FIND A BETTER BREAK CONDITION
  ros::Rate r(15);
  while( bridge->isAlive() )
  {
    bridge->publish();
    r.sleep();
  }
  ros::shutdown();
  std::cout << "shutting down .. " << std::endl;
  session->unregisterService(service_id);
  app.stop();
  return 0;
}
