// in main.cpp
#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>
#include <qi/anymodule.hpp>
#include <qi/session.hpp>

#include <alrosbridge/alrosbridge.hpp>

/*
#include <string>
#include <ros/ros.h>

#include "publishers/publisher_string.hpp"
#include "publishers/publisher_int.hpp"
#include "publishers/publisher_joint_state.hpp"

namespace alros
{

class BridgeService
{

public:
  qi::SessionPtr sessionPtr;
  qi::Object<alros::Bridge> bridge;
  size_t service_id;

  BridgeService( qi::SessionPtr& session )
    : sessionPtr( session )
  {
    std::cout << "application started " << std::endl;
    // temporary debug master ip
    const std::string& ip = "http://10.0.132.69:11311";
    qi::Object<alros::Bridge> bridge( new alros::Bridge(ip)  );
    service_id = session->registerService("ALRosBridge", bridge);

    //qi::AnyObject p_memory = session->service("ALMemory");
    qi::AnyObject p_motion = session->service("ALMotion");

    alros::publisher::Publisher string_pub = alros::publisher::StringPublisher( "string_pub", "string_pub");
    bridge->registerPublisher( string_pub );
    bridge->registerPublisher( alros::publisher::IntPublisher("int_pub", "int_pub") );
    bridge->registerPublisher( alros::publisher::JointStatePublisher("/joint_states", "/joint_states", p_motion) );

    mainLoop();
  }

  void mainLoop()
  {
    std::cout << "entering main loop" << std::endl;

    // BAD!! FIND A BETTER BREAK CONDITION
    ros::Rate r(15);
    while( bridge->isAlive() )
    {
      bridge->publish();
      r.sleep();
    }
    ros::shutdown();
    std::cout << "shutting down .. " << std::endl;
    sessionPtr->unregisterService(service_id);
  }
}; // class BridgeService

} //alros
*/

// WTF module initialization
void registerRosBridge(qi::ModuleBuilder* mb) {
  mb->advertiseFactory<alros::Bridge, qi::SessionPtr>("BridgeService");
}
QI_REGISTER_MODULE("alros", &registerRosBridge);


