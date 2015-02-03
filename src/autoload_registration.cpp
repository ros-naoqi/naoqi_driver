// in main.cpp
#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>
#include <qi/anymodule.hpp>
#include <qi/session.hpp>

#include <alrosbridge/alrosbridge.hpp>

/*
* @brief starter code for registrating the ALRosBridge module via the autoload.ini.
*/
void registerRosBridge(qi::ModuleBuilder* mb) {
  mb->advertiseFactory<alros::Bridge, qi::SessionPtr>("BridgeService");
}
QI_REGISTER_MODULE("alros", &registerRosBridge);
