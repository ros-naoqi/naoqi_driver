#include <qi/applicationsession.hpp>
#include <qi/anymodule.hpp>

int main(int argc, char** argv)
{
  qi::ApplicationSession app(argc, argv);
  app.start();
  app.session()->loadService( "alros.BridgeService" );
//app.session()->registerService("BridgeService",
//    qi::import("alros").call<qi::AnyObject>("BridgeService", app.session()));

  app.run();
  app.session()->close();
}
