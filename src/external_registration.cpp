/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


#include <qi/applicationsession.hpp>
#include <qi/anymodule.hpp>
#include <alrosbridge/alrosbridge.hpp>

int main(int argc, char** argv)
{
  qi::ApplicationSession app(argc, argv);
  app.start();
  boost::shared_ptr<alros::Bridge> bs = qi::import("alros").call<qi::Object<alros::Bridge> >("FlightRecorder", app.session()).asSharedPtr();
  app.session()->registerService("FlightRecorder", bs);

  bs->init();

  //! @note Must call ow._stopService when the application stops to do the clean-up
  app.atStop(boost::function<void()>(
                 boost::bind(&alros::Bridge::stopService, boost::ref(bs))));

  app.run();
  app.session()->close();
}
