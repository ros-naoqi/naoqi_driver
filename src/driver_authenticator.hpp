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

#ifndef DRIVER_AUTHENTICATOR_HPP
#define DRIVER_AUTHENTICATOR_HPP

#include <qi/messaging/authprovider.hpp>

namespace naoqi {

class DriverAuthenticator : public qi::ClientAuthenticator {
public:
  static const std::string user_key;
  static const std::string pass_key;

  DriverAuthenticator(
    const std::string& user, 
    const std::string& pass) : _u(user), _p(pass) {}

  qi::CapabilityMap initialAuthData() {
    qi::CapabilityMap result;
    result[DriverAuthenticator::user_key] = qi::AnyValue::from(_u);
    result[DriverAuthenticator::pass_key] = qi::AnyValue::from(_p);
    return result;
  }

  qi::CapabilityMap _processAuth(const qi::CapabilityMap&) {
    return qi::CapabilityMap();
  }
private:
  std::string _u;
  std::string _p;
};

const std::string DriverAuthenticator::user_key = "user";
const std::string DriverAuthenticator::pass_key = "token";

class DriverAuthenticatorFactory : public qi::ClientAuthenticatorFactory {
public:
  std::string user;
  std::string pass;

  qi::ClientAuthenticatorPtr newAuthenticator() {
    return boost::make_shared<DriverAuthenticator>(user, pass);
  }
};

}

#endif // DRIVER_AUTHENTICATOR