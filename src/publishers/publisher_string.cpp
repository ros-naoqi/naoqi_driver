#include <iostream>

#include "publisher_string.hpp"

namespace alros
{
namespace publisher
{

StringPublisher::StringPublisher( const std::string& name, const std::string& topic ):
    BasePublisher( name, topic )
  {}

void StringPublisher::publish()
{
  std::cout << name() << " is publishing " << std::endl;
}

void StringPublisher::reset()
{
  std::cout << name() << " is resetting" << std::endl;
}


} //publisher
} // alros
