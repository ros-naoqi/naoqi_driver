#include <iostream>

#include "publisher_int.hpp"

namespace alros
{
namespace publisher
{

IntPublisher::IntPublisher( const std::string& name, const std::string& topic ):
  BasePublisher( name, topic )
{}

void IntPublisher::publish()
{
  std::cout << name() << " is publishing " << std::endl;
}

void IntPublisher::reset()
{
  std::cout << name() << " is resetting" << std::endl;
}

} //publisher
} // alros
