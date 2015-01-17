#ifndef ALROS_BRIDGE_HPP
#define ALROS_BRIDGE_HPP

#include <vector>

#include "publishers/publisher.hpp"
namespace alros
{

class Bridge
{
public:
  void registerPublisher( const publisher::Publisher& pub );

  void publish( );

private:
  std::vector< publisher::Publisher > all_publisher_;
};

} // alros

#endif
