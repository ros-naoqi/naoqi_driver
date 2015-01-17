#ifndef INT_PUBLISHER_HPP
#define INT_PUBLISHER_HPP

#include "publisher_base.hpp"

namespace alros
{
namespace publisher
{

class IntPublisher : public BasePublisher
{

public:
  IntPublisher( const std::string& name, const std::string& topic );

  void publish();

  void reset();

}; // class

} //publisher
} // alros

#endif
