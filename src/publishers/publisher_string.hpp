#ifndef STRING_PUBLISHER_HPP
#define STRING_PUBLISHER_HPP

#include "publisher_base.hpp"

namespace alros
{
namespace publisher
{

class StringPublisher : public BasePublisher
{

public:
  StringPublisher( const std::string& name, const std::string& topic );

  void publish();

  void reset();

}; // class

} //publisher
} // alros

#endif
