#ifndef HELPERS_HPP
#define HELPERS_HPP

#include "publishers/publisher.hpp"

namespace alros
{
namespace helpers
{

inline bool hasSameName( const publisher::Publisher& first, const publisher::Publisher& second )
{
  if ( first.name() == second.name() )
    return true;
  else
    return false;
}



} //helpers
} // alros

#endif
