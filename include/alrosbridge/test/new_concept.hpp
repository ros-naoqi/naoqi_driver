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

#ifndef NEW_CONCEPT_HPP
#define NEW_CONCEPT_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

class Object;

namespace alros
{
namespace concept
{


/**
* @brief Concept concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible Concept instance has to implement the virtual functions mentioned in the concept
*/
class New
{

public:

  /**
  * @brief Constructor for concept interface
  */
  template<typename T>
  New( const T& object ):
    objPtr_( boost::make_shared<Model<T> >(object) )
  {}

  /**
  * @brief call the sole method of object
  * @return the result of this method
  */
  std::string method() const
  {
    return objPtr_->method();
  }

  friend bool operator==( const New& lhs, const New& rhs )
  {
    // decision made for OR-comparison since we want to be more restrictive
    if ( lhs.method() != rhs.method() )
      return false;
    return true;
  }

  friend bool operator==( const boost::shared_ptr<New>& lhs, const boost::shared_ptr<New>& rhs )
  {
    return operator==( *lhs, *rhs );
  }

private:

  /**
  * BASE concept struct
  */
  struct Concept
  {
    virtual ~Concept(){}
    virtual std::string method() const = 0;
  };


  /**
  * templated instances of base concept
  */
  template<typename T>
  struct Model : public Concept
  {
    Model( const T& other ):
      concept_( other )
    {}

    std::string method() const
    {
      return concept_->method();
    }

    T concept_;
  };

  boost::shared_ptr<Concept> objPtr_;

}; // class concept

} //concept
} //alros

#endif
