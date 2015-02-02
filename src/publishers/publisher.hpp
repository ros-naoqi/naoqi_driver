#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

namespace alros
{
namespace publisher
{

class Publisher
{

public:
  template<typename T>
  Publisher( T pub ):
    pubPtr_( boost::make_shared<PublisherModel<T> >(pub) )
  {};

  void publish()
  {
    pubPtr_->publish();
  }

  bool isInitialized() const
  {
    return pubPtr_->isInitialized();
  }

  bool isSubscribed() const
  {
    return pubPtr_->isSubscribed();
  }

  void reset( ros::NodeHandle& nh )
  {
    pubPtr_->reset( nh );
  }

  std::string name() const
  {
    return pubPtr_->name();
  }

  float frequency() const
  {
    return pubPtr_->frequency();
  }

  std::string topic() const
  {
    return pubPtr_->topic();
  }

  friend bool operator==( const Publisher& lhs, const Publisher& rhs )
  {
    // decision made for OR-comparison since we want to be more restrictive
    if ( lhs.name() == rhs.name() || lhs.topic() == rhs.topic() )
      return true;
    return false;
  }

private:

  /**
  * BASE concept struct
  */
  struct PublisherConcept
  {
    virtual ~PublisherConcept(){};
    virtual void publish() = 0;
    virtual bool isInitialized() const = 0;
    virtual bool isSubscribed() const = 0;
    virtual void reset( ros::NodeHandle& nh ) = 0;
    virtual std::string name() const = 0;
    virtual std::string topic() const = 0;
    virtual float frequency() const = 0;
  };


  /**
  * templated instances of base concept
  */
  template<typename T>
  struct PublisherModel : public PublisherConcept
  {
    PublisherModel( const T& other ):
      publisher_( other )
    {}

    std::string name() const
    {
      return publisher_.name();
    }

    std::string topic() const
    {
      return publisher_.topic();
    }

    float frequency() const
    {
      return publisher_.frequency();
    }

    void publish()
    {
      publisher_.publish();
    }

    bool isInitialized() const
    {
      return publisher_.isInitialized();
    }

    bool isSubscribed() const
    {
      return publisher_.isSubscribed();
    }

    void reset( ros::NodeHandle& nh )
    {
      publisher_.reset( nh );
    }

    T publisher_;
  };

  boost::shared_ptr<PublisherConcept> pubPtr_;

}; // class publisher

} //publisher
} //alros

#endif
