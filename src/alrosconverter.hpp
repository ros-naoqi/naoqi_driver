#include <string>

class ALRosConverter{
  public:
    ALRosConverter( const std::string& ip );

    /**
    * callable with qicli call
    */
    std::string getMasterURI() const;

    /**
    * callable with qicli call
    */
    void setMasterURI( const std::string& uri ) const;
};
