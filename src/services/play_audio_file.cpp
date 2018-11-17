#include "play_audio_file.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
  namespace service
  {

    PlayAudioFileService::PlayAudioFileService( const std::string& name, const std::string& topic, const qi::SessionPtr& session )
      : name_(name),
	topic_(topic),
	session_(session)
    {}

    void PlayAudioFileService::reset( ros::NodeHandle& nh )
    {
      service_ = nh.advertiseService(topic_, &PlayAudioFileService::callback, this);
    }

    bool PlayAudioFileService::callback( nao_interaction_msgs::AudioPlaybackRequest& req, nao_interaction_msgs::AudioPlaybackResponse& resp )
    {
      helpers::driver::playAudioFile(session_, req);
      return true;
    }


  }
}
