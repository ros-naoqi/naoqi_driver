#ifndef PLAY_AUDIO_FILE_SERVICE_HPP
#define PLAY_AUDIO_FILE_SERVICE_HPP

#include <iostream>

#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <nao_interaction_msgs/AudioPlayback.h>
#include <qi/session.hpp>

namespace naoqi
{
  namespace service
  {

    class PlayAudioFileService
    {
    public:
      PlayAudioFileService( const std::string& name, const std::string& topic, const qi::SessionPtr& session );

      ~PlayAudioFileService(){};

      std::string name() const
      {
	return name_;
      }

      std::string topic() const
      {
	return topic_;
      }

      void reset( ros::NodeHandle& nh );

      bool callback( nao_interaction_msgs::AudioPlaybackRequest& req, nao_interaction_msgs::AudioPlaybackResponse& resp );


    private:
      const std::string name_;
      const std::string topic_;

      const qi::SessionPtr& session_;
      ros::ServiceServer service_;
    };

  } // service
} // naoqi
#endif
