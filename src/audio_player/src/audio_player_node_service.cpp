
#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "audio_player_node"

#include "audio_player/audio_player.h"
//-----------------------------------------------------------------
//-----------------------------------------------------------------

class AudioPlayer
{
private:
    ros::NodeHandle Handle;
    ros::ServiceServer Service;

    std::string audio_folder_path;

    bool PlayAudio(audio_player::audio_player::Request &Req, audio_player::audio_player::Response &Res);

public:
    void Prepare(void);

    void RunContinuously(void);

    void Shutdown(void);

};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void AudioPlayer::Prepare(void) {
    std::string FullParamName="audio_folder_path";
    //FullParamName = Handle.getNamespace()+"/audio_folder_path";
    Service = Handle.advertiseService("audio_player_service", &AudioPlayer::PlayAudio, this);
    ROS_INFO("ROS service %s available (provided by node %s).", "audio_player_service",
             ros::this_node::getName().c_str());


    if (Handle.getParam(FullParamName, audio_folder_path)) {
        ROS_INFO("Node %s: retrieved parameter %s.",
                 ros::this_node::getName().c_str(), FullParamName.c_str());
    } else {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.",
                  ros::this_node::getName().c_str(), FullParamName.c_str());
    }


    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void AudioPlayer::RunContinuously(void) {
    ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());

    ros::spin();
}




void AudioPlayer::Shutdown(void) {
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}


bool AudioPlayer::PlayAudio(audio_player::audio_player::Request  &Req,  audio_player::audio_player::Response &Res)
{
    std::string command;
    std::stringstream ss;
    ss << "canberra-gtk-play -f " <<audio_folder_path<< Req.file_name;

    command=ss.str();

    int n=command.length();
    char system_command[n+1];
    strcpy(system_command,command.c_str());
    system(system_command);
    ROS_INFO("playing %s", system_command);
}





//-----------------------------------------------------------------
//-----------------------------------------------------------------


int main(int argc, char **argv) {

    ros::init(argc, argv, NAME_OF_THIS_NODE);
    AudioPlayer MyNode;
    MyNode.Prepare();
    MyNode.RunContinuously();

    MyNode.Shutdown();

    return (0);
}


