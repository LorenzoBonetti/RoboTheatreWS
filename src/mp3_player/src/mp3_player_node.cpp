//
// Created by lorenzo on 14/09/20.
//

#include "../include/mp3_player_node.h"
#include "ros/ros.h"


#define RUN_PERIOD_DEFAULT 0.1

#define NAME_OF_THIS_NODE "mp3_player_node"


#include "std_msgs/String.h"

class Mp3Player
/* this class template includes all the elements needed to define
 * a basic ROS node */
{
private:
    ros::NodeHandle Handle;
    ros::Subscriber Subscriber;
    ros::Publisher Publisher;

    void MessageCallback(const std_msgs::String::ConstPtr& msg);




public:
    double RunPeriod;

    void Prepare(void);


    void RunContinuously(void);


    void RunPeriodically(float Period);


    void Shutdown(void);

};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void Mp3Player::Prepare(void)
{
    RunPeriod = RUN_PERIOD_DEFAULT;

    Subscriber = Handle.subscribe("Mp3Player/audio_file", 1000, &Mp3Player::MessageCallback, this);

    Publisher = Handle.advertise<std_msgs::String>("name_of_the_topic", 1000);


    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void Mp3Player::RunContinuously(void)
{
    ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());

    ros::spin();

    /* From ROS documentation:
     * "ros::spin() will not return until the node has been
     * shutdown, either through a call to ros::shutdown() or a
     * Ctrl-C." */
}




void Mp3Player::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());

}



void Mp3Player::MessageCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string command;
    std::stringstream ss;
    ss << "canberra-gtk-play -f " <<"/home/lorenzo/RoboTheatreWS/src/mp3_player/audio_files/"<< msg->data;
    command=ss.str();

    int n=command.length();
    char system_command[n+1];
    strcpy(system_command,command.c_str());
    system(system_command);
    ROS_INFO("playing %s", system_command);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    /* NOTE: the call to ros::init should be the FIRST statement of
     * the 'main' block. You get compilation errors if you use any
     * part of the ROS system before that statement. */

    /* NOTE. If this node is launched with rosrun, a new node
     * gets added to the running ROS system, the name of which is
     * the string assigned to NAME_OF_THIS_NODE. If the node is
     * launched with roslaunch, it is possible that this choice of
     * name has been overridden, and that the newly added node takes
     * different name than NAME_OF_THIS_NODE. This happens when the
     * <node> statement in the launchfile used to launch the node
     * specifies a name for the node (which is not mandatory). */

    Mp3Player MyNode;

    MyNode.Prepare();

    MyNode.RunContinuously();


    MyNode.Shutdown();

    return (0);
}


