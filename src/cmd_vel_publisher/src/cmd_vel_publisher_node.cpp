
#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.1

#define NAME_OF_THIS_NODE "cmd_vel_publisher_node"

#define MAX_ACTIONS 1
#define ACTION_TIME 3//in seconds


#include "geometry_msgs/Twist.h"

class Cmd_vel_publisher{

  private: 
    ros::NodeHandle Handle;
    ros::Publisher Publisher;
    geometry_msgs::Twist msg;
    int action_counter;
    ros::Timer ActionTimer;
   

    void Action_Timer_Callback(const ros::TimerEvent& event);
    void PeriodicTask(void);
   
  public:
    double RunPeriod;

    void Prepare(void);
 
    void RunContinuously(void);
 
    
    void RunPeriodically(float Period);
 
    
    void Shutdown(void);
    
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void Cmd_vel_publisher::Prepare(void)
{
  RunPeriod = RUN_PERIOD_DEFAULT;
  
  action_counter=0;

  Publisher = Handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ActionTimer = Handle.createTimer(ros::Duration(ACTION_TIME), &Cmd_vel_publisher::Action_Timer_Callback, this,true);
  
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void Cmd_vel_publisher::RunPeriodically(float Period)
{
  ros::Rate LoopRate(1.0/Period);
  
  ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  
  while (ros::ok())
  {
    PeriodicTask();
   
     
    ros::spinOnce();

    ROS_INFO("Current Action :%d",action_counter);
    //LoopRate.sleep();  //removed because spins at max speed
  }
}


void Cmd_vel_publisher::Shutdown(void)
{
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
 
}

void Cmd_vel_publisher::Action_Timer_Callback(const ros::TimerEvent& event)
{
	action_counter++;
}

void Cmd_vel_publisher::PeriodicTask(void)
{
	 switch(action_counter){
		case 0: //go straight
			msg.linear.x=0.75;
			msg.linear.y=0;
			msg.linear.z=0;
			
			msg.angular.x=0;
			msg.angular.y=0;
			msg.angular.z=0;
			
			Publisher.publish(msg);
			break;
		default://stop
			msg.linear.x=0;
			msg.linear.y=0;
			msg.linear.z=0;
			
			msg.angular.x=0;
			msg.angular.y=0;
			msg.angular.z=0;
			Publisher.publish(msg);
			break;
	}
	
	

	
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);

  
  Cmd_vel_publisher MyNode;
   
  MyNode.Prepare();
  
  MyNode.RunPeriodically(MyNode.RunPeriod);
  
  MyNode.Shutdown();
  
  return (0);
}



