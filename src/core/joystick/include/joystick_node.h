

#include <ros/ros.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <vector>

#define RUN_PERIOD_DEFAULT 100


/*Listens to the joystick's input
	If LB is pressed the robot is run autonomously, this node simply republishes the velocitites published by move_base
	If RB is pressed the manual commands are enabled
		-RB+X: increase linear scale
		-RB+A: decrease linear scale
		-RB+B: increase angular scale
		-RB+Y: decrease angular scale
		-RB+left analog: move linear
		-RB+right analog: move angular
	If right_cross_key is pressed, the robot goes to the next session
	If no button is pressed the robot is automatically stopped
*/
		
class JoyTeleop
 {
	public:
		JoyTeleop();
		std_msgs::Int8MultiArray eyes_msg;
		std_msgs::Int8MultiArray body_msg;
        ros::Publisher eyes_pub;
        ros::Publisher body_pub;

	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
		void moveBaseCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void cmd_managerCallback(const geometry_msgs::Twist::ConstPtr &msg);
        bool updateParameters();
		void publishZeroMessage();

		geometry_msgs::Twist lastTwistMsg;
		
		ros::NodeHandle nh;
		double linearScale, angularScale;
		double maxLinearScale, maxAngularScale;
		int enable_manual, autonomous_move, linearXAxis, linearYAxis, angularAxis;
		int LinearScaleUp, LinearScaleDown, AngularScaleUp, AngularScaleDown;
		int body_std,body_up_down,body_left_right;
		int eyes_std, eyes_up,  eyes_down, eyes_left, eyes_right, eyes_cross, eyes_divide;
		int NextSection;
		int MoveBaseRecovery;
		std::vector<int> eyes_data;
		std::vector<int> body_data;
	
		ros::Subscriber joySub;
		ros::Subscriber moveBaseCmdVelSub;
		ros::Subscriber cmd_vel_managerSub;
      
		ros::Publisher twistPub;
		ros::Publisher next_section_pub;
		ros::Publisher move_base_recovery_pub;
        
        
        
        
		ros::Time cmd_vel_time;
};


