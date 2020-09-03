

#include <ros/ros.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
tf::TransformListener* tfListener;

class JoyTeleop
 {
	public:
		JoyTeleop();
		ros::NodeHandle nh;

	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
		void unsafeCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
		void updateParameters();
		void timerCallback(const ros::TimerEvent& e);
		void publishZeroMessage();

		geometry_msgs::Twist lastUnsafeTwistMsg_;

		double linearScale, angularScale;
		double maxLinearScale = 0, maxAngularScale = 0;
		int deadmanButton, linearXAxis, linearYAxis, angularAxis;
		bool canMove;
		bool isExit = false;
		ros::Subscriber joySub;
		ros::Subscriber unsafeCmdVelSub;
        ros::Subscriber pixelPosSub;
		ros::Publisher twistPub;
		ros::Publisher smoothTwistPub;
		ros::Timer timeout;
		ros::Time unsafe_cmd_vel_time;
};


