

#include "joystick_node.h"

JoyTeleop::JoyTeleop() {
	joySub = nh.subscribe("/joy", 10, &JoyTeleop::joyCallback, this);
	unsafeCmdVelSub = nh.subscribe("/move_base/published_cmd_vel", 10, &JoyTeleop::unsafeCmdVelCallback, this);
	twistPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	smoothTwistPub = nh.advertise<geometry_msgs::Twist>("/raw_cmd_vel", 10);
	updateParameters();
}

void JoyTeleop::unsafeCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	unsafe_cmd_vel_time = ros::Time::now();
	lastUnsafeTwistMsg_ = *msg;
}

void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {

	// process and publish
	geometry_msgs::Twist twistMsg;
	if (msg->buttons[4]){							// if autonomous publishes the commands of move_base directly on /cmd_vel
		ros::Time now = ros::Time::now();
		ros::Duration time_diff = now - unsafe_cmd_vel_time;
		if (time_diff.toSec() < 0.5){
			twistMsg = lastUnsafeTwistMsg_;
			twistPub.publish(twistMsg);
		}else{
			ROS_DEBUG_STREAM("unsafe/cmd_vel too old... skipping..  Time diff:" << time_diff.toSec());
		}
	}else{
		if (msg->buttons[deadmanButton]){
			if (msg->buttons[3]){
				ROS_DEBUG_STREAM("Increasing linearScale by 0.5\%...");
				linearScale += 0.01;//linearScale * 0.05;
			}else if (msg->buttons[2]){
				ROS_DEBUG_STREAM("Decreasing linearScale by 0.5\%...");
				linearScale -= 0.01;// linearScale * 0.05;
			}else if (msg->buttons[0]){
				ROS_DEBUG_STREAM("Increasing angularScale by 0.5\%...");
				angularScale += 0.01;// angularScale * 0.05;
			}else if (msg->buttons[1]){
				ROS_DEBUG_STREAM("Decreasing linearScale by 0.5\%...");
				angularScale -= 0.01;// angularScale * 0.05;
			}
			
			
			if (linearScale >= maxLinearScale){
				linearScale = maxLinearScale;
			}
		
			if (linearScale <= 0){
			linearScale = 0;
			}
		
			if (angularScale >= maxAngularScale){
			angularScale = maxAngularScale;
			}
		
			if (angularScale <= 0){
			angularScale = 0;
			}
		
			twistMsg.linear.x = linearScale*msg->axes[linearXAxis];
			twistMsg.linear.y = linearScale*msg->axes[linearYAxis];
			twistMsg.angular.z = angularScale*msg->axes[angularAxis];
			// ROLLBACK
			// twistPub.publish(twistMsg);
			smoothTwistPub.publish(twistMsg);
		}else
			publishZeroMessage();
	}
}

void JoyTeleop::updateParameters() {
	// update the parameters for processing the joystick messages
	nh.getParam("max_linear_scale", maxLinearScale);

	nh.getParam("max_angular_scale", maxAngularScale);

	if (!nh.getParam("linear_scale", linearScale))
		linearScale = 1;

	if (!nh.getParam("angular_scale", angularScale))
		angularScale = 0.5;

	if (!nh.getParam("button_RB", deadmanButton))
		deadmanButton = 5;

	if (!nh.getParam("button_A", linearXAxis))
		linearXAxis = 1;

	if (!nh.getParam("button_X", linearYAxis))
		linearYAxis = 0;

	if (!nh.getParam("button_B", angularAxis))
		angularAxis = 2;
}

void JoyTeleop::timerCallback(const ros::TimerEvent& e) {
	publishZeroMessage();
}

void JoyTeleop::publishZeroMessage() {
	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.angular.z = 0;
	twistPub.publish(msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_node");
	JoyTeleop teleop;
	
	double maxLinearScale, maxAngularScale;
	
	if (!teleop.nh.getParam("/max_linear_scale",maxLinearScale) || !teleop.nh.getParam("/max_angular_scale", maxAngularScale)){
	 	ROS_FATAL("max_linear_scale and max_angular_scale are required!");
	 	ros::shutdown();
	 	return -1;
	 }

	tfListener = new tf::TransformListener();
  	// Loop at 100Hz until the node is shutdown.
    ros::Rate rate(100);
	
	while(ros::ok()){
       
		ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep() ;
    }

	return 0;
}
