
#include "joystick_node.h"

JoyTeleop::JoyTeleop() {
	joySub = nh.subscribe("/joy", 10, &JoyTeleop::joyCallback, this);
	moveBaseCmdVelSub = nh.subscribe("/move_base/published_cmd_vel", 10, &JoyTeleop::moveBaseCmdVelCallback, this);
	twistPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	next_section_pub=nh.advertise<std_msgs::Bool>("next_section", 1000);
	move_base_recovery_pub=nh.advertise<std_msgs::Int8>("move_base_recovery", 1000);
	if(!updateParameters()){
		ROS_FATAL("joystick parameters required");
	 	ros::shutdown();
	}
}

void JoyTeleop::moveBaseCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	move_base_cmd_vel_time = ros::Time::now();
	lastMoveBaseTwistMsg= *msg;
}

void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {

	// process and publish
	geometry_msgs::Twist twistMsg;
	std_msgs::Bool next_sectionMsg;
	std_msgs::Int8 move_base_recoveryMsg;
	if (msg->axes[NextSection]==-1.0){ //right key pressed
	    next_sectionMsg.data=true;
		next_section_pub.publish(next_sectionMsg);
	}
	if(msg->axes[MoveBaseRecovery]==1.0){//up key pressed
	    move_base_recoveryMsg.data=1;
	    move_base_recovery_pub.publish(move_base_recoveryMsg);
	}
    if(msg->axes[MoveBaseRecovery]==-1.0){//down key pressed
        move_base_recoveryMsg.data=2;
        move_base_recovery_pub.publish(move_base_recoveryMsg);
    }
	if (msg->buttons[autonomous_move]){							// if autonomous publishes the commands of move_base directly on /cmd_vel
		ros::Time now = ros::Time::now();
		ros::Duration time_diff = now - move_base_cmd_vel_time;
		if (time_diff.toSec() < 0.5){
			twistMsg = lastMoveBaseTwistMsg;
			twistPub.publish(twistMsg);
		}else{
			ROS_DEBUG_STREAM("move_base/published_cmd_vel too old... skipping..  Time diff:" << time_diff.toSec());
		}
	}else{
		if (msg->buttons[enable_manual]){
			if (msg->buttons[LinearScaleUp]){
				ROS_DEBUG_STREAM("Increasing linearScale by 0.5\%...");
				linearScale += 0.01;
			}else if (msg->buttons[LinearScaleDown]){
				ROS_DEBUG_STREAM("Decreasing linearScale by 0.5\%...");
				linearScale -= 0.01;
			}else if (msg->buttons[AngularScaleUp]){
				ROS_DEBUG_STREAM("Increasing angularScale by 0.5\%...");
				angularScale += 0.01;
			}else if (msg->buttons[AngularScaleDown]){
				ROS_DEBUG_STREAM("Decreasing linearScale by 0.5\%...");
				angularScale -= 0.01;
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
		
			twistPub.publish(twistMsg);
		
		}else
			publishZeroMessage();
	}
}

bool JoyTeleop::updateParameters() {
	// update the parameters for processing the joystick messages
	if(!nh.getParam("max_linear_scale", maxLinearScale))
		return false;

	if(!nh.getParam("max_angular_scale", maxAngularScale))
		return false;

	if (!nh.getParam("linear_scale", linearScale))
		return false;

	if (!nh.getParam("angular_scale", angularScale))
		return false;

	if (!nh.getParam("button_RB", enable_manual))
		return false;
	
	if (!nh.getParam("button_LB", autonomous_move))
		return false;

	if (!nh.getParam("up_down_axis_stick_left", linearXAxis))
		return false;

	if (!nh.getParam("left_right_axis_stick_left", linearYAxis))
		return false;

	if (!nh.getParam("left_right_axis_stick_right", angularAxis))
		return false;
	
	if (!nh.getParam("button_X", AngularScaleUp))
		return false;

	if (!nh.getParam("button_A", AngularScaleDown))
		return false;

	if (!nh.getParam("button_B", LinearScaleDown))
		return false;

	if (!nh.getParam("button_Y", LinearScaleUp))
		return false;
	if (!nh.getParam("cross_key_left_right", NextSection))
		return false;
	if (!nh.getParam("cross_key_up_down", MoveBaseRecovery))
	    return false;
	return true;
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
	
    ros::Rate rate(RUN_PERIOD_DEFAULT);
	
	while(ros::ok()){
  
		ros::spinOnce();
        rate.sleep() ;
    }

	return 0;
}
