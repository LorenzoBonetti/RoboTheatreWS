
#include "joystick_node.h"

JoyTeleop::JoyTeleop() {
	joySub = nh.subscribe("/joy", 10, &JoyTeleop::joyCallback, this);
	moveBaseCmdVelSub = nh.subscribe("/move_base/published_cmd_vel", 10, &JoyTeleop::moveBaseCmdVelCallback, this);
    cmd_vel_managerSub = nh.subscribe("/cmd_vel_manager/cmd_vel", 10, &JoyTeleop::moveBaseCmdVelCallback, this);
	twistPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	next_section_pub=nh.advertise<std_msgs::Bool>("next_section", 1000);
	move_base_recovery_pub=nh.advertise<std_msgs::Int8>("move_base_recovery", 1000);
	body_pub=nh.advertise<std_msgs::Int8MultiArray>("arduino/body",1000);
	eyes_pub=nh.advertise<std_msgs::Int8MultiArray>("arduino/eyes",1000)
	if(!updateParameters()){
		ROS_FATAL("joystick parameters required");
	 	ros::shutdown();
	}
}

void JoyTeleop::moveBaseCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	cmd_vel_time = ros::Time::now();
	lastTwistMsg= *msg;
}
void JoyTeleop::cmd_managerCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    cmd_vel_time = ros::Time::now();
    lastTwistMsg= *msg;
}


void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {

	// process and publish
	geometry_msgs::Twist twistMsg;
	std_msgs::Bool next_sectionMsg;
	std_msgs::Int8 move_base_recoveryMsg;
	std_msgs::Int8MultiArray move_eyes_msg;
    std_msgs::Int8MultiArray move_body_msg;
	if(msg->buttons[eyes_std]){
	    move_eyes_msg.data[0]=0;
	    move_eyes_msg.data[1]=3;
	    eyes_pub.publish(move_eyes_msg);
	}
    if(msg->buttons[eyes_left]){
        move_eyes_msg.data[0]=1;
        move_eyes_msg.data[1]=3;
        eyes_pub.publish(move_eyes_msg);
    }
    if(msg->buttons[eyes_right]){
        move_eyes_msg.data[0]=2;
        move_eyes_msg.data[1]=3;
        eyes_pub.publish(move_eyes_msg);
    }
    if(msg->buttons[eyes_up]){
        move_eyes_msg.data[0]=4;
        move_eyes_msg.data[1]=3;
        eyes_pub.publish(move_eyes_msg);
    }

    if(msg->buttons[eyes_down]){
        move_eyes_msg.data[0]=3;
        move_eyes_msg.data[1]=3;
        eyes_pub.publish(move_eyes_msg);
    }
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
		ros::Duration time_diff = now - cmd_vel_time;
		if (time_diff.toSec() < 0.5){
			twistMsg = lastTwistMsg;
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
	
	if (!nh.getParam("button_start", AngularScaleUp))
		return false;

	if (!nh.getParam("button_back", AngularScaleDown))
		return false;

	if (!nh.getParam("button_LT", LinearScaleDown))
		return false;

	if (!nh.getParam("button_RT", LinearScaleUp))
		return false;
	if (!nh.getParam("cross_key_left_right", body_left_right))
		return false;
	if (!nh.getParam("cross_key_up_down", body_up_down))
	    return false;
    if (!nh.getParam("button_stick_left", body_std))
        return false;
    if (!nh.getParam("button_stick_right", eyes_std))
        return false;
    if (!nh.getParam("button_X", eyes_left))
        return false;
    if (!nh.getParam("button_B", eyes_right))
        return false;
    if (!nh.getParam("button_Y", eyes_up))
        return false;
    if (!nh.getParam("button_A", eyes_down))
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
