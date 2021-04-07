
#include "joystick_node.h"

JoyTeleop::JoyTeleop() {
	joySub = nh.subscribe("/joy", 10, &JoyTeleop::joyCallback, this);
	moveBaseCmdVelSub = nh.subscribe("/move_base/published_cmd_vel", 10, &JoyTeleop::moveBaseCmdVelCallback, this);
    cmd_vel_managerSub = nh.subscribe("/cmd_vel_manager/cmd_vel", 10, &JoyTeleop::moveBaseCmdVelCallback, this);
	twistPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	next_section_pub=nh.advertise<std_msgs::Bool>("next_section", 1000);
	move_base_recovery_pub=nh.advertise<std_msgs::Int8>("move_base_recovery", 1000);
	
  	eyes_pub=nh.advertise<std_msgs::Int8MultiArray>("arduino/eyes", 1000);
  	eyes_data=std::vector<int>(2);
  	eyes_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	eyes_msg.layout.dim[0].size = eyes_data.size();
	eyes_msg.layout.dim[0].stride = 1;
	eyes_msg.layout.dim[0].label = "x";
	
	body_pub=nh.advertise<std_msgs::Int8MultiArray>("arduino/body", 1000);
  	body_data=std::vector<int>(2);
  	body_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	body_msg.layout.dim[0].size = body_data.size();
	body_msg.layout.dim[0].stride = 1;
	body_msg.layout.dim[0].label = "y";
	if(!updateParameters()){
		ROS_FATAL("joystick parameters required");
	 	ros::shutdown();
	}
	printf("AAAA %d", body_up_down);
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
	if(msg->buttons[eyes_std]){
		eyes_data.clear();
	    eyes_data.insert(eyes_data.begin(),0);
	    eyes_data.insert(eyes_data.begin()+1,3);
	    eyes_msg.data.clear();
		eyes_msg.data.insert(eyes_msg.data.end(), eyes_data.begin(), eyes_data.end());
		eyes_pub.publish(eyes_msg);
	}
    if(msg->buttons[eyes_left]){
    	eyes_data.clear();
        eyes_data.insert(eyes_data.begin(),2);
	    eyes_data.insert(eyes_data.begin()+1,3);
	    eyes_msg.data.clear();
		eyes_msg.data.insert(eyes_msg.data.end(), eyes_data.begin(), eyes_data.end());
		eyes_pub.publish(eyes_msg);
    }
    if(msg->buttons[eyes_right]){
    	eyes_data.clear();
        eyes_data.insert(eyes_data.begin(),1);
	    eyes_data.insert(eyes_data.begin()+1,3);
	    eyes_msg.data.clear();
		eyes_msg.data.insert(eyes_msg.data.end(), eyes_data.begin(), eyes_data.end());
		eyes_pub.publish(eyes_msg);
    }
    if(msg->buttons[eyes_up]){
    	eyes_data.clear();
         eyes_data.insert(eyes_data.begin(),4);
	    eyes_data.insert(eyes_data.begin()+1,3);
	    eyes_msg.data.clear();
		eyes_msg.data.insert(eyes_msg.data.end(), eyes_data.begin(), eyes_data.end());
		eyes_pub.publish(eyes_msg);
    }

    if(msg->buttons[eyes_down]){
    	eyes_data.clear();
          eyes_data.insert(eyes_data.begin(),3);
	    eyes_data.insert(eyes_data.begin()+1,3);
	    eyes_msg.data.clear();
		eyes_msg.data.insert(eyes_msg.data.end(), eyes_data.begin(), eyes_data.end());
		eyes_pub.publish(eyes_msg);
    }
    
    //BODY
    if(msg->buttons[body_std]){
    	body_data.clear();
	    body_data.insert(body_data.begin(),0);
	    body_data.insert(body_data.begin()+1,3);
	    body_msg.data.clear();
		body_msg.data.insert(body_msg.data.end(), body_data.begin(), body_data.end());
		body_pub.publish(body_msg);
    }
    if(msg->axes[body_left_right]==-1.0){
		body_data.clear();
	    body_data.insert(body_data.begin(),2);
	    body_data.insert(body_data.begin()+1,3);
	    body_msg.data.clear();
		body_msg.data.insert(body_msg.data.end(), body_data.begin(), body_data.end());
		body_pub.publish(body_msg);
	}
    if(msg->axes[body_left_right]==1.0){
		body_data.clear();
	    body_data.insert(body_data.begin(),3);
	    body_data.insert(body_data.begin()+1,3);
	    body_msg.data.clear();
		body_msg.data.insert(body_msg.data.end(), body_data.begin(), body_data.end());
		body_pub.publish(body_msg);
	}if(msg->axes[body_up_down]==1.0){
		body_data.clear();
	    body_data.insert(body_data.begin(),1);
	    body_data.insert(body_data.begin()+1,3);
	    body_msg.data.clear();
		body_msg.data.insert(body_msg.data.end(), body_data.begin(), body_data.end());
		body_pub.publish(body_msg);
	}if(msg->axes[body_up_down]==-1.0){
		body_data.clear();
	    body_data.insert(body_data.begin(),4);
	    body_data.insert(body_data.begin()+1,3);
	    body_msg.data.clear();
		body_msg.data.insert(body_msg.data.end(), body_data.begin(), body_data.end());
		body_pub.publish(body_msg);
	}
  
	/*if (msg->axes[NextSection]==-1.0){ //right key pressed
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
    }*/
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
				linearScale += 0.01;
				ROS_INFO("Increasing linearScale, now %f", linearScale);
			}else if (msg->buttons[LinearScaleDown]){
				linearScale -= 0.01;
				ROS_INFO("Decreasing linearScale, now %f", linearScale);
			}else if (msg->buttons[AngularScaleUp]){
				angularScale += 0.01;
				ROS_INFO("Increasing angularScale, now %f", angularScale);
			}else if (msg->buttons[AngularScaleDown]){
				angularScale -= 0.01;
				ROS_INFO("Decreasing angularScale, now %f", angularScale);
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
