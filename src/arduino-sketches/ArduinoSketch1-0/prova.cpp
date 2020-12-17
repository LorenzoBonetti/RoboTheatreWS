#include <ros.h>
#include <geometry_msgs/Twist.h>

class DifferentialDriveRobot
{
    public:
        ros::NodeHandle nh;
        ros::Subscriber<geometry_msgs::Twist, DifferentialDriveRobot> sub;
        void ddr_callback(const geometry_msgs::Twist& msg){
          return;
        }

        DifferentialDriveRobot()
        : sub("/cmd_vel_mux/input/teleop", &DifferentialDriveRobot::ddr_callback, this)
        {  // Constructor
            nh.initNode();
            nh.subscribe(sub);
         }
};
