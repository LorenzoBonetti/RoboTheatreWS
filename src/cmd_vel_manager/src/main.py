#!/usr/bin/env python
import actionlib
import rospy

from triskarone_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray


class CmdVelAction(object):
    # create messages that are used to publish feedback/result
    feedback = manual_move_baseFeedback()
    result = manual_move_baseResult()
    has_to_move = False
    counter = 0

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, manual_move_baseAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odometry_callback)
        rospy.loginfo("%s is started", rospy.get_name())
        self.position =[0,0,0]
        # self.orientation = Quaternion()
        self.orientation = [0, 0, 0, 0]

    def odometry_callback(self, data):
        self.position = [round(data.pose.pose.position.x, 2), round(data.pose.pose.position.y, 2),
                         round(data.pose.pose.position.z, 2)]
        self.orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                            data.pose.pose.orientation.w]

    def execute_cb(self, goal):

        movements = goal.goal.data
        self.counter = 0
        self.has_to_move = True
        has_finished = False
        while self.counter < len(movements):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                break
            array = [round(movements[self.counter], 2), round(movements[self.counter + 1], 2),
                     round(movements[self.counter + 2], 2)]
            print(array)
            linear_speed = movements[self.counter + 3]
            angular_speed = movements[self.counter + 4]
            x_start = self.position[0]
            y_start = self.position[1]
            euler = euler_from_quaternion(self.orientation)
            yaw_start = round(euler[2], 2)
            x_done = False
            y_done = False
            yaw_done = False
            while not (x_done and y_done and yaw_done):
                r=rospy.Rate(10)
                print("Position:", self.position)
                euler = euler_from_quaternion(self.orientation)
                actual_yaw = round(euler[2], 2)
                print("Orientation:", actual_yaw)
                data_to_send = Twist()
                if (self.position[0] - x_start) > array[0]:
                    data_to_send.linear.x = linear_speed
                elif (self.position[0]- x_start) < array[0]:
                    data_to_send.linear.x = -linear_speed
                else:
                    data_to_send.linear.x = 0
                    print("finito1")
                    x_done = True
                if (self.position[1]- y_start) > array[1]:
                    data_to_send.linear.y = linear_speed
                elif (self.position[1]- y_start) < array[1]:
                    data_to_send.linear.y = -linear_speed
                else:
                    data_to_send.linear.y = 0
                    print("finito2")
                    y_done=True
                if (actual_yaw - yaw_start) > array[2]:
                    data_to_send.angular.z = angular_speed
                elif (actual_yaw-yaw_start) < array[2]:
                    data_to_send.angular.z = -angular_speed
                else:
                    data_to_send.angular.z = 0
                    print("finito3")
                    yaw_done = True
                self.cmd_vel_pub.publish(data_to_send)
                r.sleep()
            self.counter = self.counter + 5

            # se abbiamo finito, passa al successivo
        self.result.response = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node("cmd_vel_manager")
    server = CmdVelAction(rospy.get_name())
    rospy.spin()
