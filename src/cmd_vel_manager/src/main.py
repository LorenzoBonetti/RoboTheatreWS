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
from math import sin, cos, pi


class CmdVelAction(object):
    # create messages that are used to publish feedback/result
    feedback = manual_move_baseFeedback()
    result = manual_move_baseResult()

    def __init__(self, name):
        self._action_name = name

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('vel', Twist, self.vel_callback)

        self.last_time = rospy.get_rostime()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.counter = 0

        self._as = actionlib.SimpleActionServer(self._action_name, manual_move_baseAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()

        rospy.loginfo("%s is started", rospy.get_name())

    def vel_callback(self, data):
        vx = data.linear.x
        vy = data.liner.y
        vth = data.angular.z
        current_time = rospy.get_rostime()
        dt = (current_time - self.last_time).to_sec()
        delta_x = (vx * cos(self.th) - vy * sin(self.th)) * dt
        delta_y = (vx * sin(self.th) + vy * cos(self.th)) * dt
        delta_th = vth * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        self.last_time = current_time

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
            x_speed = movements[self.counter + 3]
            y_speed = movements[self.counter + 4]
            angular_speed = movements[self.counter + 5]

            self.x = 0.0
            self.y = 0.0
            self.th = 0.0

            x_start = self.x
            y_start = self.y
            th_start = self.th

            x_done = False
            y_done = False
            yaw_done = False
            while not (x_done and y_done and yaw_done):
                r = rospy.Rate(10)
                data_to_send = Twist()
                if not x_done:
                    if abs(self.x - x_start) < array[0]:
                        data_to_send.linear.x = x_speed
                    else:
                        data_to_send.linear.x = 0.0
                        print("finito1")
                        x_done = True
                if not y_done:
                    if abs(self.y - y_start) < array[1]:
                        data_to_send.linear.y = y_speed
                    else:
                        data_to_send.linear.y = 0.0
                        print("finito2")
                        y_done = True
                if not yaw_done:
                    print("Mi devo muovere di", array[2])
                    if abs(self.th - th_start) < array[2]:
                        data_to_send.angular.z = angular_speed
                    else:
                        data_to_send.angular.z = 0.0
                        print("finito3")
                        yaw_done = True
                self.cmd_vel_pub.publish(data_to_send)
                r.sleep()

            self.counter = self.counter + 6

            # se abbiamo finito, passa al successivo
        self.result.response = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node("cmd_vel_manager")
    server = CmdVelAction(rospy.get_name())
    rospy.spin()
