#!/usr/bin/env python
import actionlib
import rospy

from triskarone_msgs.msg import *
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Bool


class EyesManagerAction(object):
    # create messages that are used to publish feedback/result
    feedback = move_eyesFeedback()
    result = move_eyesResult()
    has_to_move=False
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, move_eyesAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        self.eyes_pub = rospy.Publisher('arduino/eyes', Int8MultiArray, queue_size=10)
        rospy.Subscriber("arduino/eyes_response", Bool, self.movement_callback)
        rospy.loginfo("%s is started", rospy.get_name())

    def movement_callback(self, data):
        self.has_to_move = True
        self.counter=self.counter+2

    def execute_cb(self, goal):
        print("Chiamata all'azione")
        r = rospy.Rate(10)
        movements = goal.goal.data
        print(movements)
        print(len(movements))
        self.counter = 0
        self.has_to_move = True
        has_finished = False
        while self.counter < len(movements):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                break
            self.feedback.current_movement = movements[self.counter]
            # publish the feedback
            self._as.publish_feedback(self.feedback)
            # move_eyes
            if self.has_to_move:
                array = [movements[self.counter], movements[self.counter + 1]]
                data_to_send = Int8MultiArray()
                data_to_send.data = array
                self.eyes_pub.publish(data_to_send)
            # se abbiamo finito, passa al successivo
        self.result.response = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node("audio_player_actionlib")
    server = EyesManagerAction(rospy.get_name())
    rospy.spin()
