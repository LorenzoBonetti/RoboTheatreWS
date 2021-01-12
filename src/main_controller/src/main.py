#!/usr/bin/env python
# license removed for brevity
import os
import json
import actionlib
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dynamic_reconfigure.server import Server
from main_controller.cfg import main_controllerConfig

import rospy
# import std_msgs
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Bool
from triskarone_msgs.msg import *


class main_controller():
    def __init__(self):
        # change working directory
        os.chdir(os.path.dirname(os.path.abspath(__file__)))
        os.chdir("..")
        rospy.init_node('main_controller', anonymous=True)
        # initialize reconfigure
        srv = Server(main_controllerConfig, self.reconfigure_callback)
        # get paramters
        self.script_path = rospy.get_param("script_path")
        self.move_base_recovery = rospy.get_param("move_base_recovery")
        self.section_number = rospy.get_param("section_to_start")
        # initialize publisher and clients
        self.eyes_pub = rospy.Publisher('arduino/eyes', Int8MultiArray, queue_size=10)
        self.body_pub = rospy.Publisher('arduino/body', Int8MultiArray, queue_size=10)
        rospy.Subscriber("next_section", Bool, self.next_section_callback)
        self.enable_after_command = True
        self.trigger_ok = False
        self.speech_client = actionlib.SimpleActionClient('speech_monitor', triskarone_msgs.msg.speech_monitorAction)
        self.audio_client = actionlib.SimpleActionClient('audio_player_actionlib', triskarone_msgs.msg.play_audioAction)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.eyes_manager_client = actionlib.SimpleActionClient('eyes_manager', triskarone_msgs.msg.move_eyesAction)
        self.body_manager_client = actionlib.SimpleActionClient('body_manager', triskarone_msgs.msg.move_bodyAction)
        # init node

        # wait for servers to start
        self.speech_client.wait_for_server()
        self.audio_client.wait_for_server()
        self.eyes_manager_client.wait_for_server()
        self.body_manager_client.wait_for_server()
        r = rospy.Rate(0.5)
        r.sleep()  # sleep for 2 seconds before starting

        # run
        self.run()

    def run(self):
        f = open(self.script_path)
        data = json.load(f)
        while not rospy.is_shutdown():
            rospy.loginfo("Starting section number %d", self.section_number)
            trigger, trigger_data = self.read_trigger(data, self.section_number)
            self.wait_for_trigger(trigger, trigger_data)
            actions = self.read_actions(data, self.section_number)
            self.handle_actions(actions)
            self.check_actions_status(actions)
            self.section_number = self.section_number + 1

    def reconfigure_callback(self, config, level):
        rospy.loginfo("section_number set to %d", config.section_number)
        self.section_number = config.section_number
        return config

    def next_section_callback(self, data):
        if self.enable_after_command:
            self.trigger_ok = True
            self.enable_after_command = False
            rospy.loginfo("Go to next session")

    def read_trigger(self, data, section_number):
        section = 'section' + str(section_number)
        trigger = data[section]['trigger']
        trigger_data = data[section]['trigger_data']
        return trigger, trigger_data

    def read_actions(slef, data, section_number):
        section = 'section' + str(section_number)
        actions = data[section]['actions']
        return actions

    def wait_for_trigger(self, trigger, trigger_data):
        if trigger == "after_speech":
            goal = triskarone_msgs.msg.speech_monitorGoal(wait_time=trigger_data)
            rospy.loginfo("Waiting for others to speak")
            self.speech_client.send_goal(goal)
            self.speech_client.wait_for_result()
            return
        if trigger == "after_precedent":
            return
        if trigger == "after_command":
            self.enable_after_command = True;
            self.trigger_ok = False
            rospy.loginfo("Waiting for joystick command")
            while not self.trigger_ok:
                continue
            return

    def check_actions_status(self, actions):
        has_to_speak = False
        move_base = False
        move_base_error = False
        move_eyes = False
        move_body = False
        if "speak" in actions:
            file_to_play = actions['speak']
            has_to_speak = True
        if "move_base" in actions:
            move_base = True
        if "move_eyes" in actions:
            move_eyes = True
        if "move_body" in actions:
            move_body = True

        while has_to_speak or move_base or move_eyes or move_body:
            if self.audio_client.get_state() == GoalStatus.SUCCEEDED:
                # rospy.loginfo("audio_client has finished")
                has_to_speak = False
            if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
                move_base = False
                # rospy.loginfo("Correctly moved to the desired position")
            if self.move_base_client.get_state() == GoalStatus.ABORTED:
                move_base = False
                move_base_error = True
            if self.eyes_manager_client.get_state() == GoalStatus.SUCCEEDED:
                move_eyes = False
            if self.body_manager_client.get_state() == GoalStatus.SUCCEEDED:
                move_body = False
            continue
        if move_base_error:
            rospy.loginfo("Failed to get in position, starting recovery")
            # think for a recovery here
        rospy.loginfo("actions finished")
        return

    def handle_actions(self, actions):
        if "move_eyes" in actions:
            array = []
            for data in actions['move_eyes']:
                array.append(data)
            rospy.loginfo("Moving eyes in position: ", array)
            data_to_send = Int8MultiArray()
            data_to_send.data = array
            goal = triskarone_msgs.msg.move_eyesGoal(goal=data_to_send)
            self.eyes_manager_client.send_goal(goal)
        if "move_body" in actions:
            array = []
            for data in actions['move_body']:
                array.append(data)
            rospy.loginfo("Moving body in position: ", array)
            data_to_send = Int8MultiArray()
            data_to_send.data = array
            goal = triskarone_msgs.msg.move_bodyGoal(goal=data_to_send)
            self.body_manager_client.send_goal(goal)
        if "speak" in actions:
            file_to_play = actions['speak']
            goal = triskarone_msgs.msg.play_audioGoal(filename=file_to_play)
            self.audio_client.send_goal(goal)
            rospy.loginfo("Playing audio: %s", file_to_play)
        if "move_base" in actions:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = actions['move_base'][0]
            goal.target_pose.pose.position.y = actions['move_base'][1]
            goal.target_pose.pose.orientation.w = actions['move_base'][2]
            self.move_base_client.send_goal(goal)
            rospy.loginfo("Moving to position: %f %f %f", actions['move_base'][0], actions['move_base'][1],
                          actions['move_base'][2])
        if "do_nothing" in actions:
            time = int(actions['do_nothing'])
            rospy.loginfo("Do nothing for %d seconds", time)
            rospy.sleep(time)
        if "end" in actions:
            rospy.loginfo("The show is finished, bye!")
            rospy.signal_shutdown("The show is finished")
        return


if __name__ == '__main__':
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    os.chdir("..")
    try:
        main_controller()
    except rospy.ROSInterruptException:
        pass
