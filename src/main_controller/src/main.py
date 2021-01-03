# !/usr/bin/env python
# license removed for brevity
import os
import json
import actionlib
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import rospy
# import std_msgs
from std_msgs.msg import Int8MultiArray
from triskarone_msgs.msg import *

isSpeaking = False


def read_trigger(data, section_number):
    section = 'section' + str(section_number)
    trigger = data[section]['trigger']
    trigger_data = data[section]['trigger_data']
    return trigger, trigger_data


def read_actions(data, section_number):
    section = 'section' + str(section_number)
    actions = data[section]['actions']
    return actions


def wait_for_trigger(trigger, trigger_data, speech_client):
    if trigger == "after_speech":
        goal = triskarone_msgs.msg.speech_monitorGoal(wait_time=trigger_data)
        rospy.loginfo("Waiting for others to speak")
        speech_client.send_goal(goal)
        speech_client.wait_for_result()
        return
    if trigger == "after_precedent":
        return
    if trigger == "after_command":
        # subscribes to joystick and wait
        return


def check_actions_status(actions, audio_client, move_base_client):
    has_to_speak = False
    move_base = False
    move_base_error = False
    if "speak" in actions:
        file_to_play = actions['speak']
        has_to_speak = True
    if "move_base" in actions:
        move_base = True
    while has_to_speak or move_base:
        if audio_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("audio_client has finished")
            has_to_speak = False
        if move_base_client.get_state() == GoalStatus.SUCCEEDED:
            move_base = False
            rospy.loginfo("Correctly moved to the desired position")
        if move_base_client.get_state() == GoalStatus.ABORTED:
            move_base = False
            move_base_error = True
        continue
    if move_base_error:
        rospy.loginfo("Failed to get in position, starting recovery")
        #think for a recovery here
    return


def handle_actions(actions, eyes_pub, body_pub, audio_client, move_base_client):
    if "move_eyes" in actions:
        rospy.loginfo("Moving eyes in position %d by speed %d", int(actions['move_eyes'][0]),
                      int(actions['move_eyes'][1]))
        array = [int(actions['move_eyes'][0]), int(actions['move_eyes'][1])]
        data_to_send = Int8MultiArray()
        data_to_send.data = array
        eyes_pub.publish(data_to_send)
    if "move_body" in actions:
        rospy.loginfo("Moving body in position %d by speed %d", int(actions['move_body'][0]),
                      int(actions['move_body'][1]))
        array = [int(actions['move_body'][0]), int(actions['move_body'][1])]
        data_to_send = Int8MultiArray()
        data_to_send.data = array
        body_pub.publish(data_to_send)
    if "speak" in actions:
        file_to_play = actions['speak']
        goal = triskarone_msgs.msg.play_audioGoal(filename=file_to_play)
        audio_client.send_goal(goal)
        rospy.loginfo("Playing audio: %s", file_to_play)
    if "move_base" in actions:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = actions['move_base'][0]
        goal.target_pose.pose.position.y = actions['move_base'][1]
        goal.target_pose.pose.orientation.w = actions['move_base'][2]
        move_base_client.send_goal(goal)
        rospy.loginfo("Moving to position: %f %f %f", actions['move_base'][0], actions['move_base'][1],
                      actions['move_base'][2])
    if "do_nothing" in actions:
        time = int(actions['do_nothing'])
        rospy.loginfo("Do nothing for %d seconds", time)
        rospy.sleep()
    if "end" in actions:
        rospy.loginfo("The show is finished, bye!")
        rospy.signal_shutdown("The show is finished")
    return


def main_controller():
    eyes_pub = rospy.Publisher('arduino/eyes', Int8MultiArray, queue_size=10)
    body_pub = rospy.Publisher('arduino/body', Int8MultiArray, queue_size=10)
    speech_client = actionlib.SimpleActionClient('speech_monitor', triskarone_msgs.msg.speech_monitorAction)
    audio_client = actionlib.SimpleActionClient('audio_player_actionlib', triskarone_msgs.msg.play_audioAction)
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.init_node('main_controller', anonymous=True)
    speech_client.wait_for_server()
    audio_client.wait_for_server()
    f = open('scripts/script.json')
    data = json.load(f)
    section_number = 1
    while not rospy.is_shutdown():
        rospy.loginfo("Starting section number %d", section_number)
        trigger, trigger_data = read_trigger(data, section_number)
        wait_for_trigger(trigger, trigger_data, speech_client)
        actions = read_actions(data, section_number)
        handle_actions(actions, eyes_pub, body_pub, audio_client, move_base_client)
        check_actions_status(actions, audio_client, move_base_client)
        section_number = section_number + 1


if __name__ == '__main__':
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    os.chdir("..")
    try:
        main_controller()
    except rospy.ROSInterruptException:
        pass
