#!/usr/bin/env python
import actionlib
import rospy

import pyaudio
import wave
import time
import os

from audio_player_actionlib.msg import *


def audio_player_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    print("1")
    client = actionlib.SimpleActionClient('audio_player_actionlib', audio_player_actionlib.msg.play_audioAction)
    print("2")
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print("3")
    # Creates a goal to send to the action server.
    goal = audio_player_actionlib.msg.play_audioGoal(filename="blade_runner.wav")
    print("4")
    # Sends the goal to the action server.
    client.send_goal(goal)
    print("5")
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print("6")
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


if __name__ == '__main__':
    print("hello")
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('audio_player_client_py')
        print("0")
        result = audio_player_client()
        
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
