
#!/usr/bin/env python
 # license removed for brevity
import os
import json
import actionlib

import rospy
#import std_msgs
from std_msgs.msg import Int8MultiArray
from triskarone_msgs.msg import *

def read_trigger(data,section_number):
    section='section'+str(section_number)
    trigger=data[section]['trigger']
    trigger_type=data[section]['trigger_data']
    return trigger,trigger_type


def main_controller():
    eyes_pub = rospy.Publisher('arduino/eyes', Int8MultiArray, queue_size=10)
    speech_client = actionlib.SimpleActionClient('speech_monitor', triskarone_msgs.msg.speech_monitorAction)
    rospy.init_node('main_controller', anonymous=True)
    print("aaaa")
    speech_client.wait_for_server()
    goal = triskarone_msgs.msg.speech_monitorGoal(wait_time=2.5)

    speech_client.send_goal(goal)
    speech_client.wait_for_result()
    print(speech_client.get_result())
    f=open('scripts/script.json')
    data=json.load(f)
    while not rospy.is_shutdown():
        trigger, trigger_type=read_trigger(data, 1)
        print(trigger,trigger_type)
        continue

if __name__ == '__main__':
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    os.chdir("..")
    try:
        main_controller()
    except rospy.ROSInterruptException:
        pass