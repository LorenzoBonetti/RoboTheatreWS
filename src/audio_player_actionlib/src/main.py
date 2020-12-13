#!/usr/bin/env python
import actionlib
import rospy

import pyaudio
import wave
import time
import os

from triskarone_msgs.msg import *


class AudioPlayerAction(object):
    # create messages that are used to publish feedback/result
    feedback = play_audioFeedback()
    result = play_audioResult()
    audio_folder_path = ""

    def __init__(self, name):
        # change working directory
        os.chdir(os.path.dirname(os.path.abspath(__file__)))
        os.chdir("..")
        #get audio folder path
        self.audio_folder_path = rospy.get_param("/audio_folder_path")

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, play_audioAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        file_to_play=self.audio_folder_path+goal.filename
        # publish info to the console for the user
        print(goal.filename)
        # start executing the action
        wf = wave.open(file_to_play, 'rb')

        p = pyaudio.PyAudio()

        def callback(in_data, frame_count, time_info, status):
            data = wf.readframes(frame_count)
            return (data, pyaudio.paContinue)

        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                channels=wf.getnchannels(),
                rate=wf.getframerate(),
                output=True,
                stream_callback=callback)

        stream.start_stream()

        while stream.is_active():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                stream.stop_stream()
                stream.close()
                wf.close()
                break
            self.feedback.hasFinished = False
            # publish the feedback
            self._as.publish_feedback(self.feedback)
            print("active")
            time.sleep(0.1)
        print("NOT ACTIVE")
        stream.stop_stream()
        stream.close()
        wf.close()
        p.terminate()

        if success:
            self.feedback.hasFinished=True
            self.result.response = self.feedback.hasFinished
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node("audio_player")
    server = AudioPlayerAction(rospy.get_name())
    rospy.spin()
