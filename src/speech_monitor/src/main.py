import actionlib
import rospy

import pyaudio
import os
import audioop
import math
from triskarone_msgs.msg import *
import time
class TimerError(Exception):
    """A custom exception used to report errors in use of Timer class"""


class Timer_Class:
    def __init__(self):
        self._start_time = None
    def start(self):
        """Start a new timer"""
        if self._start_time is not None:
            raise TimerError(f"Timer is running. Use .stop() to stop it")
        self._start_time = time.perf_counter()
    def stop(self):
        """Stop the timer, and report the elapsed time"""
        if self._start_time is None:
            raise TimerError(f"Timer is not running. Use .start() to start it")
        self._start_time = None
    def elapsed_time(self):
        elapsed_time = time.perf_counter() - self._start_time
        return elapsed_time

class SpeechMonitorAction(object):
    # create messages that are used to publish feedback/result
    feedback = speech_monitorFeedback()
    result = speech_monitorResult()
    def __init__(self, name):
        # change working directory
        #
        #os.chdir("..")
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, speech_monitorAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        print("OOOO YES")

    def execute_cb(self, goal):
        CHUNK = 1024 * 4
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 44100
        isSpeaking=True
        time = goal.wait_time
        self.feedback.isSpeaking = isSpeaking
        # publish the feedback
        self._as.publish_feedback(self.feedback)
        #rimane fermo a non fare niente per tot tempo
        t=Timer_Class()
        t.start()
        while t.elapsed_time()<time:
            # publish the feedback
            self._as.publish_feedback(self.feedback)
            continue
        t.stop()


        p = pyaudio.PyAudio()
        stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, output=False, frames_per_buffer=CHUNK)
        not_speaking = 0
        while isSpeaking:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                stream.stop_stream()
                stream.close()
                break
            data = stream.read(CHUNK)
            rms = audioop.rms(data, 2)  # media quadratica dei dati
            decibel = 20 * math.log10(rms)  # trasforma in db
            if decibel < 65:
                not_speaking += 1
            else:
                not_speaking = 0
            if not_speaking > 5:
                isSpeaking=False
                print("qua")
                break
            else:
                print("qui")
                self._as.publish_feedback(self.feedback)

        stream.stop_stream()
        stream.close()
        p.terminate()

        self.feedback.isSpeaking=False
        self.result.hasFinished = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node("speech_monitor")
    server = SpeechMonitorAction(rospy.get_name())
    rospy.spin()