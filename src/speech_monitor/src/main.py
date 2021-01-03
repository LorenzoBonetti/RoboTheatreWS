import sys

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


class suppress_stdout_stderr(object):
    '''
    A context manager for doing a "deep suppression" of stdout and stderr in
    Python, i.e. will suppress all print, even if the print originates in a
    compiled C/Fortran sub-function.
       This will not suppress raised exceptions, since exceptions are printed
    to stderr just before a script exits, and after the context manager has
    exited (at least, I think that is why it lets exceptions through).

    '''

    def __init__(self):
        # Open a pair of null files
        self.null_fds = [os.open(os.devnull, os.O_RDWR) for x in range(2)]
        # Save the actual stdout (1) and stderr (2) file descriptors.
        self.save_fds = [os.dup(1), os.dup(2)]

    def __enter__(self):
        # Assign the null pointers to stdout and stderr.
        os.dup2(self.null_fds[0], 1)
        os.dup2(self.null_fds[1], 2)

    def __exit__(self, *_):
        # Re-assign the real stdout/stderr back to (1) and (2)
        os.dup2(self.save_fds[0], 1)
        os.dup2(self.save_fds[1], 2)
        # Close all file descriptors
        for fd in self.null_fds + self.save_fds:
            os.close(fd)


class SpeechMonitorAction(object):
    # create messages that are used to publish feedback/result
    feedback = speech_monitorFeedback()
    result = speech_monitorResult()

    def __init__(self, name):
        os.chdir(os.path.dirname(os.path.abspath(__file__)))
        os.chdir("..")
        self._action_name = name
        self.threshold = rospy.get_param("/threshold")
        self.consecutive_silent_data = rospy.get_param("/consecutive_silent_data")
        self.chunk_dimension = rospy.get_param("/chunk_dimension")
        self.rate = rospy.get_param("/rate")

        self._as = actionlib.SimpleActionServer(self._action_name, speech_monitorAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        rospy.loginfo("%s is started", rospy.get_name())

    def execute_cb(self, goal):
        with suppress_stdout_stderr():
            CHUNK = self.chunk_dimension
            FORMAT = pyaudio.paInt16
            CHANNELS = 1
            RATE = self.rate
            isSpeaking = True
            time = goal.wait_time
            self.feedback.isSpeaking = isSpeaking
            # publish the feedback
            self._as.publish_feedback(self.feedback)
            # rimane fermo a non fare niente per tot tempo
            t = Timer_Class()
            t.start()
            while t.elapsed_time() < time:
                # publish the feedback
                self._as.publish_feedback(self.feedback)
                continue
            t.stop()

            p = pyaudio.PyAudio()
            stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, output=False,
                            frames_per_buffer=CHUNK)
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
                if decibel < self.threshold:
                    not_speaking += 1
                else:
                    not_speaking = 0
                if not_speaking > self.consecutive_silent_data:
                    isSpeaking = False
                    break
                else:
                    self._as.publish_feedback(self.feedback)

            stream.stop_stream()
            stream.close()
            p.terminate()

            self.feedback.isSpeaking = False
            self.result.hasFinished = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node("speech_monitor")
    server = SpeechMonitorAction(rospy.get_name())
    rospy.spin()
