#!/usr/bin/env python
from __future__ import print_function

from audio_player_py.srv import *
import rospy

import pyaudio
import wave
import time
import os

audio_folder_path = ""


def play_audio(req):
    print(os.getcwd())
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    print(os.getcwd())
    os.chdir("..")
    print(os.getcwd())
    print("Returning %s" % req.file_name)
    # file = "audiofiles" + req.file_name
    file = "audio_files/blade_runner.wav"
    print("Reproducing:", file)
    wf = wave.open(file, 'rb')

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
        print("active")
        time.sleep(0.1)
    print("NOT ACTIVE")
    stream.stop_stream()
    stream.close()
    wf.close()

    p.terminate()

    x = True
    return bool(x)


def audio_player_server():
    rospy.init_node('audio_player_server')
    s = rospy.Service('audio_player', audio_player, play_audio)
    print("Audio Player Server ready")
    global audio_folder_path
    audio_folder_path = rospy.get_param("/audio_folder_path")
    print("FOLDER:", audio_folder_path)
    rospy.spin()


audio_player_server()
