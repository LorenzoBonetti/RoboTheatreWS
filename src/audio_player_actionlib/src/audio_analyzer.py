import audioop
import math

import pyaudio
import struct
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
print("hello")
CHUNK = 1024 * 4
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
p = pyaudio.PyAudio()
stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, output=True, frames_per_buffer=CHUNK)

while True:
    data = stream.read(CHUNK)
    rms = audioop.rms(data, 2)
    decibel = 20 * math.log10(rms)
    print(decibel)
