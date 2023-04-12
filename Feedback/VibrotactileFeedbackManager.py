# -----------------------------------------------------------------------
# Author:   Takumi Katagiri, Takumi Nishimura (Nagoya Institute of Technology)
# Created:  2021
# Summary:  振動フィードバック制御マネージャー
# -----------------------------------------------------------------------

import math
import winsound
from typing import List

import numpy as np
import pyaudio
from CyberneticAvatarMotionBehaviour import \
    CyberneticAvatarMotionBehaviour
from FileIO.FileIO import FileIO
from AvatarControl.MotionFilter import MotionFilter
from scipy import signal
from VibrotactileFeedback.AudioDeviceIndexes import AudioDeviceIndexes


class VibrotactileFeedbackManager:
    def __init__(self):
        """
        重い
        """
        self.posList = []
        fs = 180
        self.dt = round(1/fs, 4)

        # ----- listIndexNum from settings.csv ----- #
        fileIO = FileIO()
        dat = fileIO.Read('settings.csv',',')
        listIndexNum = [addr for addr in dat if 'listIndexNum' in addr[0]]
        self.listIndexNum = listIndexNum
        self.listIndexNum[0].remove('listIndexNum')
        listIndexNumstr = self.listIndexNum[0]
        listIndexNum = list(map(int,listIndexNumstr))

        # ----- Find audio device indexes ----- #
        audioDeviceIndexes = AudioDeviceIndexes()
        ListIndexNum = audioDeviceIndexes.Find(host_api='MME', name='スピーカー (3- Sound Blaster Play! 3')
        ListIndexNum = [10]
        OutputDeviceNum = len(ListIndexNum)

        self.p = pyaudio.PyAudio()
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.amp = 100
        self.freq = 200
        self.CHUNK = int(self.rate / self.freq)
        self.sin = np.sin(2.0 * np.pi * np.arange(self.CHUNK) * float(self.freq) / float(self.rate))
        self.square = signal.square(2.0 * np.pi * np.arange(self.CHUNK) * float(self.freq) / float(self.rate))

        # ----- Initialize the parameter of data_out according as OutputDeviceNum ----- #
        for i in range(OutputDeviceNum):
            data_out_command = 'self.data_out_' + str(i+1) + '= 0.0'
            exec(data_out_command)

        # ----- Define streamming command according as OutputDeviceNum ----- #
        for i in range(OutputDeviceNum):
            stream_command = 'self.stream' + str(i+1) + '= self.p.open('\
                + 'rate = self.rate,'\
                + 'channels = self.channels,'\
                + 'format = self.format,'\
                + 'output = True,'\
                + 'output_device_index = ListIndexNum[' + str(i) + '],'\
                + 'frames_per_buffer = self.CHUNK,'\
                + 'stream_callback = self.callback' + str(i+1)\
            + ')'
            exec(stream_command)

            start_streamming_command = 'self.stream' + str(i+1) + '.start_stream()'
            exec(start_streamming_command)

        self.data_out = 0

    def callback1(self, in_data, frame_count, time_info, status):
        out_data = (self.data_out*self.amp*self.sin).astype(np.int16)
        # out_data = ((503.0463*math.exp((self.data_out*self.amp*0.00008)/0.3752)-500) * self.sin).astype(np.int16)
        return (out_data, pyaudio.paContinue)

    def close(self):
        self.p.terminate()

    def velocityFeedback(self, position, Flag):
        self.posList.append(position)

        if len(self.posList) == 2:
            vel  = np.linalg.norm(np.diff(self.posList, n=1, axis=0) / self.dt)
            if Flag == True:
                self.data_out = vel
            else:
                self.data_out = 0

            del self.posList[0]

        else:
            pass
